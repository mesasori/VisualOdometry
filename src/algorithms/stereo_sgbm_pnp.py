"""A2: стерео VO на StereoSGBM + FAST + LK + 3D-2D PnP.

Hybrid-pipeline по варианту C большого плана:
* dense StereoSGBM на каждом стерео-кадре → disparity map левого изображения;
* FAST на левом → отбор фич, попавших в зоны валидной disparity;
* депроекция в 3D через f * baseline / disp + intrinsics;
* между кадрами — LK-tracking на левом (как в A1, переиспользуется через
  src.algorithms.base.LKTracker);
* solvePnPRansac(prev_pts_3d, curr_pts_2d, K) → R, t (cam_curr <- cam_prev),
  инверс SE(3) даёт T_prev_curr в KITTI-семантике;
* накопление через Pose.compose() (без ручной формулы как в A1).

Архитектурные решения для A2 (вынесены из stages/04_a1_implementation.md):
1. Heuristic gate из A1 (forward_dominant) НЕ переносится — у стерео есть
   истинный масштаб от baseline, движение в любую сторону должно копиться;
   гейт отрезал бы валидные update'ы на поворотах seq 05/08.
2. Используется Pose.compose() — единственный официальный SE(3)-композитор
   в проекте; A1 пока пишет композицию явно (исторически), A2/A3/A4
   мигрируют на compose() как стандарт.
3. После каждого успешного шага prev_pts_3d/2d полностью переинициализируются
   свежим набором FAST-фич на curr-кадре — избегаем накопления ошибок LK
   через много кадров (типичный приём в stereo VO туториалах).
4. SGBM работает в полноразмерном MODE_SGBM_3WAY (компромисс память/качество).
"""
from __future__ import annotations

from typing import Any, Dict, Optional, Tuple

import cv2
import numpy as np

from src.algorithms import register_algorithm
from src.algorithms.base import (
    FastDetector,
    LKTracker,
    pixels_to_3d,
    select_features_with_valid_disparity,
)
from src.vo.interface import (
    AlgorithmRequirements,
    Calibration,
    FrameData,
    Pose,
    VOAlgorithm,
)


# ---------------------------------------------------------------------------
# SGBM defaults
# ---------------------------------------------------------------------------
# Подобраны под KITTI grayscale stereo 1241x376, baseline ~0.54 м.
# numDisparities=128 покрывает дистанции от ~3 м (disp 128) до бесконечности
# (disp 1). blockSize=5 — компромисс плотности и шума. P1/P2 — каноническая
# формула из stereo_match.cpp (8 и 32 для grayscale). mode SGBM_3WAY — две
# проходки + 5-направленная динамика (быстрее MODE_HH, точнее MODE_SGBM).
DEFAULT_SGBM_PARAMS: Dict[str, Any] = dict(
    minDisparity=0,
    numDisparities=128,
    blockSize=5,
    P1=8 * 1 * 5 ** 2,
    P2=32 * 1 * 5 ** 2,
    disp12MaxDiff=1,
    uniquenessRatio=10,
    speckleWindowSize=100,
    speckleRange=2,
    preFilterCap=63,
    mode=cv2.STEREO_SGBM_MODE_SGBM_3WAY,
)


# ---------------------------------------------------------------------------
# Core: чистая stereo-pose-estimation, без I/O и без Pose/Calibration
# ---------------------------------------------------------------------------
class StereoSGBMPnPCore:
    """SGBM + FAST + LK + solvePnPRansac → relative R, t (cam_prev <- cam_curr).

    Состояние:
        prev_left:    (H, W) uint8 — последний обработанный левый кадр;
        prev_pts_2d:  (N, 1, 2) float32 — 2D-точки на prev_left, у которых
                      есть валидная disparity (использовались при триангуляции);
        prev_pts_3d:  (N, 3) float32 — 3D-координаты этих точек в frame cam_prev
                      (Z = forward).

    Все «низкие» порты алгоритма (мин. кол-во inliers, RANSAC error и т.п.)
    параметризуются через __init__, чтобы экспериментировать без правки кода.
    """

    def __init__(
        self,
        K: np.ndarray,
        baseline_m: float,
        sgbm_params: Optional[Dict[str, Any]] = None,
        fast_threshold: int = 25,
        min_disparity_value: float = 1.0,
        max_disparity_value: Optional[float] = None,
        min_features_for_pnp: int = 8,
        pnp_reprojection_error: float = 2.0,
        pnp_iterations: int = 200,
        pnp_confidence: float = 0.99,
        min_inliers_ratio: float = 0.3,
    ) -> None:
        self.K = np.asarray(K, dtype=np.float64).reshape(3, 3)
        self.baseline_m = float(baseline_m)
        if self.baseline_m <= 0:
            raise ValueError(f"baseline_m must be > 0, got {self.baseline_m}")

        params = dict(DEFAULT_SGBM_PARAMS)
        if sgbm_params:
            params.update(sgbm_params)
        self.sgbm = cv2.StereoSGBM_create(**params)
        self._sgbm_params = params

        self.detector = FastDetector(threshold=fast_threshold)
        self.tracker = LKTracker()

        self.min_disparity_value = float(min_disparity_value)
        self.max_disparity_value = (
            float(max_disparity_value) if max_disparity_value is not None else None
        )
        self.min_features_for_pnp = int(min_features_for_pnp)
        self.pnp_reprojection_error = float(pnp_reprojection_error)
        self.pnp_iterations = int(pnp_iterations)
        self.pnp_confidence = float(pnp_confidence)
        self.min_inliers_ratio = float(min_inliers_ratio)

        self.prev_left: Optional[np.ndarray] = None
        self.prev_pts_2d: Optional[np.ndarray] = None
        self.prev_pts_3d: Optional[np.ndarray] = None

    def reset(self) -> None:
        self.prev_left = None
        self.prev_pts_2d = None
        self.prev_pts_3d = None

    # ------------------------------------------------------------------
    # Внутренние утилиты
    # ------------------------------------------------------------------
    def _compute_disparity(self, left: np.ndarray, right: np.ndarray) -> np.ndarray:
        """StereoSGBM возвращает int16 с фиксированной точкой Q4 — делим на 16."""
        disp_raw = self.sgbm.compute(left, right)
        return disp_raw.astype(np.float32) / 16.0

    def _build_features(
        self, left: np.ndarray, right: np.ndarray
    ) -> Tuple[Optional[np.ndarray], Optional[np.ndarray]]:
        """Дает (pts_2d, pts_3d) или (None, None), если фич мало."""
        disp = self._compute_disparity(left, right)
        kp_2d = self.detector.detect(left)
        if len(kp_2d) == 0:
            return None, None

        kept_2d, disp_vals = select_features_with_valid_disparity(
            kp_2d,
            disp,
            min_disp=self.min_disparity_value,
            max_disp=self.max_disparity_value,
        )
        if len(kept_2d) < self.min_features_for_pnp:
            return None, None

        pts_3d = pixels_to_3d(kept_2d, disp_vals, self.K, self.baseline_m)
        return kept_2d, pts_3d

    # ------------------------------------------------------------------
    # Основной шаг
    # ------------------------------------------------------------------
    def step(
        self, left_curr: np.ndarray, right_curr: np.ndarray
    ) -> Tuple[Optional[np.ndarray], Optional[np.ndarray], bool]:
        """Один шаг: (left_curr, right_curr) → (R_rel, t_rel, success).

        Семантика результата: R_rel (3x3), t_rel (3x1) задают T_prev_curr,
        то есть позу кадра curr в системе кадра prev (KITTI-композиция:
        T_abs_new = T_abs @ T_rel переводит curr-кадр в frame 0).

        Возвращает (None, None, False), если шаг не удался — нет prev,
        мало tracked-точек, RANSAC провалился, мало inliers. Состояние
        Core продвигается в любом случае (prev_left обновляется на curr,
        фичи переинициализируются), чтобы следующий шаг мог продолжить.
        """
        # Шаг 0: нет prev — только наполняем состояние из curr.
        if self.prev_left is None:
            pts_2d, pts_3d = self._build_features(left_curr, right_curr)
            if pts_2d is None:
                # фичей мало — но prev_left всё равно обновим, чтобы
                # следующий кадр имел chance
                self.prev_left = left_curr
                return None, None, False
            self.prev_left = left_curr
            self.prev_pts_2d = pts_2d
            self.prev_pts_3d = pts_3d
            return None, None, False

        # Шаг 1: если prev_pts отсутствуют (предыдущий шаг провалил build) —
        # пробуем построить на текущем стерео-парe и пропускаем PnP.
        if self.prev_pts_2d is None or self.prev_pts_3d is None or len(self.prev_pts_2d) == 0:
            pts_2d, pts_3d = self._build_features(left_curr, right_curr)
            self.prev_left = left_curr
            if pts_2d is not None:
                self.prev_pts_2d = pts_2d
                self.prev_pts_3d = pts_3d
            else:
                self.prev_pts_2d = None
                self.prev_pts_3d = None
            return None, None, False

        # Шаг 2: LK от prev_left → left_curr
        _good_prev, good_curr, status = self.tracker.track(
            self.prev_left, left_curr, self.prev_pts_2d
        )
        mask = status == 1
        # 3D-точки берём по тому же mask из prev_pts_3d (они синхронизированы)
        prev_3d_tracked = self.prev_pts_3d[mask]
        curr_2d_tracked = good_curr.reshape(-1, 2)

        ok = False
        R_rel: Optional[np.ndarray] = None
        t_rel: Optional[np.ndarray] = None

        if len(prev_3d_tracked) >= self.min_features_for_pnp:
            # solvePnPRansac ожидает objectPoints (N,3), imagePoints (N,2),
            # cameraMatrix (3x3) float64, distCoeffs=None для rectified.
            obj_pts = prev_3d_tracked.astype(np.float64)
            img_pts = curr_2d_tracked.astype(np.float64)
            try:
                ret, rvec, tvec, inliers = cv2.solvePnPRansac(
                    obj_pts,
                    img_pts,
                    self.K,
                    distCoeffs=None,
                    iterationsCount=self.pnp_iterations,
                    reprojectionError=self.pnp_reprojection_error,
                    confidence=self.pnp_confidence,
                    flags=cv2.SOLVEPNP_ITERATIVE,
                )
            except cv2.error:
                ret, rvec, tvec, inliers = False, None, None, None

            n_inliers = 0 if inliers is None else len(inliers)
            min_inliers = max(
                self.min_features_for_pnp,
                int(self.min_inliers_ratio * len(obj_pts)),
            )
            if ret and rvec is not None and tvec is not None and n_inliers >= min_inliers:
                # solvePnP даёт rvec/tvec такие, что
                #   pts_curr_cam = R_cv @ pts_prev_world + tvec
                # где «world» здесь = frame cam_prev (наша objectPoints).
                # Это transform cam_prev → cam_curr. В KITTI-семантике
                # абсолютная поза T_abs (frame cam_k в frame cam_0)
                # обновляется как T_abs_new = T_abs @ T_prev_curr.
                # Поэтому нужен инверс: R_rel = R_cv.T, t_rel = -R_cv.T @ tvec.
                R_cv, _ = cv2.Rodrigues(rvec)
                R_rel = R_cv.T
                t_rel = -R_cv.T @ tvec
                ok = True

        # Шаг 3: переинициализация состояния свежим набором фич на curr.
        # Делаем независимо от исхода PnP — даже если шаг провалился,
        # хотим попробовать на следующем кадре.
        pts_2d_new, pts_3d_new = self._build_features(left_curr, right_curr)
        self.prev_left = left_curr
        if pts_2d_new is not None:
            self.prev_pts_2d = pts_2d_new
            self.prev_pts_3d = pts_3d_new
        else:
            self.prev_pts_2d = None
            self.prev_pts_3d = None

        return R_rel, t_rel, ok


# ---------------------------------------------------------------------------
# Adapter: VOAlgorithm для стенда
# ---------------------------------------------------------------------------
@register_algorithm("A2")
class StereoSGBMPnP(VOAlgorithm):
    """A2: Stereo SGBM + FAST + LK + 3D-2D PnP, обёрнутый как VOAlgorithm.

    Args:
        sgbm_params: переопределяет DEFAULT_SGBM_PARAMS (можно отдать
            частичный dict — будет смержен с дефолтом).
        fast_threshold: FAST detector threshold (по умолчанию 25).
        min_disparity_value / max_disparity_value: фильтрация фич по
            disparity. min=1.0 убирает «бесконечно далёкие» точки (numerical
            inf depth). max=None — без верхнего порога.
        min_features_for_pnp: минимальное число tracked-точек, чтобы
            запускать solvePnPRansac.
        pnp_*: параметры RANSAC внутри solvePnPRansac.
        min_inliers_ratio: доля inliers от tracked-точек, ниже которой
            результат считается ненадёжным (отбрасывается, кадр идёт как
            «потеря трекинга» в runner).
    """

    name = "A2"

    def __init__(
        self,
        sgbm_params: Optional[Dict[str, Any]] = None,
        fast_threshold: int = 25,
        min_disparity_value: float = 1.0,
        max_disparity_value: Optional[float] = None,
        min_features_for_pnp: int = 8,
        pnp_reprojection_error: float = 2.0,
        pnp_iterations: int = 200,
        pnp_confidence: float = 0.99,
        min_inliers_ratio: float = 0.3,
    ) -> None:
        self.sgbm_params = sgbm_params
        self.fast_threshold = int(fast_threshold)
        self.min_disparity_value = float(min_disparity_value)
        self.max_disparity_value = (
            float(max_disparity_value) if max_disparity_value is not None else None
        )
        self.min_features_for_pnp = int(min_features_for_pnp)
        self.pnp_reprojection_error = float(pnp_reprojection_error)
        self.pnp_iterations = int(pnp_iterations)
        self.pnp_confidence = float(pnp_confidence)
        self.min_inliers_ratio = float(min_inliers_ratio)

        self._calib: Optional[Calibration] = None
        self._core: Optional[StereoSGBMPnPCore] = None
        self._abs_pose: Pose = Pose.identity()

    # ------------------------------------------------------------------
    # VOAlgorithm API
    # ------------------------------------------------------------------
    def reset(self, calibration: Calibration) -> None:
        if not calibration.has_stereo():
            raise ValueError(
                "A2 (StereoSGBMPnP) requires stereo calibration "
                "(K_right + baseline_m). Got mono-only calibration."
            )
        self._calib = calibration
        self._core = StereoSGBMPnPCore(
            K=calibration.K_left,
            baseline_m=float(calibration.baseline_m),
            sgbm_params=self.sgbm_params,
            fast_threshold=self.fast_threshold,
            min_disparity_value=self.min_disparity_value,
            max_disparity_value=self.max_disparity_value,
            min_features_for_pnp=self.min_features_for_pnp,
            pnp_reprojection_error=self.pnp_reprojection_error,
            pnp_iterations=self.pnp_iterations,
            pnp_confidence=self.pnp_confidence,
            min_inliers_ratio=self.min_inliers_ratio,
        )
        self._abs_pose = Pose.identity()

    def process(self, frame: FrameData) -> Optional[Pose]:
        if self._core is None:
            raise RuntimeError("StereoSGBMPnP.reset() must be called before process()")
        if frame.right is None:
            raise ValueError(
                f"A2 requires stereo right image (frame {frame.index} has None)"
            )

        R_rel, t_rel, ok = self._core.step(frame.left, frame.right)

        # Первый кадр (или кадр сразу после инициализации) — Core возвращает
        # (None, None, False), но это не «потеря трекинга»: алгоритм просто
        # ещё не имел двух последовательных кадров. Возвращаем identity.
        if not ok:
            # Если ещё не было ни одного успешного шага — это инициализация,
            # отдаём identity (как делает A1 на кадре 0).
            if (self._abs_pose.R == np.eye(3)).all() and not np.any(self._abs_pose.t):
                return Pose(R=self._abs_pose.R.copy(), t=self._abs_pose.t.copy())
            # Иначе это реальная потеря трекинга — None даёт runner'у право
            # повторить последнюю успешную позу и инкрементировать счётчик.
            return None

        rel = Pose(R=R_rel, t=t_rel)
        self._abs_pose = self._abs_pose.compose(rel)
        return Pose(R=self._abs_pose.R.copy(), t=self._abs_pose.t.copy())

    def requirements(self) -> AlgorithmRequirements:
        return AlgorithmRequirements(needs_stereo=True, needs_imu=False)
