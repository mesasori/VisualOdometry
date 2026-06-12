"""A4: Stereo + IMU через loosely-coupled EKF (Visual-Inertial Odometry).

Архитектура (loosely-coupled stereo VIO)::

    Frame_i:                       Frame_{i+1}:
    ┌────────────────┐             ┌────────────────┐
    │ left + right   │──┐          │ left + right   │──┐
    │ IMU bursts     │  │          │ IMU bursts     │  │
    └────────────────┘  │          └────────────────┘  │
            │            │                  │           │
            │            ▼                  │           ▼
            │  ┌──────────────────┐         │  ┌──────────────────┐
            │  │ EKF.predict(imu) │         │  │ EKF.predict(imu) │
            │  │  (p, q, v)       │         │  │  (p, q, v)       │
            │  └──────────────────┘         │  └──────────────────┘
            │            │                  │           │
            ▼            │                  ▼           │
    ┌──────────────────┐│           ┌──────────────────┐│
    │ StereoSGBMPnP    ││           │ StereoSGBMPnP    ││
    │ R_rel, t_rel     ││           │ R_rel, t_rel     ││
    │ (METRIC scale)   ││           │ (METRIC scale)   ││
    └──────────────────┘│           └──────────────────┘│
            │            │                  │           │
            ▼            │                  ▼           │
    ┌──────────────────────────────────────────────────────┐
    │ T_rel  = (R_rel, t_rel)  -- уже metric             │
    │ T_w_cam_meas = T_w_cam_meas_{i-1} ∘ T_rel           │
    │ T_w_imu_meas = T_w_cam_meas ∘ T_cam_imu             │
    └──────────────────────────────────────────────────────┘
            │
            ▼
    ┌──────────────────────────────────────────┐
    │ EKF.update_pose(T_w_imu_meas, R_meas)    │
    │   (Joseph form, manifold update δθ_w)    │
    └──────────────────────────────────────────┘
            │
            ▼
    T_w_cam = EKF.get_pose(T_imu_cam) → Pose

Архитектурные решения (зафиксированы в [stages/08_a4_implementation.md](../../stages/08_a4_implementation.md)):

1. **GT-инициализация первого кадра** (same_as_a3): на frame 0 берётся
   `frame.gt_pose` (по построению KittiRaw — identity в системе cam0[0]) для
   T_w_cam_0, на frame 1 уточняется `v_0 = (gt[1].t - gt[0].t)/dt` (one-shot
   доступ к gt, после этого алгоритм работает без подсматривания). Это
   позволяет напрямую сравнивать A4 и A3 на одинаковых начальных условиях.

2. **НЕТ scale per-step из IMU** (в отличие от A3): A2 даёт `t_rel` с
   metric translation сразу через depth от disparity. Никаких
   ‖p_predict − p_meas_prev‖ для масштаба не нужно — это и было главной
   причиной дрейфа A3 (mono-VIO без global anchor).

3. **НЕТ forward-axis gate** (в отличие от A3): у стерео-PnP нет проблемы
   шумных направлений на остановках/боковом движении, она физически
   измерима через 3D-2D соответствия. Outliers отсекаются `min_inliers_ratio`
   внутри `StereoSGBMPnPCore.step` (RANSAC). Поэтому в A4 — никаких
   эвристик поверх Core.

4. **Composition напрямую через Pose.compose()**: как в A2 в одиночку
   (см. [stereo_sgbm_pnp.py](stereo_sgbm_pnp.py) `process()`).

5. **Constant R_meas (sigma_p, sigma_theta)**: дефолты подобраны после
   sweep на drive_0018 (см. stages/08). Стерео даёт более точное
   измерение, чем mono-VIO с scale-from-IMU, поэтому sigma_p_meas ниже
   чем у A3 (default A3 = 0.1 м; A4 — тоже 0.1 м после sweep, см. ниже).
   Adaptive R_meas через n_inliers PnP — open question, проверяется
   в Phase 5 этапа.

6. **Frame conventions** (повторение [stages/07_a3_implementation.md](../../stages/07_a3_implementation.md)):
   world frame = cam0[0] (та же система, что у GT KittiRaw и KITTI Odometry
   poses). В этой системе ось y направлена вниз (cam convention), поэтому
   gravity в EKF задаётся как (0, +9.81, 0) — это позволяет T_w_cam от EKF
   напрямую совпадать с GT без дополнительных преобразований.
"""
from __future__ import annotations

from typing import Any, Dict, Optional

import numpy as np

from src.algorithms import register_algorithm
from src.algorithms.stereo_sgbm_pnp import StereoSGBMPnPCore
from src.fusion.ekf import EKF
from src.fusion.imu_model import ImuState, _R_to_quat
from src.vo.interface import (
    AlgorithmRequirements,
    Calibration,
    FrameData,
    Pose,
    VOAlgorithm,
)


@register_algorithm("A4")
class StereoImuEKF(VOAlgorithm):
    """A4: Stereo SGBM + FAST + LK + PnP + IMU через loosely-coupled EKF.

    Args:
        sigma_a: continuous-time PSD акселерометра (м/с² / √Hz).
        sigma_g: continuous-time PSD гироскопа (рад/с / √Hz).
        sigma_ba: random walk bias акселерометра.
        sigma_bg: random walk bias гироскопа.
        sigma_p_meas: σ позиционного измерения от VO (м); входит в R_meas.
        sigma_theta_meas: σ ориентационного измерения от VO (рад); входит в R_meas.
        initial_p_bias_acc: начальная диагональ P для bias акселерометра.
        initial_p_bias_gyro: начальная диагональ P для bias гироскопа.
        sgbm_params: переопределение DEFAULT_SGBM_PARAMS A2 (см. stereo_sgbm_pnp.py).
        fast_threshold: порог FAST detector.
        min_disparity_value / max_disparity_value: фильтр фич по disparity.
        min_features_for_pnp: минимальное число tracked-точек для solvePnPRansac.
        pnp_*: параметры RANSAC внутри solvePnPRansac.
        min_inliers_ratio: доля inliers от tracked-точек, ниже которой step
            считается неудачным (ok=False → EKF идёт в predict-only fallback).

    Дефолты σ_p_meas=0.02 / σ_theta_meas=0.01 подобраны после sweep на
    drive_0018 (см. stages/08_a4_implementation.md, раздел Phase 3). σ_p_meas
    у A4 в 5× меньше A3 default (0.1) — это значит EKF доверяет стерео-VO
    сильнее, чем mono+IMU-scale: стерео-PnP даёт metric translation с
    sub-decimeter точностью на каждом кадре, в то время как mono-scale из
    IMU прогрессирует с накопленной IMU-ошибкой. На drive_0018 это даёт
    ATE 6.70 м (vs A2 = 9.76 м, ×30% улучшение от добавления IMU).
    """

    name = "A4"

    def __init__(
        self,
        sigma_a: float = 0.05,
        sigma_g: float = 0.005,
        sigma_ba: float = 1e-8,
        sigma_bg: float = 1e-8,
        sigma_p_meas: float = 0.02,
        sigma_theta_meas: float = 0.01,
        initial_p_bias_acc: float = 1e-6,
        initial_p_bias_gyro: float = 1e-6,
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
        self.sigma_a = float(sigma_a)
        self.sigma_g = float(sigma_g)
        self.sigma_ba = float(sigma_ba)
        self.sigma_bg = float(sigma_bg)
        self.sigma_p_meas = float(sigma_p_meas)
        self.sigma_theta_meas = float(sigma_theta_meas)
        self.initial_p_bias_acc = float(initial_p_bias_acc)
        self.initial_p_bias_gyro = float(initial_p_bias_gyro)

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
        self._ekf: Optional[EKF] = None
        self._T_imu_cam: Optional[np.ndarray] = None
        self._T_cam_imu: Optional[np.ndarray] = None
        # Последняя VO-измеренная T_w_cam (база для composite next step)
        self._T_w_cam_meas: Optional[np.ndarray] = None
        # Последняя оценённая T_w_cam, выходящая наружу
        self._last_pose_out: Pose = Pose.identity()
        # Для оценки v0
        self._frame0_gt: Optional[Pose] = None
        self._frame0_ts: Optional[float] = None
        # Для dt в predict (frame_ts - prev_processed_ts)
        self._prev_ts: Optional[float] = None
        # State machine
        self._initialized: bool = False
        # R_meas пересобирается в reset()
        self._R_meas: Optional[np.ndarray] = None

    # ------------------------------------------------------------------
    # VOAlgorithm API
    # ------------------------------------------------------------------
    def reset(self, calibration: Calibration) -> None:
        if not calibration.has_stereo():
            raise ValueError(
                "A4 (StereoImuEKF) requires stereo calibration "
                "(K_right + baseline_m). Got mono-only calibration."
            )
        if calibration.T_imu_cam is None:
            raise ValueError(
                "A4 requires Calibration.T_imu_cam; "
                "dataset must provide IMU↔camera extrinsic"
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
        # World frame для A4 = cam0[0] (та же система, что у GT KittiRaw и
        # KITTI Odometry poses). В этой системе ось y направлена вниз
        # (cam convention), поэтому гравитация в world frame = (0, +g, 0),
        # а не дефолтная ENU-конвенция (0, 0, -g) из imu_model.GRAVITY_WORLD.
        # Без этой правки EKF расходится ×40 (тот же эффект что в A3, см.
        # [stages/07_a3_implementation.md](../../stages/07_a3_implementation.md), раздел frame conventions).
        self._ekf = EKF(
            sigma_a=self.sigma_a,
            sigma_g=self.sigma_g,
            sigma_ba=self.sigma_ba,
            sigma_bg=self.sigma_bg,
            gravity=np.array([0.0, 9.81, 0.0], dtype=np.float64),
        )
        self._T_imu_cam = np.asarray(calibration.T_imu_cam, dtype=np.float64).reshape(4, 4)
        self._T_cam_imu = np.linalg.inv(self._T_imu_cam)
        self._T_w_cam_meas = None
        self._last_pose_out = Pose.identity()
        self._frame0_gt = None
        self._frame0_ts = None
        self._prev_ts = None
        self._initialized = False
        # R_meas: диагональ из σ позиции и σ ориентации (constant per-step).
        self._R_meas = np.diag(
            np.array(
                [
                    self.sigma_p_meas ** 2,
                    self.sigma_p_meas ** 2,
                    self.sigma_p_meas ** 2,
                    self.sigma_theta_meas ** 2,
                    self.sigma_theta_meas ** 2,
                    self.sigma_theta_meas ** 2,
                ],
                dtype=np.float64,
            )
        )

    def process(self, frame: FrameData) -> Optional[Pose]:
        if self._core is None or self._ekf is None or self._T_imu_cam is None:
            raise RuntimeError("StereoImuEKF.reset() must be called before process()")
        if frame.right is None:
            raise ValueError(
                f"A4 requires stereo right image (frame {frame.index} has None)"
            )

        # ------------------------------------------------------------------
        # Кадр 0 — инициализация EKF из gt[0] (если есть). Скорость пока
        # неизвестна (узнаем на кадре 1 из gt[1]-gt[0]). До этого core тоже
        # «прогревается»: первый step() стерео-PnP лишь набивает prev-state
        # и сам по себе возвращает ok=False — так что VO-update пропустим
        # естественно.
        # ------------------------------------------------------------------
        if not self._initialized:
            self._init_ekf(frame)
            # Прогон stereo core на первом кадре — нужен, чтобы prev_pts_3d
            # был построен и следующий шаг мог сразу выдать PnP. Возврат
            # игнорируется (core всё равно вернёт (None, None, False) на 1-м).
            self._core.step(frame.left, frame.right)
            self._frame0_gt = frame.gt_pose
            self._frame0_ts = frame.timestamp
            # prev_ts = ts кадра 0 — на frame 1 даст корректный dt = ~0.1 с
            # для первого EKF.predict (между frame 0 и frame 1).
            self._prev_ts = frame.timestamp
            self._T_w_cam_meas = self._ekf_pose_in_cam_frame()
            self._last_pose_out = Pose.from_matrix(self._T_w_cam_meas)
            return self._last_pose_out

        # ------------------------------------------------------------------
        # Frame 1: уточняем v0 ДО первой predict-итерации. Без этого
        # propagate начинает с v=0, что для стерео фатально менее критично
        # (metric scale всё равно восстанавливается мгновенно из VO update),
        # но для apples-to-apples сравнения с A3 — оставляем такой же init.
        # Это единственный «второй gt-доступ», после которого A4 работает
        # без подсматривания.
        # ------------------------------------------------------------------
        if (
            self._frame0_gt is not None
            and frame.gt_pose is not None
            and self._frame0_ts is not None
        ):
            dt01 = frame.timestamp - self._frame0_ts
            if dt01 > 1e-6:
                v0_world = (
                    np.asarray(frame.gt_pose.t).reshape(3)
                    - np.asarray(self._frame0_gt.t).reshape(3)
                ) / dt01
                # v_imu_world ≈ v_cam_world; ω × r-поправка < 1%.
                self._ekf.state.v = v0_world.astype(np.float64)
            self._frame0_gt = None  # one-shot

        # ------------------------------------------------------------------
        # Predict: пропагация EKF по IMU-семплам кадра (sync-режим = 1 семпл).
        # dt = frame.timestamp - prev_processed_timestamp. На frame 1 prev_ts
        # ещё None (выставляли только _frame0_ts на init), но он совпадает с
        # _frame0_ts, поэтому корректно начинаем с разности.
        # ------------------------------------------------------------------
        if self._prev_ts is None:
            self._prev_ts = frame.timestamp
        dt_frame = float(frame.timestamp - self._prev_ts)
        if frame.imu:
            n_imu = len(frame.imu)
            dt_step = dt_frame / max(n_imu, 1)
            for imu_sample in frame.imu:
                self._ekf.predict(imu_sample, dt_step)
        self._prev_ts = frame.timestamp

        # ------------------------------------------------------------------
        # VO step: стерео-PnP даёт metric R_rel, t_rel (KITTI-композиция
        # T_abs_new = T_abs @ T_rel). Возвращает ok=False, если LK / RANSAC
        # не дали достаточно inliers; тогда — predict-only fallback.
        # ------------------------------------------------------------------
        R_rel, t_rel, ok = self._core.step(frame.left, frame.right)

        if not ok or R_rel is None or t_rel is None:
            # VO потеряла трекинг — публикуем pose из EKF (predict-only).
            # Возвращаем НЕ-None, потому что у нас есть оценка от IMU.
            self._last_pose_out = Pose.from_matrix(self._ekf_pose_in_cam_frame())
            return self._last_pose_out

        # ------------------------------------------------------------------
        # Composition в абсолютную VO-позу: metric scale уже учтён в t_rel.
        # Никаких gates (forward-axis / min_scale) и никакого scale-per-step
        # из IMU prediction (в отличие от A3) — стерео самодостаточно.
        # ------------------------------------------------------------------
        T_rel = np.eye(4, dtype=np.float64)
        T_rel[:3, :3] = R_rel
        T_rel[:3, 3] = np.asarray(t_rel, dtype=np.float64).reshape(3)

        T_w_cam_meas_new = self._T_w_cam_meas @ T_rel
        T_w_imu_meas = T_w_cam_meas_new @ self._T_cam_imu
        pose_meas_imu = Pose.from_matrix(T_w_imu_meas)

        # ------------------------------------------------------------------
        # EKF update. После update сохраняем T_w_cam_meas (поза cam в world
        # после ekf-коррекции) как базу для следующего composition.
        # ------------------------------------------------------------------
        self._ekf.update_pose(pose_meas_imu, self._R_meas)
        T_w_cam_after_update = self._ekf_pose_in_cam_frame()
        self._T_w_cam_meas = T_w_cam_after_update

        self._last_pose_out = Pose.from_matrix(T_w_cam_after_update)
        return self._last_pose_out

    def requirements(self) -> AlgorithmRequirements:
        return AlgorithmRequirements(needs_stereo=True, needs_imu=True)

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------
    def _init_ekf(self, frame: FrameData) -> None:
        """Инициализация EKF state по первому кадру.

        Если у frame.gt_pose есть значение — T_w_cam_0 = gt[0] (по построению
        KittiRaw — identity в системе cam0[0]), T_w_imu_0 = T_w_cam_0 @ T_cam_imu.
        Иначе T_w_imu_0 = identity (warning: первый VO update задаст реальную
        позу, но v будет восстанавливаться 1-2 кадра дольше).
        """
        if frame.gt_pose is not None:
            T_w_cam_0 = frame.gt_pose.to_matrix()
        else:
            T_w_cam_0 = np.eye(4, dtype=np.float64)
        T_w_imu_0 = T_w_cam_0 @ self._T_cam_imu
        p0 = T_w_imu_0[:3, 3]
        R0 = T_w_imu_0[:3, :3]
        q0 = _R_to_quat(R0)
        init_state = ImuState(
            p=p0,
            q=q0,
            v=np.zeros(3, dtype=np.float64),
            ba=np.zeros(3, dtype=np.float64),
            bg=np.zeros(3, dtype=np.float64),
        )
        # Initial P: позиция почти точная (gt), ориентация в пределах
        # миллиметрового шума, скорость нулевая (быстро сходится на frame 1
        # из v0_world корректировки). Bias очень узкая полоса (1e-6) — как у
        # A3; на KITTI с почти постоянной скоростью это снижает scale drift.
        initial_P_diag = np.array(
            [
                1e-6, 1e-6, 1e-6,                                          # δp
                1e-4, 1e-4, 1e-4,                                          # δθ
                0.25, 0.25, 0.25,                                          # δv (0.5 м/с std)
                self.initial_p_bias_acc, self.initial_p_bias_acc, self.initial_p_bias_acc,    # δb_a
                self.initial_p_bias_gyro, self.initial_p_bias_gyro, self.initial_p_bias_gyro,  # δb_g
            ],
            dtype=np.float64,
        )
        self._ekf.reset(init_state, initial_P_diag=initial_P_diag)
        self._initialized = True

    def _ekf_pose_in_cam_frame(self) -> np.ndarray:
        """Конвертирует текущее EKF-состояние в T_w_cam (4x4)."""
        pose = self._ekf.get_pose(T_imu_cam=self._T_imu_cam)
        return pose.to_matrix()
