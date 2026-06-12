"""A1: монокулярный VO на FAST + Lucas-Kanade + Essential matrix.

Адаптер над рабочей реализацией из old_structure/src/vo/visual_odometry.py
под единый интерфейс VOAlgorithm. В отличие от старого класса:
* ядро (MonoFastLkCore) принимает уже декодированные кадры и не знает про
  файлы, видео, GT-файлы — это просто функция (prev_gray, curr_gray)
  → (R_rel, t_rel, success);
* адаптер MonoFastLk накапливает позу, делает scale-фактор, реализует
  heuristic gate и сохраняет историю предыдущей GT-позы для расчёта scale.

Scale-режим:
* если у кадра есть `frame.gt_pose` и был предыдущий gt_pose — scale =
  ||t_curr - t_prev|| (KITTI-стиль, как в старом коде);
* иначе — фиксированный `fixed_scale` из конструктора (по умолчанию 1.0).
"""
from __future__ import annotations

from typing import Optional, Tuple

import cv2
import numpy as np

from src.algorithms import register_algorithm
from src.vo.interface import (
    AlgorithmRequirements,
    Calibration,
    FrameData,
    Pose,
    VOAlgorithm,
)


# ---------------------------------------------------------------------------
# Core: чистая two-frame pose estimation, без I/O и scale
# ---------------------------------------------------------------------------
class MonoFastLkCore:
    """FAST + LK + Essential Matrix → relative R, t (без scale).

    Внутреннее состояние: текущий набор tracked-точек p0 и счётчик n_features.
    Когда фичей мало (< redetect_threshold), на следующем step() детектор
    запускается заново на prev_gray.
    """

    def __init__(
        self,
        focal: float,
        pp: Tuple[float, float],
        redetect_threshold: int = 2000,
        fast_threshold: int = 25,
        lk_params: Optional[dict] = None,
    ) -> None:
        self.focal = float(focal)
        self.pp = (float(pp[0]), float(pp[1]))
        self.redetect_threshold = int(redetect_threshold)

        self.detector = cv2.FastFeatureDetector_create(
            threshold=fast_threshold, nonmaxSuppression=True
        )
        self.lk_params = lk_params or dict(
            winSize=(21, 21),
            criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 30, 0.01),
        )

        self.p0: Optional[np.ndarray] = None
        self.n_features: int = 0

    def reset(self) -> None:
        self.p0 = None
        self.n_features = 0

    def _detect(self, img: np.ndarray) -> np.ndarray:
        kp = self.detector.detect(img)
        if not kp:
            return np.zeros((0, 1, 2), dtype=np.float32)
        return np.array([k.pt for k in kp], dtype=np.float32).reshape(-1, 1, 2)

    def step(
        self, prev_gray: np.ndarray, curr_gray: np.ndarray
    ) -> Tuple[Optional[np.ndarray], Optional[np.ndarray], bool]:
        """Один шаг: prev_gray → curr_gray → (R_rel, t_rel, success).

        R_rel, t_rel задают переход от системы prev_gray (камера i) к системе
        curr_gray (камера i+1) в семантике cv2.recoverPose. t_rel имеет
        unit-norm — реальный масштаб накладывает адаптер.
        """
        if self.n_features < self.redetect_threshold:
            self.p0 = self._detect(prev_gray)
            self.n_features = len(self.p0) if self.p0 is not None else 0

        if self.p0 is None or len(self.p0) < 10:
            return None, None, False

        p1, st, _err = cv2.calcOpticalFlowPyrLK(
            prev_gray, curr_gray, self.p0, None, **self.lk_params
        )
        if p1 is None or st is None:
            return None, None, False

        good_old = self.p0[st == 1]
        good_new = p1[st == 1]
        if len(good_old) < 10 or len(good_new) < 10:
            return None, None, False

        E, _mask = cv2.findEssentialMat(
            good_old, good_new, self.focal, self.pp, cv2.RANSAC, 0.999, 1.0, None
        )
        if E is None:
            self.p0 = good_new.reshape(-1, 1, 2)
            self.n_features = len(self.p0)
            return None, None, False

        _, R, t, _ = cv2.recoverPose(
            E, good_old, good_new, focal=self.focal, pp=self.pp, mask=None
        )

        self.p0 = good_new.reshape(-1, 1, 2)
        self.n_features = len(self.p0)
        return R, t, True


# ---------------------------------------------------------------------------
# Adapter: VOAlgorithm для стенда
# ---------------------------------------------------------------------------
@register_algorithm("A1")
class MonoFastLk(VOAlgorithm):
    """A1: Mono FAST+LK+Essential как VOAlgorithm.

    Args:
        fixed_scale: scale, который применяется, если у кадра нет gt_pose
            (т.е. в режиме без ground truth). Для KITTI Odometry с GT
            scale всегда берётся из gt_pose.
        redetect_threshold: re-detect FAST когда tracked features < threshold.
        fast_threshold: FAST detector threshold.
    """

    name = "A1"

    def __init__(
        self,
        fixed_scale: float = 1.0,
        redetect_threshold: int = 2000,
        fast_threshold: int = 25,
    ) -> None:
        self.fixed_scale = float(fixed_scale)
        self.redetect_threshold = int(redetect_threshold)
        self.fast_threshold = int(fast_threshold)

        self._calib: Optional[Calibration] = None
        self._core: Optional[MonoFastLkCore] = None
        self._prev_gray: Optional[np.ndarray] = None
        self._prev_gt_pose: Optional[Pose] = None
        self._abs: Pose = Pose.identity()
        self._processed: int = 0  # сколько кадров видел process(), включая первый

    # ------------------------------------------------------------------
    # VOAlgorithm API
    # ------------------------------------------------------------------
    def reset(self, calibration: Calibration) -> None:
        self._calib = calibration
        self._core = MonoFastLkCore(
            focal=calibration.focal_x(),
            pp=calibration.principal_point(),
            redetect_threshold=self.redetect_threshold,
            fast_threshold=self.fast_threshold,
        )
        self._prev_gray = None
        self._prev_gt_pose = None
        self._abs = Pose.identity()
        self._processed = 0

    def process(self, frame: FrameData) -> Optional[Pose]:
        if self._core is None:
            raise RuntimeError("MonoFastLk.reset() must be called before process()")

        # Кадр 0 — нечего сравнивать, возвращаем identity
        if self._prev_gray is None:
            self._prev_gray = frame.left
            self._prev_gt_pose = frame.gt_pose
            self._processed += 1
            return Pose(R=self._abs.R.copy(), t=self._abs.t.copy())

        R_cv, t_cv, ok = self._core.step(self._prev_gray, frame.left)
        if not ok or R_cv is None or t_cv is None:
            self._prev_gray = frame.left
            self._prev_gt_pose = frame.gt_pose
            self._processed += 1
            return None

        # Scale: GT-режим (если есть) или fixed
        scale = self.fixed_scale
        if frame.gt_pose is not None and self._prev_gt_pose is not None:
            scale = float(np.linalg.norm(frame.gt_pose.t - self._prev_gt_pose.t))

        # cv2.recoverPose возвращает (R, t) в семантике "переход от curr к prev
        # в системе curr": t — это направление от cam_curr к cam_prev в frame
        # cam_curr (single норма). Чтобы получить KITTI-style T_rel (поза
        # cam_curr в frame cam_prev), инвертируем SE(3): R_rel = R^T, t_rel = -R^T @ t.
        R_rel = R_cv.T
        t_rel = -R_cv.T @ t_cv

        # Heuristic gate: проверяем доминирование forward axis (на cv-формате
        # модули совпадают с KITTI-форматом, так что ось/знак не важны).
        t_x = float(t_cv[0, 0])
        t_y = float(t_cv[1, 0])
        t_z = float(t_cv[2, 0])
        forward_dominant = abs(t_z) > abs(t_x) and abs(t_z) > abs(t_y)

        # KITTI-композиция через Pose.compose(): T_abs_new = T_abs @ T_rel.
        # Первый успешный VO update берём как есть (без gate), как делал
        # old_structure — иначе на самом старте seq 04 теряли первый шаг.
        if self._processed < 2:
            self._abs = Pose(R=R_rel.copy(), t=(scale * t_rel).copy())
        elif scale > 0.1 and forward_dominant:
            rel_pose = Pose(R=R_rel, t=scale * t_rel)
            self._abs = self._abs.compose(rel_pose)

        # Обновление prev
        self._prev_gray = frame.left
        self._prev_gt_pose = frame.gt_pose
        self._processed += 1

        return Pose(R=self._abs.R.copy(), t=self._abs.t.copy())

    def requirements(self) -> AlgorithmRequirements:
        return AlgorithmRequirements(needs_stereo=False, needs_imu=False)
