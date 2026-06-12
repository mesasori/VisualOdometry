"""Общие хелперы для алгоритмов VO (детектирование, tracking, stereo helpers).

Эти утилиты переиспользуются между mono (A1, A6) и stereo (A2) алгоритмами,
а также понадобятся для A3/A4 (mono+IMU / stereo+IMU) на Этапах 6-7.

Сейчас живут как тонкие обёртки над cv2:
* FastDetector — единая обёртка над cv2.FastFeatureDetector_create, возвращает
  точки в формате (N,1,2) float32, ожидаемом cv2.calcOpticalFlowPyrLK;
* LKTracker — обёртка над cv2.calcOpticalFlowPyrLK с фильтрацией по st==1;
* pixels_to_3d — депроекция 2D-точек в 3D через известную disparity и
  intrinsics (для stereo VO);
* select_features_with_valid_disparity — фильтр фич по валидной disparity
  с отсечением пограничных пикселей disparity-map.

Решено в плане Этапа 4: A1 на этом этапе не переписываем под эти хелперы,
чтобы не сломать уже работающий и провалидированный код. Используются
только в A2 (а позже — в A6).
"""
from __future__ import annotations

from typing import Optional, Tuple

import cv2
import numpy as np


# ---------------------------------------------------------------------------
# Детекторы фич
# ---------------------------------------------------------------------------
class FastDetector:
    """Обёртка над cv2.FastFeatureDetector_create с единым выходом для LK.

    detect(gray) -> np.ndarray shape (N, 1, 2), dtype float32 — формат,
    который ожидает cv2.calcOpticalFlowPyrLK без дополнительного reshape.
    """

    def __init__(self, threshold: int = 25, nonmax_suppression: bool = True) -> None:
        self.threshold = int(threshold)
        self.nonmax_suppression = bool(nonmax_suppression)
        self._detector = cv2.FastFeatureDetector_create(
            threshold=self.threshold,
            nonmaxSuppression=self.nonmax_suppression,
        )

    def detect(self, gray: np.ndarray) -> np.ndarray:
        kps = self._detector.detect(gray)
        if not kps:
            return np.zeros((0, 1, 2), dtype=np.float32)
        return np.array([k.pt for k in kps], dtype=np.float32).reshape(-1, 1, 2)


# ---------------------------------------------------------------------------
# LK tracker
# ---------------------------------------------------------------------------
class LKTracker:
    """Обёртка над cv2.calcOpticalFlowPyrLK с фильтрацией статуса.

    track(prev_gray, curr_gray, p0) -> (good_prev, good_curr, status_mask).
    Если поток не сошёлся или входной p0 пуст — все три значения пустые.

    Точки p0 и выходные good_prev/good_curr имеют форму (M, 1, 2) float32
    (та же, что отдаёт FastDetector, и та, что требуют все cv2 stereo/LK API).
    """

    def __init__(
        self,
        win_size: Tuple[int, int] = (21, 21),
        max_level: int = 3,
        criteria: Optional[Tuple[int, int, float]] = None,
    ) -> None:
        self.win_size = win_size
        self.max_level = int(max_level)
        self.criteria = criteria or (
            cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT,
            30,
            0.01,
        )

    def track(
        self, prev_gray: np.ndarray, curr_gray: np.ndarray, p0: np.ndarray
    ) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        if p0 is None or len(p0) == 0:
            empty = np.zeros((0, 1, 2), dtype=np.float32)
            return empty, empty, np.zeros((0,), dtype=np.uint8)

        p1, st, _err = cv2.calcOpticalFlowPyrLK(
            prev_gray,
            curr_gray,
            p0.astype(np.float32),
            None,
            winSize=self.win_size,
            maxLevel=self.max_level,
            criteria=self.criteria,
        )
        if p1 is None or st is None:
            empty = np.zeros((0, 1, 2), dtype=np.float32)
            return empty, empty, np.zeros((0,), dtype=np.uint8)

        status = st.flatten().astype(np.uint8)
        mask = status == 1
        good_prev = p0[mask].reshape(-1, 1, 2)
        good_curr = p1[mask].reshape(-1, 1, 2)
        return good_prev, good_curr, status


# ---------------------------------------------------------------------------
# Stereo helpers
# ---------------------------------------------------------------------------
def select_features_with_valid_disparity(
    pts_2d: np.ndarray,
    disp_map: np.ndarray,
    min_disp: float = 1.0,
    max_disp: Optional[float] = None,
    border: int = 1,
) -> Tuple[np.ndarray, np.ndarray]:
    """Оставить только точки, для которых disp_map[v,u] валиден.

    Args:
        pts_2d: (N, 1, 2) или (N, 2) float — координаты фич на левом кадре.
        disp_map: (H, W) float — disparity (например, после
            ``cv2.StereoSGBM.compute(...).astype(float32) / 16.0``).
            Некорректные пиксели обычно имеют отрицательные значения.
        min_disp: ниже этого порога точка считается слишком далёкой
            (numerically inf depth) и отбрасывается.
        max_disp: если задан, верхний порог (отбрасываем NaN, выбросы).
        border: ширина исключаемой рамки disp_map (там SGBM ненадёжен).

    Returns:
        kept_pts (M, 1, 2) float32 и disp_at_pts (M,) float32 — disparity
        в каждой сохранённой точке.
    """
    if pts_2d is None or len(pts_2d) == 0:
        empty_pts = np.zeros((0, 1, 2), dtype=np.float32)
        empty_disp = np.zeros((0,), dtype=np.float32)
        return empty_pts, empty_disp

    pts = np.asarray(pts_2d, dtype=np.float32).reshape(-1, 2)
    h, w = disp_map.shape[:2]

    u = pts[:, 0]
    v = pts[:, 1]
    in_image = (
        (u >= border)
        & (u < w - border)
        & (v >= border)
        & (v < h - border)
    )
    if not np.any(in_image):
        empty_pts = np.zeros((0, 1, 2), dtype=np.float32)
        empty_disp = np.zeros((0,), dtype=np.float32)
        return empty_pts, empty_disp

    ui = u[in_image].astype(np.int32)
    vi = v[in_image].astype(np.int32)
    pts_in = pts[in_image]
    disp_vals = disp_map[vi, ui].astype(np.float32)

    valid = (disp_vals >= float(min_disp)) & np.isfinite(disp_vals)
    if max_disp is not None:
        valid &= disp_vals <= float(max_disp)

    if not np.any(valid):
        empty_pts = np.zeros((0, 1, 2), dtype=np.float32)
        empty_disp = np.zeros((0,), dtype=np.float32)
        return empty_pts, empty_disp

    kept = pts_in[valid].reshape(-1, 1, 2)
    return kept, disp_vals[valid]


def pixels_to_3d(
    pts_2d: np.ndarray,
    disp_vals: np.ndarray,
    K: np.ndarray,
    baseline_m: float,
) -> np.ndarray:
    """Депроекция 2D-точек левого кадра в 3D в его собственной системе.

    Формула стандартная для rectified stereo:
        Z = fx * baseline / disp
        X = (u - cx) * Z / fx
        Y = (v - cy) * Z / fy

    Args:
        pts_2d: (M, 1, 2) или (M, 2) float — пиксельные координаты.
        disp_vals: (M,) float — disparity в каждой точке (>0).
        K: (3, 3) intrinsics матрица левой камеры.
        baseline_m: расстояние между cam0 и cam1 в метрах (>0).

    Returns:
        (M, 3) float32 — 3D-координаты в frame левой камеры (Z = forward).
    """
    pts = np.asarray(pts_2d, dtype=np.float32).reshape(-1, 2)
    disp = np.asarray(disp_vals, dtype=np.float32).reshape(-1)
    if len(pts) != len(disp):
        raise ValueError(
            f"pts_2d ({len(pts)}) and disp_vals ({len(disp)}) length mismatch"
        )
    if len(pts) == 0:
        return np.zeros((0, 3), dtype=np.float32)

    fx = float(K[0, 0])
    fy = float(K[1, 1])
    cx = float(K[0, 2])
    cy = float(K[1, 2])
    b = float(baseline_m)

    Z = fx * b / disp
    X = (pts[:, 0] - cx) * Z / fx
    Y = (pts[:, 1] - cy) * Z / fy
    return np.stack([X, Y, Z], axis=1).astype(np.float32)
