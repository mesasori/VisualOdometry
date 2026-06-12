"""6-DoF Extended Kalman Filter для visual-inertial fusion (loosely-coupled).

Состояние ImuState = (p, q, v, b_a, b_g), размерность 16.
Ковариация P 15x15: ориентация параметризуется 3 малыми углами в касательном
пространстве world frame (см. описание в src/fusion/imu_model.py).

Жизненный цикл (используется в A3/A4 на Этапах 6-7):

    ekf = EKF(sigma_a=..., sigma_g=..., ...)
    ekf.reset(initial_state)                      # опционально, по умолчанию identity
    for frame in dataset.frames():
        for imu in frame.imu:
            ekf.predict(imu, dt)                  # high-rate IMU integration
        pose_meas = vo_core.absolute_pose(...)    # абсолютная VO-поза
        ekf.update_pose(pose_meas, R_meas)        # коррекция от VO
        out_pose = ekf.get_pose(T_imu_cam=calib.T_imu_cam)

Контракт `update_pose(pose, R_meas)`:
* `pose` — Pose в world frame (та же семантика, что у Pose из interface.py).
* `R_meas` — 6x6 ковариация измерения; первые 3 диагональных — позиция (м²),
  последние 3 — ориентация (рад²).

Реализация — своя, без зависимости от filterpy. Quaternion math и
state-on-manifold update проще держать в одном модуле, чем обходить ограничения
filterpy.kalman.ExtendedKalmanFilter, который не поддерживает manifold нативно.

Synthetic self-test и predict-only smoke-test на KittiRaw — в `__main__` блоке
по образцу src/eval/runner.py._cli (CLI: --selftest, --smoke-raw).
"""
from __future__ import annotations

import argparse
import time
from typing import Optional

import numpy as np

from src.fusion.imu_model import (
    GRAVITY_WORLD,
    ImuState,
    _quat_from_so3,
    _quat_mul,
    _quat_normalize,
    _quat_to_R,
    _so3_log,
    propagate,
    state_transition_jacobian,
)
from src.vo.interface import IMUSample, Pose


class EKF:
    """Loosely-coupled visual-inertial EKF (state 16, cov 15x15).

    Args:
        sigma_a:  std акселерометра (м/с² / √Hz), continuous-time PSD.
        sigma_g:  std гироскопа (рад/с / √Hz).
        sigma_ba: std random walk bias акселерометра (м/с³ / √Hz).
        sigma_bg: std random walk bias гироскопа (рад/с² / √Hz).
        initial_P_diag: диагональная инициализация P. Либо одно число
            (все 15 диагональных одинаковы), либо вектор длины 15
            (порядок: δp, δθ, δv, δb_a, δb_g).
        gravity: гравитация в world frame, по умолчанию (0, 0, -9.81).

    Дефолтные σ — порядок OXTS-RT3003 (KITTI). Конкретные A3/A4 могут передать
    свои значения через __init__.
    """

    def __init__(
        self,
        sigma_a: float = 0.05,
        sigma_g: float = 0.005,
        sigma_ba: float = 1e-4,
        sigma_bg: float = 1e-5,
        initial_P_diag: float | np.ndarray = 1e-6,
        gravity: Optional[np.ndarray] = None,
    ) -> None:
        self.sigma_a = float(sigma_a)
        self.sigma_g = float(sigma_g)
        self.sigma_ba = float(sigma_ba)
        self.sigma_bg = float(sigma_bg)
        self.gravity = (
            np.asarray(gravity, dtype=np.float64).reshape(3)
            if gravity is not None
            else GRAVITY_WORLD.copy()
        )

        # Continuous-time process noise PSD (12x12): [n_a, n_g, n_ba, n_bg].
        self.Q_c = np.diag(
            np.concatenate(
                [
                    np.full(3, self.sigma_a ** 2),
                    np.full(3, self.sigma_g ** 2),
                    np.full(3, self.sigma_ba ** 2),
                    np.full(3, self.sigma_bg ** 2),
                ]
            )
        )

        self.state = ImuState.identity()
        self._initial_P_diag = initial_P_diag
        self.P = self._init_P(initial_P_diag)

    @staticmethod
    def _init_P(diag: float | np.ndarray) -> np.ndarray:
        if np.isscalar(diag):
            return np.eye(15) * float(diag)
        d = np.asarray(diag, dtype=np.float64).reshape(15)
        return np.diag(d)

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------
    def reset(
        self,
        initial_state: Optional[ImuState] = None,
        initial_P_diag: float | np.ndarray | None = None,
    ) -> None:
        """Сбросить state и (опционально) P.

        Если `initial_state` не задан — state = identity.
        Если `initial_P_diag` не задан — берётся из __init__.
        """
        self.state = initial_state.copy() if initial_state is not None else ImuState.identity()
        if initial_P_diag is None:
            initial_P_diag = self._initial_P_diag
        self.P = self._init_P(initial_P_diag)

    # ------------------------------------------------------------------
    # Predict
    # ------------------------------------------------------------------
    def predict(self, imu: IMUSample, dt: float) -> None:
        """Прогноз: пропагация nominal state + ковариации через одну IMU-точку.

        Discrete-time covariance update (first-order):
            P_{k+1} = F · P · Fᵀ + G_c · Q_c · G_cᵀ · dt
        где F содержит dt в нужных блоках, G_c — continuous-time noise input.
        """
        if dt <= 0.0:
            return

        new_state = propagate(self.state, imu, dt, self.gravity)
        F, G_c = state_transition_jacobian(self.state, imu, dt)
        Q_d = G_c @ self.Q_c @ G_c.T * dt
        self.P = F @ self.P @ F.T + Q_d
        # Симметризация против накопления численной асимметрии.
        self.P = 0.5 * (self.P + self.P.T)
        self.state = new_state

    # ------------------------------------------------------------------
    # Update from VO pose measurement
    # ------------------------------------------------------------------
    def update_pose(self, pose: Pose, R_meas: np.ndarray) -> None:
        """Loosely-coupled update от VO-измерения 6-DoF позы.

        Args:
            pose: абсолютная поза в world frame.
            R_meas: 6x6 ковариация измерения (первые 3 — позиция, последние 3 —
                ориентация в касательном пространстве world frame).
        """
        R_meas = np.asarray(R_meas, dtype=np.float64).reshape(6, 6)

        # H: 6x15, наблюдаемы только p (rows 0..3) и θ (rows 3..6).
        H = np.zeros((6, 15), dtype=np.float64)
        H[0:3, 0:3] = np.eye(3)
        H[3:6, 3:6] = np.eye(3)

        # Innovation: y = [p_meas - p_state, log(R_meas · R_stateᵀ)].
        # log(R_meas · R_stateᵀ) — это малый поворот δθ в world frame такой, что
        # R_meas = exp(δθ) · R_state, что соответствует нашей параметризации δθ.
        p_state = self.state.p
        R_state = self.state.R_wb()
        R_pose = pose.R
        y_p = pose.t.reshape(3) - p_state
        y_theta = _so3_log(R_pose @ R_state.T)
        y = np.concatenate([y_p, y_theta])

        S = H @ self.P @ H.T + R_meas
        K = self.P @ H.T @ np.linalg.inv(S)
        dx = K @ y  # 15

        # State ⊞-update.
        new_p = self.state.p + dx[0:3]
        # Для δθ в world frame: q_new = exp_q(δθ) ⊗ q_old.
        new_q = _quat_normalize(_quat_mul(_quat_from_so3(dx[3:6]), self.state.q))
        new_v = self.state.v + dx[6:9]
        new_ba = self.state.ba + dx[9:12]
        new_bg = self.state.bg + dx[12:15]

        self.state = ImuState(
            p=new_p, q=new_q, v=new_v, ba=new_ba, bg=new_bg
        )

        # Joseph-form covariance update для численной устойчивости.
        I = np.eye(15)
        IKH = I - K @ H
        self.P = IKH @ self.P @ IKH.T + K @ R_meas @ K.T
        self.P = 0.5 * (self.P + self.P.T)

    # ------------------------------------------------------------------
    # Outputs
    # ------------------------------------------------------------------
    def get_pose(self, T_imu_cam: Optional[np.ndarray] = None) -> Pose:
        """Возвращает текущую позу в world frame.

        Args:
            T_imu_cam: если задано (4x4), результирующая поза — позиция камеры,
                т.е. T_w_cam = T_w_imu @ T_imu_cam. Иначе — поза IMU body frame.
        """
        T_w_imu = np.eye(4, dtype=np.float64)
        T_w_imu[:3, :3] = self.state.R_wb()
        T_w_imu[:3, 3] = self.state.p
        if T_imu_cam is None:
            return Pose.from_matrix(T_w_imu)
        T_imu_cam = np.asarray(T_imu_cam, dtype=np.float64).reshape(4, 4)
        return Pose.from_matrix(T_w_imu @ T_imu_cam)


# ---------------------------------------------------------------------------
# Synthetic self-test: круговое движение в плоскости XY
# ---------------------------------------------------------------------------
def _synthetic_test(
    verbose: bool = True,
    seed: int = 42,
    duration: float = 30.0,
    imu_hz: float = 100.0,
    vo_hz: float = 10.0,
    radius: float = 10.0,
    omega: float = 0.5,
) -> dict:
    """Круговое движение, idealized IMU + noisy VO. Должно сходиться к истине.

    Гарантирует ATE_pos < 0.3 м, ATE_ori < 0.05 рад на стандартных параметрах
    (30 с, IMU 100 Hz, VO 10 Hz, радиус 10 м, ω = 0.5 рад/с).
    """
    rng = np.random.default_rng(seed)

    imu_dt = 1.0 / imu_hz
    vo_every_n = int(round(imu_hz / vo_hz))
    n_steps = int(round(duration * imu_hz))

    # IMU и VO «истинные» параметры (то, что генератор внутри использует для шумов).
    sigma_a_meas = 0.02
    sigma_g_meas = 0.002
    sigma_p_vo = 0.05
    sigma_theta_vo = 0.01

    ba_true = np.array([0.03, -0.02, 0.04], dtype=np.float64)
    bg_true = np.array([0.005, -0.003, 0.002], dtype=np.float64)
    g_world = GRAVITY_WORLD

    # EKF: сигмы чуть «щедрее» истинных, чтобы фильтр не был переоптимистичен.
    ekf = EKF(
        sigma_a=0.05,
        sigma_g=0.005,
        sigma_ba=1e-3,
        sigma_bg=1e-4,
        initial_P_diag=1e-4,
    )

    # Начальные условия: точка (R, 0, 0), heading = +π/2 вокруг z (смотрим в +Y),
    # скорость = (0, R·ω, 0). Гонорас counter-clockwise при ω > 0.
    init_state = ImuState(
        p=np.array([radius, 0.0, 0.0]),
        q=np.array([np.cos(np.pi / 4), 0.0, 0.0, np.sin(np.pi / 4)]),
        v=np.array([0.0, radius * omega, 0.0]),
        ba=np.zeros(3),
        bg=np.zeros(3),
    )
    ekf.reset(init_state, initial_P_diag=1e-4)

    R_meas_default = np.diag(
        [
            sigma_p_vo ** 2,
            sigma_p_vo ** 2,
            sigma_p_vo ** 2,
            sigma_theta_vo ** 2,
            sigma_theta_vo ** 2,
            sigma_theta_vo ** 2,
        ]
    )

    pos_errs = np.zeros(n_steps)
    ori_errs = np.zeros(n_steps)

    for k in range(n_steps):
        t = k * imu_dt
        theta = omega * t

        # Ground truth.
        p_true = np.array(
            [radius * np.cos(theta), radius * np.sin(theta), 0.0],
            dtype=np.float64,
        )
        # Скорость касательна к окружности.
        v_true = np.array(
            [-radius * omega * np.sin(theta), radius * omega * np.cos(theta), 0.0],
            dtype=np.float64,
        )
        # Heading — direction of velocity; heading angle = theta + π/2.
        heading = theta + np.pi / 2
        q_true = np.array(
            [np.cos(heading / 2), 0.0, 0.0, np.sin(heading / 2)],
            dtype=np.float64,
        )
        # Центростремительное ускорение в world.
        a_true_w = -radius * omega * omega * np.array(
            [np.cos(theta), np.sin(theta), 0.0],
            dtype=np.float64,
        )

        R_true_wb = _quat_to_R(q_true)
        # Specific force: f = R_wbᵀ · (a_true - g).
        f_body_true = R_true_wb.T @ (a_true_w - g_world)
        imu_acc = f_body_true + ba_true + rng.normal(scale=sigma_a_meas, size=3)
        # Угловая скорость в body: ω_b = R_wbᵀ · ω_w. Здесь ω_w = (0, 0, omega),
        # и вращение происходит вокруг z (совпадает с body z) → ω_b = (0, 0, ω).
        omega_b_true = R_true_wb.T @ np.array([0.0, 0.0, omega])
        imu_gyro = omega_b_true + bg_true + rng.normal(scale=sigma_g_meas, size=3)

        imu = IMUSample(timestamp=t, acc=imu_acc, gyro=imu_gyro)
        ekf.predict(imu, imu_dt)

        # VO-измерение раз в vo_every_n шагов (пропускаем k=0, потому что предсказание
        # ещё не запускалось перед измерением).
        if k > 0 and k % vo_every_n == 0:
            p_meas = p_true + rng.normal(scale=sigma_p_vo, size=3)
            theta_noise = rng.normal(scale=sigma_theta_vo, size=3)
            q_meas = _quat_mul(q_true, _quat_from_so3(theta_noise))
            pose_meas = Pose(R=_quat_to_R(q_meas), t=p_meas.reshape(3, 1))
            ekf.update_pose(pose_meas, R_meas_default)

        # Накопление ошибок (после prediction и опционально update).
        pos_errs[k] = float(np.linalg.norm(ekf.state.p - p_true))
        R_est = ekf.state.R_wb()
        ori_errs[k] = float(np.linalg.norm(_so3_log(R_est.T @ R_true_wb)))

    pos_rmse = float(np.sqrt(np.mean(pos_errs ** 2)))
    pos_max = float(np.max(pos_errs))
    pos_final = float(pos_errs[-1])
    ori_rmse = float(np.sqrt(np.mean(ori_errs ** 2)))
    ori_max = float(np.max(ori_errs))
    ori_final = float(ori_errs[-1])

    ba_est = ekf.state.ba.copy()
    bg_est = ekf.state.bg.copy()

    result = {
        "n_steps": n_steps,
        "duration_s": duration,
        "imu_hz": imu_hz,
        "vo_hz": vo_hz,
        "pos_rmse_m": pos_rmse,
        "pos_max_m": pos_max,
        "pos_final_m": pos_final,
        "ori_rmse_rad": ori_rmse,
        "ori_max_rad": ori_max,
        "ori_final_rad": ori_final,
        "ba_estimated": ba_est.tolist(),
        "ba_true": ba_true.tolist(),
        "bg_estimated": bg_est.tolist(),
        "bg_true": bg_true.tolist(),
    }

    if verbose:
        print("=" * 70)
        print(f"EKF synthetic test (circular motion)")
        print(
            f"radius={radius:.1f} m, omega={omega:.2f} rad/s, "
            f"duration={duration:.1f} s"
        )
        print(f"IMU at {imu_hz:.0f} Hz, VO updates at {vo_hz:.0f} Hz")
        print(f"n_steps={n_steps}, vo_every={vo_every_n}")
        print(
            f"position  RMSE/max/final: {pos_rmse:.4f} / {pos_max:.4f} / "
            f"{pos_final:.4f} m"
        )
        print(
            f"ori       RMSE/max/final: {ori_rmse:.5f} / {ori_max:.5f} / "
            f"{ori_final:.5f} rad"
        )
        print(f"bias_a estimated: {ba_est}")
        print(f"bias_a true:      {ba_true}")
        print(f"bias_g estimated: {bg_est}")
        print(f"bias_g true:      {bg_true}")

    assert pos_rmse < 0.3, f"position RMSE {pos_rmse:.4f} m > 0.3 m"
    assert ori_rmse < 0.05, f"orientation RMSE {ori_rmse:.4f} rad > 0.05 rad"

    if verbose:
        print("OK — synthetic test passed (pos_rmse < 0.3 m, ori_rmse < 0.05 rad)")
    return result


# ---------------------------------------------------------------------------
# Predict-only smoke-test on KittiRaw (для иллюстрации, что IMU без VO дрейфует)
# ---------------------------------------------------------------------------
def _smoke_raw(
    date: str = "2011_09_26",
    drive: str = "0001",
    basedir: str = "kitti_data/raw",
    verbose: bool = True,
) -> dict:
    """Predict-only прогон IMU из реального KITTI Raw драйва.

    Запускает EKF.predict() на каждом OXTS-семпле, без VO-коррекции, и сравнивает
    итоговую позицию с GT. Цель — продемонстрировать характер дрейфа IMU без
    update (для аргументации в сводке Этапа 6).

    Состояние инициализируется так, чтобы world frame совпал с первым GT:
    p = gt[0].position, q = R-из gt[0].R, v ≈ (gt[1].position - gt[0].position) / dt.
    """
    from src.datasets.kitti_raw import KittiRaw

    ds = KittiRaw(basedir, date, drive)
    gt = ds.ground_truth()
    if gt is None or len(gt) < 2:
        raise RuntimeError(f"GT недоступен для {date}/{drive}, smoke-test невозможен")

    frames = list(ds.frames())
    if len(frames) < 2:
        raise RuntimeError(f"{date}/{drive}: меньше двух кадров")

    # Initial state из gt[0] и (gt[1] - gt[0]) / dt.
    from src.fusion.imu_model import _R_to_quat

    p0 = np.array(gt[0].position(), dtype=np.float64)
    R0 = gt[0].R.copy()
    q0 = _R_to_quat(R0)
    dt01 = frames[1].timestamp - frames[0].timestamp
    if dt01 <= 0:
        dt01 = 0.1
    v0 = (np.array(gt[1].position()) - p0) / dt01

    ekf = EKF()
    ekf.reset(ImuState(p=p0, q=q0, v=v0, ba=np.zeros(3), bg=np.zeros(3)))

    estimates = [ekf.state.p.copy()]
    gt_positions = [p0.copy()]
    timestamps = [frames[0].timestamp]
    prev_t = frames[0].timestamp

    for i in range(1, len(frames)):
        frame = frames[i]
        dt = frame.timestamp - prev_t
        prev_t = frame.timestamp
        if frame.imu and dt > 0:
            ekf.predict(frame.imu[0], dt)
        estimates.append(ekf.state.p.copy())
        gt_positions.append(np.array(gt[i].position()))
        timestamps.append(frame.timestamp)

    estimates_np = np.array(estimates)
    gt_np = np.array(gt_positions)
    ts_np = np.array(timestamps)

    diff = estimates_np - gt_np
    drift = np.linalg.norm(diff, axis=1)
    duration = float(ts_np[-1] - ts_np[0])

    result = {
        "date": date,
        "drive": drive,
        "n_frames": len(frames),
        "duration_s": duration,
        "drift_final_m": float(drift[-1]),
        "drift_max_m": float(np.max(drift)),
        "drift_t5s_m": float(drift[np.searchsorted(ts_np, ts_np[0] + 5.0)])
        if duration > 5.0
        else None,
    }

    if verbose:
        print("=" * 70)
        print(f"EKF predict-only smoke-test on {date}/drive_{drive}")
        print(f"frames: {len(frames)}, duration: {duration:.2f} s")
        print(f"drift (no VO update):")
        print(f"  final: {result['drift_final_m']:.2f} m")
        print(f"  max:   {result['drift_max_m']:.2f} m")
        if result["drift_t5s_m"] is not None:
            print(f"  at 5s: {result['drift_t5s_m']:.2f} m")
        print(
            "Этот дрейф — нормальное поведение IMU без коррекции от VO. "
            "На Этапах 6-7 EKF.update_pose от A1/A2 будет ловить эту ошибку."
        )
    return result


# ---------------------------------------------------------------------------
# CLI entry point
# ---------------------------------------------------------------------------
def _cli() -> int:
    parser = argparse.ArgumentParser(
        description="EKF self-test и predict-only smoke-test на KittiRaw"
    )
    parser.add_argument(
        "--selftest",
        action="store_true",
        help="запустить synthetic-тест (круговое движение)",
    )
    parser.add_argument(
        "--smoke-raw",
        action="store_true",
        help="запустить predict-only прогон IMU на KittiRaw drive",
    )
    parser.add_argument("--date", default="2011_09_26", help="для --smoke-raw")
    parser.add_argument("--drive", default="0001", help="для --smoke-raw")
    parser.add_argument(
        "--basedir",
        default="kitti_data/raw",
        help="для --smoke-raw, по умолчанию kitti_data/raw",
    )
    args = parser.parse_args()

    if not args.selftest and not args.smoke_raw:
        parser.print_help()
        return 1

    if args.selftest:
        t0 = time.perf_counter()
        _synthetic_test(verbose=True)
        print(f"selftest elapsed: {time.perf_counter() - t0:.2f} s")

    if args.smoke_raw:
        _smoke_raw(date=args.date, drive=args.drive, basedir=args.basedir)

    return 0


if __name__ == "__main__":
    raise SystemExit(_cli())
