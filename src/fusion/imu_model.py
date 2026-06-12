"""Дискретная IMU-модель для prediction step EKF (учёт bias и гравитации).

Конвенции:
* World frame: первая поза IMU (T_w_imu_0 = identity). Ось z направлена вверх
  (ENU-подобно): g_world = (0, 0, -9.81). Тогда при неподвижном IMU акселерометр
  выдаёт specific force (0, 0, +9.81) в body frame, что соответствует KITTI
  OXTS (au ≈ +10.21 в первой строке drive_0001).
* Body frame: KITTI OXTS, x=forward, y=left, z=up.
* IMU model:
    a_meas = R(q)^T (a_true - g_world) + b_a + n_a    (specific force)
    ω_meas = ω_true + b_g + n_g
* Кватернион формата (w, x, y, z), Hamilton convention, body→world:
    v_world = R(q) · v_body.

Состояние ImuState = (p, q, v, b_a, b_g), размерность 16.

Error state δx = (δp, δθ, δv, δb_a, δb_g), размерность 15. Ошибка ориентации
δθ ∈ R³ задаёт малый поворот в касательном пространстве world frame:
    q_true = exp_q(δθ_w) ⊗ q_state,   R_true = exp(δθ_w) · R_state.

Ссылки:
* J. Sola, "Quaternion kinematics for the error-state Kalman filter",
  arXiv:1711.02508 — формулы exp/log, дискретизация error-state.
* S. Weiss et al., "Multi-sensor fusion for ego-motion of MAV" (ETH MSF, 2013)
  — классическая loosely-coupled VIO EKF с тем же state layout.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Optional, Tuple

import numpy as np

from src.vo.interface import IMUSample


GRAVITY_WORLD: np.ndarray = np.array([0.0, 0.0, -9.81], dtype=np.float64)


# ---------------------------------------------------------------------------
# Quaternion / SO(3) helpers
# ---------------------------------------------------------------------------
def _skew(v: np.ndarray) -> np.ndarray:
    """Skew-symmetric matrix [v]× для cross-product: [v]× @ u = v × u."""
    v = np.asarray(v, dtype=np.float64).reshape(3)
    return np.array(
        [
            [0.0, -v[2], v[1]],
            [v[2], 0.0, -v[0]],
            [-v[1], v[0], 0.0],
        ],
        dtype=np.float64,
    )


def _quat_normalize(q: np.ndarray) -> np.ndarray:
    q = np.asarray(q, dtype=np.float64).reshape(4)
    n = float(np.linalg.norm(q))
    if n < 1e-12:
        raise ValueError("Quaternion has zero norm")
    return q / n


def _quat_mul(q1: np.ndarray, q2: np.ndarray) -> np.ndarray:
    """Hamilton product q1 ⊗ q2 для (w, x, y, z)-кватернионов."""
    q1 = np.asarray(q1, dtype=np.float64).reshape(4)
    q2 = np.asarray(q2, dtype=np.float64).reshape(4)
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    return np.array(
        [
            w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
            w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        ],
        dtype=np.float64,
    )


def _quat_to_R(q: np.ndarray) -> np.ndarray:
    """Rotation matrix 3x3 (body→world): v_world = R · v_body."""
    q = _quat_normalize(q)
    w, x, y, z = q
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ],
        dtype=np.float64,
    )


def _R_to_quat(R: np.ndarray) -> np.ndarray:
    """Rotation matrix → quaternion (w, x, y, z). Численно устойчиво через trace."""
    R = np.asarray(R, dtype=np.float64).reshape(3, 3)
    tr = R[0, 0] + R[1, 1] + R[2, 2]
    if tr > 0.0:
        s = 0.5 / np.sqrt(tr + 1.0)
        w = 0.25 / s
        x = (R[2, 1] - R[1, 2]) * s
        y = (R[0, 2] - R[2, 0]) * s
        z = (R[1, 0] - R[0, 1]) * s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = 2.0 * np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
        w = (R[2, 1] - R[1, 2]) / s
        x = 0.25 * s
        y = (R[0, 1] + R[1, 0]) / s
        z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = 2.0 * np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
        w = (R[0, 2] - R[2, 0]) / s
        x = (R[0, 1] + R[1, 0]) / s
        y = 0.25 * s
        z = (R[1, 2] + R[2, 1]) / s
    else:
        s = 2.0 * np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
        w = (R[1, 0] - R[0, 1]) / s
        x = (R[0, 2] + R[2, 0]) / s
        y = (R[1, 2] + R[2, 1]) / s
        z = 0.25 * s
    return _quat_normalize(np.array([w, x, y, z], dtype=np.float64))


def _so3_exp(omega: np.ndarray) -> np.ndarray:
    """Rodrigues' formula: 3-вектор оси×угла → rotation matrix 3x3."""
    omega = np.asarray(omega, dtype=np.float64).reshape(3)
    theta = float(np.linalg.norm(omega))
    if theta < 1e-8:
        K = _skew(omega)
        return np.eye(3) + K + 0.5 * (K @ K)
    K = _skew(omega / theta)
    return np.eye(3) + np.sin(theta) * K + (1.0 - np.cos(theta)) * (K @ K)


def _so3_log(R: np.ndarray) -> np.ndarray:
    """Inverse Rodrigues: rotation matrix → 3-вектор оси×угла."""
    R = np.asarray(R, dtype=np.float64).reshape(3, 3)
    cos_theta = (np.trace(R) - 1.0) * 0.5
    cos_theta = float(np.clip(cos_theta, -1.0, 1.0))
    theta = float(np.arccos(cos_theta))
    if theta < 1e-8:
        return 0.5 * np.array(
            [R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]],
            dtype=np.float64,
        )
    sin_theta = float(np.sin(theta))
    if abs(sin_theta) < 1e-8:
        # Численно сложный случай theta ≈ π — для нашего сценария не возникает,
        # делаем устойчивый fallback через diag(R)+I.
        diag = np.diag(R) + 1.0
        i = int(np.argmax(diag))
        axis = np.zeros(3)
        axis[i] = np.sqrt(diag[i] * 0.5)
        for j in range(3):
            if j != i:
                axis[j] = R[i, j] / (2.0 * axis[i])
        return theta * axis
    factor = theta / (2.0 * sin_theta)
    return factor * np.array(
        [R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]],
        dtype=np.float64,
    )


def _quat_from_so3(omega: np.ndarray) -> np.ndarray:
    """Экспоненциальное отображение касательного so(3) → SO(3) в кватернионах.

    quat_from_so3(omega) = (cos(|omega|/2), sin(|omega|/2) · omega/|omega|).
    """
    omega = np.asarray(omega, dtype=np.float64).reshape(3)
    theta = float(np.linalg.norm(omega))
    if theta < 1e-8:
        half = 0.5 * omega
        return np.array(
            [1.0 - 0.125 * theta * theta, half[0], half[1], half[2]],
            dtype=np.float64,
        )
    half = 0.5 * theta
    s = float(np.sin(half) / theta)
    return np.array(
        [float(np.cos(half)), s * omega[0], s * omega[1], s * omega[2]],
        dtype=np.float64,
    )


# ---------------------------------------------------------------------------
# ImuState
# ---------------------------------------------------------------------------
@dataclass
class ImuState:
    """Полное состояние EKF: позиция / ориентация / скорость / bias акселя / bias гиро.

    Все векторы — в world frame, кроме bias (в body frame).
    Кватернион — (w, x, y, z), Hamilton, body→world.
    """

    p: np.ndarray
    q: np.ndarray
    v: np.ndarray
    ba: np.ndarray
    bg: np.ndarray

    def __post_init__(self) -> None:
        self.p = np.asarray(self.p, dtype=np.float64).reshape(3)
        self.q = _quat_normalize(np.asarray(self.q, dtype=np.float64).reshape(4))
        self.v = np.asarray(self.v, dtype=np.float64).reshape(3)
        self.ba = np.asarray(self.ba, dtype=np.float64).reshape(3)
        self.bg = np.asarray(self.bg, dtype=np.float64).reshape(3)

    @classmethod
    def identity(cls) -> "ImuState":
        return cls(
            p=np.zeros(3),
            q=np.array([1.0, 0.0, 0.0, 0.0]),
            v=np.zeros(3),
            ba=np.zeros(3),
            bg=np.zeros(3),
        )

    def to_vector(self) -> np.ndarray:
        return np.concatenate([self.p, self.q, self.v, self.ba, self.bg])

    @classmethod
    def from_vector(cls, x: np.ndarray) -> "ImuState":
        x = np.asarray(x, dtype=np.float64).reshape(16)
        return cls(p=x[0:3], q=x[3:7], v=x[7:10], ba=x[10:13], bg=x[13:16])

    def R_wb(self) -> np.ndarray:
        """Rotation matrix body→world."""
        return _quat_to_R(self.q)

    def copy(self) -> "ImuState":
        return ImuState(
            p=self.p.copy(),
            q=self.q.copy(),
            v=self.v.copy(),
            ba=self.ba.copy(),
            bg=self.bg.copy(),
        )


# ---------------------------------------------------------------------------
# Propagation (nominal state through one IMU sample)
# ---------------------------------------------------------------------------
def propagate(
    state: ImuState,
    imu: IMUSample,
    dt: float,
    gravity: Optional[np.ndarray] = None,
) -> ImuState:
    """Одна итерация дискретной IMU-модели (Euler).

    Уравнения::

        ω      = imu.gyro - state.bg
        a_body = imu.acc  - state.ba
        a_w    = R(q) · a_body + g_world
        p_{k+1} = p_k + v_k · dt + 0.5 · a_w · dt²
        v_{k+1} = v_k + a_w · dt
        q_{k+1} = q_k ⊗ exp_q(ω · dt)
        b_a, b_g — без изменения (random walk учитывается через шум в EKF.predict).

    Замечание про специфическую силу: KITTI OXTS af/al/au — это specific force в
    body frame, т.е. (a_true - g) повёрнутое в body. Поэтому при компенсации
    гравитации в world frame: a_w = R · imu.acc + g_world (после вычитания bias),
    что и реализовано выше.
    """
    g = gravity if gravity is not None else GRAVITY_WORLD
    g = np.asarray(g, dtype=np.float64).reshape(3)

    omega = imu.gyro - state.bg
    acc_body = imu.acc - state.ba

    R = state.R_wb()
    a_w = R @ acc_body + g

    new_p = state.p + state.v * dt + 0.5 * a_w * dt * dt
    new_v = state.v + a_w * dt
    new_q = _quat_normalize(_quat_mul(state.q, _quat_from_so3(omega * dt)))

    return ImuState(
        p=new_p,
        q=new_q,
        v=new_v,
        ba=state.ba.copy(),
        bg=state.bg.copy(),
    )


# ---------------------------------------------------------------------------
# Error-state Jacobians (F: 15x15, G_c: 15x12)
# ---------------------------------------------------------------------------
def state_transition_jacobian(
    state: ImuState, imu: IMUSample, dt: float
) -> Tuple[np.ndarray, np.ndarray]:
    """Линеаризация error-state dynamics для propagation covariance.

    Возвращает (F, G_c):

    * F (15x15): ∂(δx_{k+1})/∂(δx_k), discrete-time, включает dt.
    * G_c (15x12): continuous-time noise input matrix (без dt). EKF строит
      дискретную ковариацию Q_d = G_c · Q_c · G_cᵀ · dt (первый порядок).

    Порядок ошибок::

        δx = [δp (3), δθ (3), δv (3), δb_a (3), δb_g (3)]

    Все δ — в world frame, кроме δb_a, δb_g (в body).

    Порядок шумов::

        w  = [n_a (3), n_g (3), n_ba (3), n_bg (3)]

    Формулы (Sola §6, eq. 270 и MSF Weiss eq. 4.62)::

        δp_{k+1}  = δp_k + δv_k · dt
        δθ_{k+1}  = δθ_k - R · δb_g · dt - R · n_g · dt
        δv_{k+1}  = δv_k - [R · (a - b_a)]× · δθ_k · dt - R · δb_a · dt - R · n_a · dt
        δb_a_{k+1} = δb_a_k + n_ba · dt
        δb_g_{k+1} = δb_g_k + n_bg · dt
    """
    R = state.R_wb()
    acc_body = imu.acc - state.ba

    F = np.eye(15, dtype=np.float64)
    # δp_{k+1} = δp_k + δv_k · dt
    F[0:3, 6:9] = np.eye(3) * dt
    # δθ_{k+1} = δθ_k - R · δb_g · dt
    F[3:6, 12:15] = -R * dt
    # δv_{k+1} = δv_k - [R · acc_body]× · δθ_k · dt - R · δb_a · dt
    F[6:9, 3:6] = -_skew(R @ acc_body) * dt
    F[6:9, 9:12] = -R * dt

    G = np.zeros((15, 12), dtype=np.float64)
    # gyro noise n_g → δθ через -R
    G[3:6, 3:6] = -R
    # accel noise n_a → δv через -R
    G[6:9, 0:3] = -R
    # bias random walks
    G[9:12, 6:9] = np.eye(3)
    G[12:15, 9:12] = np.eye(3)

    return F, G
