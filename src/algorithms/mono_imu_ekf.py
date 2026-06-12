"""A3: Mono + IMU через loosely-coupled EKF (Visual-Inertial Odometry).
Архитектура (loosely-coupled VIO)::

    Frame_i:                       Frame_{i+1}:
    ┌────────────────┐             ┌────────────────┐
    │ left image     │──┐          │ left image     │──┐
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
    │ MonoFastLkCore   ││           │ MonoFastLkCore   ││
    │ R_rel, t̂_rel    ││           │ R_rel, t̂_rel    ││
    │ (unit-norm)      ││           │ (unit-norm)      ││
    └──────────────────┘│           └──────────────────┘│
            │            │                  │           │
            ▼            │                  ▼           │
    ┌──────────────────────────────────────────────────────┐
    │ scale_i = ‖p̂_cam_predict_i − p_cam_meas_{i−1}‖    │
    │ T_rel  = (R_rel, scale_i · t̂_rel)                  │
    │ T_w_cam_meas = T_w_cam_meas_{i−1} ∘ T_rel           │
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

Архитектурные решения:
1. **GT-инициализация первого кадра**: при reset нам известна только
   калибровка, поэтому frame[0] входит в process(): берётся `gt_pose`
   (cam0 first frame = identity по построению KittiRaw GT) для T_w_cam_0,
   а T_w_imu_0 = T_w_cam_0 ∘ T_cam_imu. Скорость v0 оценивается грубо как
   (gt[1].t − gt[0].t) / dt — первый IMU-burst даёт сам runner на frame 0
   через `frame.imu`, но скорость нам нужна сразу. Принципиально:
   обращаемся к `frame.gt_pose` только один раз (frame 0) — дальше A3
   работает без подсматривания.
   Если GT отсутствует — фолбэк: T_w_imu = identity, v0 = 0; в этом
   режиме первые ~30 кадров EKF будет восстанавливать масштаб только из
   IMU, что обычно даёт большой transient drift.

2. **Scale per-step из IMU** (см. большой план §6.5 и stages/07):
   IMU предсказание текущей позы IMU → проецирование в cam frame →
   разность с прошлой VO-измеренной позой cam. Норма этой разности и
   есть «реальный пробег камеры между кадрами», на который умножается
   unit-norm `t̂_rel` из MonoFastLkCore. Если предсказанный сдвиг
   меньше `min_scale_m`, VO-измерение пропускается (innovation slamming
   против шумной EKF velocity).

3. **MonoFastLkCore без A1-gate**: heuristic gate из A1 (forward-axis
   dominance, scale > 0.1) был костылём для скрытия моментов остановки
   и backward-движения; в VIO эту функцию выполняет сам EKF (плюс
   мы дополнительно skip-im scale меньше порога). Поэтому используем
   только `MonoFastLkCore.step()`, без обёртки `MonoFastLk.process()`.

4. **Constant R_meas**: для loosely-coupled VIO на KITTI достаточно
   фиксированной 6x6 диагонали (`sigma_p_meas=0.2 м`, `sigma_theta_meas=
   0.02 рад` по умолчанию). На Этапе 9 можно будет адаптивно задавать
   через success rate VO или residual chi².

5. **IMU sync rate ~10 Hz**: KITTI `*_sync/` даёт 1 IMU-семпл на кадр.
   Loosely-coupled EKF спокойно работает на этой частоте, потому что
   propagate один большой шаг по 100 ms всё ещё стабилен (Euler).
   Для tightly-coupled VIO нужны unsync-данные (100 Hz) — это уже за
   рамками Этапа 6.

Frame conventions (повторение [stages/03_data_layout.md](../../stages/03_data_layout.md)
для самодостаточности этого модуля)::

    world frame  = cam0 первого кадра (z вверх для GT после Tr-преобразования).
    T_imu_cam    = `Calibration.T_imu_cam`, переводит точку cam0 → IMU body.
    T_w_imu      = T_w_cam ∘ T_cam_imu = T_w_cam ∘ T_imu_cam⁻¹.
    T_w_cam      = T_w_imu ∘ T_imu_cam   (это то, что EKF.get_pose отдаёт).
"""
from __future__ import annotations

from typing import Optional

import numpy as np

from src.algorithms import register_algorithm
from src.algorithms.mono_fast_lk import MonoFastLkCore
from src.fusion.ekf import EKF
from src.fusion.imu_model import ImuState, _R_to_quat
from src.vo.interface import (
    AlgorithmRequirements,
    Calibration,
    FrameData,
    Pose,
    VOAlgorithm,
)


@register_algorithm("A3")
class MonoImuEKF(VOAlgorithm):
    """A3: Mono FAST+LK+Essential + IMU через loosely-coupled EKF.

    Args:
        sigma_a: continuous-time PSD акселерометра (м/с² / √Hz).
        sigma_g: continuous-time PSD гироскопа (рад/с / √Hz).
        sigma_ba: random walk bias акселерометра.
        sigma_bg: random walk bias гироскопа.
        sigma_p_meas: σ позиционного измерения от VO (м); входит в R_meas.
        sigma_theta_meas: σ ориентационного измерения от VO (рад); входит в R_meas.
        min_scale_m: минимальный модуль IMU-предсказанного сдвига камеры; если
            ‖p̂_cam − p_cam_prev‖ < min_scale_m, VO-измерение пропускается
            (нет надёжного scale на этом шаге).
        redetect_threshold: re-detect FAST когда tracked features < threshold.
        fast_threshold: FAST detector threshold.

    Дефолтные σ — порядок OXTS-RT3003 (KITTI), как у EKF defaults в
    [src/fusion/ekf.py](../fusion/ekf.py). σ_p_meas=0.2 м оставляет EKF
    достаточно «веры» в IMU при шумном VO без того чтобы измерение полностью
    игнорировалось.
    """

    name = "A3"

    def __init__(
        self,
        sigma_a: float = 0.05,
        sigma_g: float = 0.005,
        sigma_ba: float = 1e-8,
        sigma_bg: float = 1e-8,
        sigma_p_meas: float = 0.1,
        sigma_theta_meas: float = 0.01,
        min_scale_m: float = 0.01,
        initial_p_bias_acc: float = 1e-6,
        initial_p_bias_gyro: float = 1e-6,
        redetect_threshold: int = 2000,
        fast_threshold: int = 25,
    ) -> None:
        self.sigma_a = float(sigma_a)
        self.sigma_g = float(sigma_g)
        self.sigma_ba = float(sigma_ba)
        self.sigma_bg = float(sigma_bg)
        self.sigma_p_meas = float(sigma_p_meas)
        self.sigma_theta_meas = float(sigma_theta_meas)
        self.min_scale_m = float(min_scale_m)
        self.initial_p_bias_acc = float(initial_p_bias_acc)
        self.initial_p_bias_gyro = float(initial_p_bias_gyro)
        self.redetect_threshold = int(redetect_threshold)
        self.fast_threshold = int(fast_threshold)

        self._calib: Optional[Calibration] = None
        self._core: Optional[MonoFastLkCore] = None
        self._ekf: Optional[EKF] = None
        # 4x4 матрицы для удобства композиции
        self._T_imu_cam: Optional[np.ndarray] = None
        self._T_cam_imu: Optional[np.ndarray] = None
        # Последняя VO-измеренная T_w_cam (база для composite next step)
        self._T_w_cam_meas: Optional[np.ndarray] = None
        # Последняя оценённая T_w_cam, выходящая наружу (для случая lost-кадров)
        self._last_pose_out: Pose = Pose.identity()
        # Чтобы оценить v0 при инициализации
        self._frame0_gt: Optional[Pose] = None
        self._frame0_ts: Optional[float] = None
        # Prev gray для MonoFastLkCore
        self._prev_gray: Optional[np.ndarray] = None
        self._prev_ts: Optional[float] = None
        # State machine
        self._initialized: bool = False
        # R_meas пересобирается в reset()
        self._R_meas: Optional[np.ndarray] = None

    def reset(self, calibration: Calibration) -> None:
        if calibration.T_imu_cam is None:
            raise ValueError(
                "A3 requires Calibration.T_imu_cam; "
                "dataset must provide IMU↔camera extrinsic"
            )
        self._calib = calibration
        self._core = MonoFastLkCore(
            focal=calibration.focal_x(),
            pp=calibration.principal_point(),
            redetect_threshold=self.redetect_threshold,
            fast_threshold=self.fast_threshold,
        )
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
        self._prev_gray = None
        self._prev_ts = None
        self._initialized = False
        # R_meas: диагональ из σ позиции и σ ориентации
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
            raise RuntimeError("MonoImuEKF.reset() must be called before process()")

        # ------------------------------------------------------------------
        # Кадр 0 — инициализация EKF из gt[0] (если есть). Скорость пока
        # неизвестна (узнаем на кадре 1), поэтому стартуем с v=0 и поправляем
        # её на кадре 1 — это даёт несколько кадров transient, но без второго
        # raw-доступа к gt[1].
        # ------------------------------------------------------------------
        if not self._initialized:
            self._init_ekf(frame)
            self._prev_gray = frame.left
            self._prev_ts = frame.timestamp
            self._frame0_gt = frame.gt_pose
            self._frame0_ts = frame.timestamp
            self._T_w_cam_meas = self._ekf_pose_in_cam_frame()
            self._last_pose_out = Pose.from_matrix(self._T_w_cam_meas)
            return self._last_pose_out

        # ------------------------------------------------------------------
        # Frame 1: уточняем v0 ДО первой predict-итерации. Без этого
        # propagate начинает с v=0, predict даёт почти нулевое смещение
        # камеры, scale per-step < min_scale_m → VO update пропускается,
        # EKF расходится в predict-only режиме. Эта правка — единственный
        # дополнительный «доступ к GT», ровно как у A1 (scale из ||gt[1]-gt[0]||).
        # Если GT отсутствует — пропускаем, и EKF/IMU сами сходятся за
        # несколько кадров (медленнее, но без полного развала).
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
                # v_imu_world ≈ v_cam_world; ω × r-поправка < 1% (см. docstring).
                self._ekf.state.v = v0_world.astype(np.float64)
            self._frame0_gt = None  # one-shot

        # ------------------------------------------------------------------
        # Predict: пропагация EKF по IMU-семплам кадра (sync-режим = 1 семпл).
        # dt = frame.ts - prev.ts. Чем «толще» kitti_raw sync (~100 ms), тем
        # больше пропагация делается за один Euler-step; для loosely-coupled
        # это терпимо.
        # ------------------------------------------------------------------
        if self._prev_ts is None:
            self._prev_ts = frame.timestamp
        dt_frame = float(frame.timestamp - self._prev_ts)
        # Пропагируем для каждого IMU-семпла; в sync-режиме это ровно 1 семпл.
        if frame.imu:
            n_imu = len(frame.imu)
            # Если несколько семплов на кадр (unsync-режим в будущем) — делим dt;
            # в sync он будет 1, и dt_step == dt_frame.
            dt_step = dt_frame / max(n_imu, 1)
            for imu_sample in frame.imu:
                self._ekf.predict(imu_sample, dt_step)
        elif dt_frame > 0:
            # IMU отсутствует на этом кадре — пропустить predict. EKF просто
            # «стоит на месте» в state, но scale per-step с дыркой будет
            # завышен; для KITTI Raw такого не бывает (на каждый кадр есть OXTS).
            pass

        # ------------------------------------------------------------------
        # VO step: получаем unit-norm relative pose из MonoFastLkCore.
        # ------------------------------------------------------------------
        R_cv, t_cv, ok = self._core.step(self._prev_gray, frame.left)
        # Независимо от успеха VO — обновляем prev_gray (так делал A1)
        self._prev_gray = frame.left
        self._prev_ts = frame.timestamp

        if not ok or R_cv is None or t_cv is None:
            # VO потеряла трекинг — публикуем pose из EKF (т.е. чистое IMU
            # предсказание). Возвращаем НЕ-None, потому что у нас есть оценка,
            # пусть и без свежей коррекции; lost-кадров runner считает по
            # process()→None, что зарезервировано для полной потери оценки.
            self._last_pose_out = Pose.from_matrix(self._ekf_pose_in_cam_frame())
            return self._last_pose_out

        # Forward-axis dominance gate (KITTI-specific): на остановках или при
        # backward движении cv2.recoverPose возвращает шум в направлении
        # перемещения; такие measurement-ы только сбивают EKF (см. эксперимент
        # в [stages/07_a3_implementation.md](../../stages/07_a3_implementation.md)).
        # Аналог heuristic gate в A1 (`[mono_fast_lk.py](mono_fast_lk.py)`),
        # но без проверки на скаляр scale (мы её делаем после
        # IMU-предсказания, см. ниже).
        t_x, t_y, t_z = float(t_cv[0, 0]), float(t_cv[1, 0]), float(t_cv[2, 0])
        forward_dominant = abs(t_z) > abs(t_x) and abs(t_z) > abs(t_y)
        if not forward_dominant:
            self._last_pose_out = Pose.from_matrix(self._ekf_pose_in_cam_frame())
            return self._last_pose_out

        # cv2.recoverPose → SE(3) inverse в стиле A1.
        R_rel = R_cv.T
        t_rel_unit = -R_cv.T @ t_cv  # (3, 1), unit-norm

        # ------------------------------------------------------------------
        # Scale per-step из IMU prediction.
        #
        # p̂_w_cam_predict — куда EKF (после predict, до update) думает что
        # сейчас находится камера. p_w_cam_prev_meas — последняя VO-измеренная
        # позиция камеры. Их разность в world frame даёт ожидаемый сдвиг.
        # ------------------------------------------------------------------
        T_w_cam_predict = self._ekf_pose_in_cam_frame()
        T_w_cam_prev = self._T_w_cam_meas
        delta_world = T_w_cam_predict[:3, 3] - T_w_cam_prev[:3, 3]
        scale = float(np.linalg.norm(delta_world))

        if scale < self.min_scale_m:
            # Слишком маленький предсказанный сдвиг — VO измерение становится
            # шумным (направление не определено). Skip update, публикуем pose
            # из EKF (после predict).
            self._last_pose_out = Pose.from_matrix(T_w_cam_predict)
            return self._last_pose_out

        # ------------------------------------------------------------------
        # Собираем VO-измерение в cam frame и конвертируем в IMU frame.
        # ------------------------------------------------------------------
        T_rel = np.eye(4, dtype=np.float64)
        T_rel[:3, :3] = R_rel
        T_rel[:3, 3] = (scale * t_rel_unit).reshape(3)

        T_w_cam_meas_new = T_w_cam_prev @ T_rel
        T_w_imu_meas = T_w_cam_meas_new @ self._T_cam_imu

        pose_meas_imu = Pose.from_matrix(T_w_imu_meas)

        # ------------------------------------------------------------------
        # EKF update. После update сохраняем new T_w_cam_meas (поза cam в world
        # после ekf-коррекции) как базу для следующего scale per-step.
        # ------------------------------------------------------------------
        self._ekf.update_pose(pose_meas_imu, self._R_meas)
        T_w_cam_after_update = self._ekf_pose_in_cam_frame()
        self._T_w_cam_meas = T_w_cam_after_update

        self._last_pose_out = Pose.from_matrix(T_w_cam_after_update)
        return self._last_pose_out

    def requirements(self) -> AlgorithmRequirements:
        return AlgorithmRequirements(needs_stereo=False, needs_imu=True)

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------
    def _init_ekf(self, frame: FrameData) -> None:
        """Инициализация EKF state по первому кадру.

        Если у frame.gt_pose есть значение — T_w_cam_0 = gt[0] (обычно
        identity в KittiRaw), и T_w_imu_0 = T_w_cam_0 ∘ T_cam_imu.
        Иначе T_w_imu_0 = identity (warning: транзиент будет дольше).
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
        # Инициализация P: позиция почти точная (gt), ориентация в пределах
        # миллиметрового шума, скорость нулевая (быстро сходится на frame 1
        # из v0_world корректировки), bias очень узкая полоса (1e-6) — это
        # эмпирически снижает scale drift на drive_0016 с 35 м до 24 м
        # (см. эксперименты в [stages/07_a3_implementation.md](../../stages/07_a3_implementation.md), раздел tuning).
        # Биас всё равно эволюционирует через process noise sigma_ba/sigma_bg
        # на каждом predict.
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
