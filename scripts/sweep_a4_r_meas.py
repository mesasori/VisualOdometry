"""Sweep по R_meas для A4 на drive_0018 (Phase 3 Этапа 7).

Запускает A4 с разными (sigma_p_meas, sigma_theta_meas), печатает таблицу
ATE_rmse / ATE_max / drift / time/frame. Результаты не сохраняются на диск
(save=False), только в stdout — потому что это эксплоратория.

Использование (выполняется из workspace root):
    venv/bin/python -m scripts.sweep_a4_r_meas
"""
from __future__ import annotations

import sys
import time
from pathlib import Path

# Sweep 1: фиксируем sigma_theta = 0.01, варьируем sigma_p.
SWEEP1_P = [0.02, 0.05, 0.1, 0.2, 0.5]
SWEEP1_THETA = 0.01

# Sweep 2: с лучшим sigma_p из шага 1, варьируем sigma_theta.
SWEEP2_THETA = [0.005, 0.02]  # 0.01 уже было в sweep 1, не повторяем


def _run_one(sigma_p: float, sigma_theta: float, drive_date: str, drive: str) -> dict:
    """Один прогон A4 с заданными sigmas, возвращает метрики."""
    from src.algorithms import import_all_algorithms
    from src.algorithms.stereo_imu_ekf import StereoImuEKF
    from src.datasets import import_all_datasets
    from src.datasets.kitti_raw import KittiRaw
    from src.eval.runner import Runner

    import_all_algorithms()
    import_all_datasets()

    ds = KittiRaw("kitti_data/raw", drive_date, drive)
    algo = StereoImuEKF(sigma_p_meas=sigma_p, sigma_theta_meas=sigma_theta)
    runner = Runner(seed=42)
    t0 = time.perf_counter()
    result = runner.run(algo, ds, save=False)
    elapsed = time.perf_counter() - t0
    ate = result.metrics.get("ate", {})
    return {
        "sigma_p": sigma_p,
        "sigma_theta": sigma_theta,
        "ate_rmse": ate.get("rmse", float("nan")),
        "ate_max": ate.get("max", float("nan")),
        "ate_mean": ate.get("mean", float("nan")),
        "drift": result.metrics.get("final_drift_m", float("nan")),
        "time_per_frame_ms": result.time_per_frame_ms,
        "success_rate": result.success_rate,
        "elapsed_s": elapsed,
    }


def main(drive_date: str = "2011_09_30", drive: str = "0018") -> int:
    print(f"=== Sweep A4 R_meas on {drive_date}/drive_{drive} ===\n")

    rows: list[dict] = []

    print("--- Sweep 1: vary sigma_p, sigma_theta=0.01 ---")
    print(f"{'sigma_p':>8}  {'sigma_theta':>11}  {'ATE rmse':>10}  {'ATE max':>10}  "
          f"{'drift':>10}  {'success':>8}  {'ms/fr':>7}  {'wall_s':>7}")
    for sigma_p in SWEEP1_P:
        row = _run_one(sigma_p, SWEEP1_THETA, drive_date, drive)
        rows.append(row)
        print(
            f"{row['sigma_p']:>8.3f}  {row['sigma_theta']:>11.4f}  "
            f"{row['ate_rmse']:>10.3f}  {row['ate_max']:>10.3f}  "
            f"{row['drift']:>10.3f}  {row['success_rate']*100:>7.1f}%  "
            f"{row['time_per_frame_ms']:>7.2f}  {row['elapsed_s']:>7.1f}"
        )

    # Победитель sweep 1 — минимальный ATE_rmse с условием max < 3*rmse (нет outliers).
    sweep1 = [r for r in rows if r["ate_max"] < 3.0 * r["ate_rmse"]]
    if not sweep1:
        sweep1 = rows  # фолбэк, если ни один не прошёл — берём общий минимум
    best1 = min(sweep1, key=lambda r: r["ate_rmse"])
    print(f"\n--> best sigma_p={best1['sigma_p']} (ATE rmse={best1['ate_rmse']:.3f} m)")

    print(f"\n--- Sweep 2: vary sigma_theta, sigma_p={best1['sigma_p']} ---")
    print(f"{'sigma_p':>8}  {'sigma_theta':>11}  {'ATE rmse':>10}  {'ATE max':>10}  "
          f"{'drift':>10}  {'success':>8}  {'ms/fr':>7}  {'wall_s':>7}")
    for sigma_theta in SWEEP2_THETA:
        row = _run_one(best1["sigma_p"], sigma_theta, drive_date, drive)
        rows.append(row)
        print(
            f"{row['sigma_p']:>8.3f}  {row['sigma_theta']:>11.4f}  "
            f"{row['ate_rmse']:>10.3f}  {row['ate_max']:>10.3f}  "
            f"{row['drift']:>10.3f}  {row['success_rate']*100:>7.1f}%  "
            f"{row['time_per_frame_ms']:>7.2f}  {row['elapsed_s']:>7.1f}"
        )

    # Финальный победитель среди всех 7 прогонов.
    final_pool = [r for r in rows if r["ate_max"] < 3.0 * r["ate_rmse"]] or rows
    best = min(final_pool, key=lambda r: r["ate_rmse"])
    print(
        f"\n=== Best overall: sigma_p={best['sigma_p']}, sigma_theta={best['sigma_theta']}"
        f"  ATE rmse={best['ate_rmse']:.3f} m, ATE max={best['ate_max']:.3f} m,"
        f" drift={best['drift']:.3f} m ==="
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
