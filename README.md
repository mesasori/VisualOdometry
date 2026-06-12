# Visual Odometry Research Stand

Стенд для сравнительного исследования алгоритмов **Visual Odometry / Visual-Inertial Odometry**
на KITTI Odometry и KITTI Raw. Цель ВКР — реализовать единый каркас из 4 алгоритмов
(A1..A4), прогнать на одних и тех же сценах и сравнить по `ATE / RPE / drift`.

| ID | Алгоритм | Stage doc | Состояние |
|---|---|---|---|
| A1 | Mono FAST + LK + Essential |
| A2 | Stereo SGBM + FAST + LK + PnP |
| A3 | Mono + IMU EKF (loosely-coupled VIO) |
| A4 | Stereo + IMU EKF 
---

## Быстрый старт

```bash
python3 -m venv venv && source venv/bin/activate
pip install -r requirements.txt

python -m src.gui                    # ← главное окно, две вкладки
```

В открывшемся окне:

1. **Вкладка «Запустить прогон»** — выбери Dataset (`kitti_odometry` или
   `kitti_raw`), Algorithm (фильтр под совместимость) и Sequence/Drive →
   нажми «▶ Запустить». Справа появится live-визуализация: текущий кадр +
   накапливаемая траектория поверх ground truth. Кнопка «⏹ Стоп» прерывает
   прогон.
2. По завершении предложит перейти на **«Сравнить траектории»** — там все
   сохранённые `.txt` сгруппированы по сцене. Отметь чек-боксами нужные
   алгоритмы и нажми «📊 Показать наложение» — отдельное окно с N
   траекторий + GT + таблицей метрик (ATE / RPE / drift / success /
   ms/frame). По умолчанию все траектории выравниваются Umeyama-методом.

---

## CLI

```bash
python -m src.eval.runner --algo A1 --dataset kitti_odometry --sequence 05 --seed 42
python -m src.eval.runner --algo A2 --dataset kitti_odometry --sequence 08 --seed 42
python -m src.eval.runner --algo A3 --dataset kitti_raw --date 2011_09_30 --drive 0018 --seed 42
python -m src.eval.runner --algo A4 --dataset kitti_raw --date 2011_09_30 --drive 0018 --seed 42

python -m src.fusion.ekf --selftest                  # synthetic EKF
python -m src.fusion.ekf --smoke-raw \
    --date 2011_09_30 --drive 0018                   # predict-only EKF на raw
```

Артефакты пишутся одинаково из GUI и CLI:

* `results/trajectories/<algo>__<dataset>.txt` — KITTI-формат (12 чисел/строка).
* `results/metrics/<algo>__<dataset>.json` — полная сводка ATE / RPE / drift / time/frame.
* При cancel'е из GUI добавляется суффикс `__cancelled` к имени файла.

---

## Архитектура

Чистый event-based pipeline. Каждый алгоритм описывает свои **требования**
(`AlgorithmRequirements`), каждый датасет — свои **возможности**
(`DatasetCapabilities`). `Runner` валидирует совместимость и крутит
`Dataset.frames()` → `algorithm.process(frame)` → `Pose`, сохраняет в
`results/`. GUI использует тот же Runner в worker-потоке + очередь для
прокидывания прогресса/кадров в LiveVisualizer.

```
                    ┌─────────────────────┐
                    │  python -m src.gui  │
                    └──────────┬──────────┘
                               │
                ┌──────────────┴──────────────┐
                │      MainApp (Notebook)     │
                └──────┬───────────────┬──────┘
                       │               │
                       ▼               ▼
                ┌─────────────┐ ┌──────────────────┐
                │ ChooserTab  │ │  ComparatorTab   │
                └──────┬──────┘ └────────┬─────────┘
                       │                 │
                       ▼                 ▼
            ┌──────────────────┐  ┌─────────────────┐
            │ Worker thread:   │  │ discover *.txt  │
            │ Runner.run(...)  │  │ → Treeview      │
            └─────┬────────────┘  └────────┬────────┘
                  │ queue.Queue            │ checked items
                  ▼                        ▼
        ┌──────────────────┐      ┌─────────────────────┐
        │ LiveVisualizer   │      │ plot_overlay_multi  │
        │ (FigureCanvasTk) │      │ (mpl window)        │
        └──────────────────┘      └─────────────────────┘
```

### Структура `src/`

```
src/
├── vo/
│   └── interface.py            # Pose, FrameData, IMUSample, AlgorithmRequirements,
│                               # DatasetCapabilities, VOAlgorithm ABC,
│                               # is_compatible(req, caps) helper
├── algorithms/
│   ├── base.py                 # @register_algorithm, ALGORITHMS registry
│   ├── mono_fast_lk.py         # A1
│   ├── mono_orb_lk.py          # A1-alt (используется внутри A2/A4)
│   ├── stereo_sgbm_pnp.py      # A2
│   ├── mono_imu_ekf.py         # A3
│   ├── stereo_imu_ekf.py       # A4
│   └── orbslam3_wrapper.py     # A5 (заглушка)
├── datasets/
│   ├── base.py                 # Dataset ABC
│   ├── kitti_odometry.py       # KITTI Odometry
│   ├── kitti_raw.py            # KITTI Raw (pykitti)
│   └── samsung_s23.py          # S23 demo (не в реестре, не показывается в GUI)
├── fusion/
│   ├── imu_model.py            # IMU predict (страпдаун)
│   └── ekf.py                  # EKF state + update step
├── eval/
│   ├── runner.py               # Runner.run(...) с progress/frame/cancel callbacks
│   ├── metrics.py              # обёртки над evo (ATE/RPE/drift)
│   └── benchmark.py            # KITTI_ODO_RAW_PAIRS mapping
└── gui/
    ├── __main__.py             # точка входа `python -m src.gui --tab ...`
    ├── main_app.py             # MainApp(tk.Tk) — Notebook(Chooser, Comparator)
    ├── chooser.py              # ChooserTab + worker + cancel
    ├── comparator.py           # ComparatorTab + plot_overlay_multi
    └── live_visualizer.py      # LiveVisualizer (FigureCanvasTkAgg, ~20fps throttle)
```

`old_structure/` — монолитный VO-стенд из предыдущей версии диплома
(`run_gui.py`, `visualize_vo.py`, `run_vo_s23.py`). Оставлен как
справочный архив; для всех актуальных задач используется
`python -m src.gui` и `python -m src.eval.runner`.

---

## Где взять KITTI

[KITTI Visual Odometry Benchmark](https://www.cvlibs.net/datasets/kitti/eval_odometry.php)
+ [KITTI Raw recordings](https://www.cvlibs.net/datasets/kitti/raw_data.php).
Подробная раскладка по директориям + используемые сцены — в
[stages/03_data_layout.md](stages/03_data_layout.md). Краткий канонический
layout:

```
kitti_data/
├── poses/<XX>.txt                # GT для KITTI Odometry seq 00..10
├── sequences/<XX>/
│   ├── calib.txt
│   ├── times.txt
│   ├── image_0/                  # left grayscale
│   └── image_1/                  # right grayscale (stereo)
└── raw/<YYYY_MM_DD>/<YYYY_MM_DD>_drive_XXXX_sync/
    ├── image_00/data/            # left grayscale
    ├── image_01/data/            # right grayscale
    └── oxts/data/                # IMU + GPS
```

KITTI Raw нужен только для A3/A4 (требуют IMU). Маппинг между Odometry
sequences и Raw drives (drive_0018 ↔ Odo 05, drive_0042 ↔ Odo 01, ...) —
в [src/eval/benchmark.py](src/eval/benchmark.py).

---

## Производительность (ориентир)

| Алгоритм | KITTI seq 04 (271 fr) | drive_0018 (2762 fr) | время/кадр |
|---|---:|---:|---:|
| A1 mono+gt-scale | ATE 0.25 m | ATE 10.9 m | 17 ms |
| A2 stereo+pnp | ATE 0.05 m | ATE 9.8 m | ~70 ms |
| A3 mono+IMU EKF | n/a (no IMU) | ATE 570 m ⚠ | ~25 ms |
| A4 stereo+IMU EKF | n/a (no IMU) | ATE 6.7 m ✓ | ~95 ms |

A3 в одиночку «не умеет» удерживать масштаб на длинных сценах
(loosely-coupled mono-VIO без anchor) — см. A4
закрывает эту проблему через stereo-PnP с абсолютным scale; полностью
закроет ожидаемо A5 (ORB-SLAM3 с loop closure).

---

## Научная основа

* Scaramuzza & Fraundorfer — *Visual Odometry: Part I/II*, IEEE RAM 2011/2012
* Forster, Carlone, Dellaert, Scaramuzza — *On-Manifold Preintegration for
  Real-Time Visual-Inertial Odometry*, IEEE T-RO 2017
* Mur-Artal, Montiel, Tardós — *ORB-SLAM: a Versatile and Accurate Monocular
  SLAM System*, IEEE T-RO 2015 (и mono-inertial extension)
* Geiger et al. — *Vision meets Robotics: The KITTI Dataset*, IJRR 2013
* Grupp — *evo: Python package for the evaluation of odometry and SLAM*,
  2017 (используется как численная база для метрик)

---

**Автор:** Руслан Нигматуллин  
**Учебное заведение:** МГТУ им. Баумана  
**Год:** 2026
