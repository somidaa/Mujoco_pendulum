"""Milestone 3 + 5 / Experiment C + D: PD control and the Kp x Kd sweep.

Part 1  Kp sweep   Kp in [1, 5, 10, 20],  Kd = 1      -> pd_kp_comparison.png
Part 2  Kd sweep   Kd in [0.1, 0.5, 1, 2], Kp = 10    -> pd_kd_comparison.png
Part 3  Full 4x4 sweep -> pd_metrics.csv + settling-time table figure
Part 4  Tuned run  Kp = 20, Kd = 6  -> pd_response.png + video
                                        outputs/videos/pd_control.mp4

Every run is saved as outputs/csv/pd_kp<Kp>_kd<Kd>.csv.
"""
import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

import csv  # noqa: E402
import numpy as np  # noqa: E402
import mujoco  # noqa: E402

from src.model import load_model, create_data  # noqa: E402
from src.simulation import run_simulation  # noqa: E402
from src.controller import PDController  # noqa: E402
from src.metrics import evaluate_run  # noqa: E402
from src.visualization import (  # noqa: E402
    plot_pd_response,
    plot_pd_gain_comparison,
    plot_pd_metrics_table,
    render_pendulum_video,
)

OUT_CSV = PROJECT_ROOT / "outputs" / "csv"
OUT_VIDEO = PROJECT_ROOT / "outputs" / "videos"
DURATION = 10.0
Q0 = np.radians(45.0)
Q_DESIRED = 0.0

KPS = [1.0, 5.0, 10.0, 20.0]
KDS = [0.1, 0.5, 1.0, 2.0]
TUNED = (20.0, 6.0)


def reset_pendulum(model, data):
    mujoco.mj_resetData(model, data)
    data.qpos[0] = Q0
    data.qvel[0] = 0.0
    mujoco.mj_forward(model, data)


def run_pd(model, data, kp, kd):
    reset_pendulum(model, data)
    controller = PDController(kp=kp, kd=kd, q_desired=Q_DESIRED)
    logger = run_simulation(model, data, DURATION, controller=controller)
    return logger


def main():
    model = load_model()
    data = create_data(model)

    # ---- part 1: Kp sweep (Kd fixed) ----
    kp_series = {}
    for kp in KPS:
        logger = run_pd(model, data, kp, 1.0)
        logger.save_csv(OUT_CSV / f"pd_kp{kp:g}_kd1.csv")
        d = logger.to_dict()
        kp_series[f"{kp:g}"] = (d["time"], d["q"])
        m = evaluate_run(d["time"], d["q"], d["u"], Q_DESIRED)
        print(f"Kp={kp:4.1f} Kd=1.0 -> settling={m['settling_time']:.2f}s "
              f"overshoot={m['overshoot']*100:5.1f}% rms={m['rms_error']:.4f}")
    plot_pd_gain_comparison(kp_series, gain_name="Kp", stem="pd_kp_comparison")

    # ---- part 2: Kd sweep (Kp fixed) ----
    kd_series = {}
    for kd in KDS:
        logger = run_pd(model, data, 10.0, kd)
        logger.save_csv(OUT_CSV / f"pd_kp10_kd{kd:g}.csv")
        d = logger.to_dict()
        kd_series[f"{kd:g}"] = (d["time"], d["q"])
        m = evaluate_run(d["time"], d["q"], d["u"], Q_DESIRED)
        print(f"Kp=10.0 Kd={kd:4.1f} -> settling={m['settling_time']:.2f}s "
              f"overshoot={m['overshoot']*100:5.1f}% rms={m['rms_error']:.4f}")
    plot_pd_gain_comparison(kd_series, gain_name="Kd", stem="pd_kd_comparison")

    # ---- part 3: full 4x4 sweep with metrics ----
    rows = []
    for kp in KPS:
        for kd in KDS:
            logger = run_pd(model, data, kp, kd)
            logger.save_csv(OUT_CSV / f"pd_kp{kp:g}_kd{kd:g}.csv")
            d = logger.to_dict()
            m = evaluate_run(d["time"], d["q"], d["u"], Q_DESIRED)
            rows.append({"kp": kp, "kd": kd, **m})
    metrics_path = OUT_CSV / "pd_metrics.csv"
    with metrics_path.open("w", newline="") as fh:
        writer = csv.DictWriter(fh, fieldnames=["kp", "kd", "settling_time",
                                                "overshoot", "rms_error",
                                                "control_effort"])
        writer.writeheader()
        for row in rows:
            writer.writerow(row)
    plot_pd_metrics_table(rows)
    print(f"\nsweep metrics -> {metrics_path}")

    # ---- part 4: tuned run + response plot + video ----
    kp, kd = TUNED
    logger = run_pd(model, data, kp, kd)
    logger.save_csv(OUT_CSV / f"pd_kp{kp:g}_kd{kd:g}.csv")
    d = logger.to_dict()
    plot_pd_response(d["time"], d["q"], d["u"], Q_DESIRED,
                     title=f"PD control (Kp = {kp:g}, Kd = {kd:g})")
    m = evaluate_run(d["time"], d["q"], d["u"], Q_DESIRED)
    print(f"\ntuned run Kp={kp} Kd={kd}: settling={m['settling_time']:.2f}s "
          f"overshoot={m['overshoot']*100:.1f}% "
          f"rms_error={m['rms_error']:.4f}")

    render_pendulum_video(d["time"], d["q"],
                          OUT_VIDEO / "pd_control.mp4")
    print("\nPD control outputs written to outputs/csv, outputs/plots "
          "and outputs/videos.")


if __name__ == "__main__":
    main()
