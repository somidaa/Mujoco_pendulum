import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

import csv
import numpy as np
import mujoco

from src.model import load_model, create_data
from src.simulation import run_simulation
from src.logger import Logger
from src.controller import pd_control
from src.metrics import settling_time, overshoot, control_effort, rms
from src.visualization import plot_pd_comparison

XML_PATH = PROJECT_ROOT / "model" / "pendulum.xml"
OUT_CSV = PROJECT_ROOT / "outputs" / "csv"
OUT_PLOTS = PROJECT_ROOT / "outputs" / "plots"

Q0 = np.pi / 4
Q_DESIRED = 0.0
DURATION = 10.0

KPS = [1.0, 5.0, 10.0, 20.0]
KDS = [0.1, 0.5, 1.0, 2.0]


def run_pd(model, data, kp, kd):
    # 跑一组 PD 控制并记录
    mujoco.mj_resetData(model, data)
    data.qpos[0] = Q0
    data.qvel[0] = 0.0

    logger = Logger()
    controller = lambda q, dq, t: pd_control(q, dq, Q_DESIRED, kp, kd)
    run_simulation(model, data, DURATION, controller=controller, logger=logger)
    return logger


def main():
    model = load_model(str(XML_PATH))
    data = create_data(model)

    rows = []
    kp_series = {}   # Kd = 1.0 时不同 Kp 的响应
    kd_series = {}   # Kp = 10 时不同 Kd 的响应

    for kp in KPS:
        for kd in KDS:
            logger = run_pd(model, data, kp, kd)
            logger.save_csv(str(OUT_CSV / f"pd_kp{kp:g}_kd{kd:g}.csv"))

            t, q, dq, u = logger.time, logger.q, logger.dq, logger.u
            rows.append({
                "kp": kp, "kd": kd,
                "settling_time": settling_time(t, q, Q_DESIRED),
                "overshoot": overshoot(q, Q_DESIRED),
                "rms_error": rms(np.asarray(q) - Q_DESIRED),
                "control_effort": control_effort(u),
            })

            if abs(kd - 1.0) < 1e-9:
                kp_series[f"{kp:g}"] = (t, q)
            if abs(kp - 10.0) < 1e-9:
                kd_series[f"{kd:g}"] = (t, q)

            m = rows[-1]
            print(f"Kp={kp:4.1f} Kd={kd:4.1f} -> "
                  f"settling={m['settling_time']:.2f}s "
                  f"overshoot={m['overshoot']*100:5.1f}% "
                  f"rms={m['rms_error']:.4f}")

    # 指标表
    with (OUT_CSV / "pd_metrics.csv").open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)
    print(f"saved {OUT_CSV / 'pd_metrics.csv'}")

    # 对比图
    plot_pd_comparison(kp_series, "Kp", str(OUT_PLOTS / "pd_kp_comparison.png"))
    plot_pd_comparison(kd_series, "Kd", str(OUT_PLOTS / "pd_kd_comparison.png"))


if __name__ == "__main__":
    main()
