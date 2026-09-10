import sys
import time
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

import mujoco
import mujoco.viewer
import numpy as np
from src.model import load_model, create_data
from src.logger import Logger
from src.visualization import plot_free_dynamics

XML_PATH = PROJECT_ROOT / "model" / "pendulum.xml"
CSV_PATH = PROJECT_ROOT / "outputs" / "csv" / "free_dynamics.csv"
FIG_PATH = PROJECT_ROOT / "outputs" / "plots" / "free_dynamics.png"

def main():
    model = load_model(str(XML_PATH))
    data = create_data(model)

    q0 = np.pi / 4
    data.qpos[0] = q0
    data.qvel[0] = 0.0
    data.ctrl[0] = 0.0

    logger = Logger()

    with mujoco.viewer.launch_passive(model, data) as viewer:

        while viewer.is_running() and data.time < 10.0:

            step_start = time.time()

            mujoco.mj_step(model, data)

            logger.record(data)

            viewer.sync()
            elapsed = time.time() - step_start
            timestep = model.opt.timestep

            if elapsed < timestep:
                time.sleep(timestep - elapsed)

    logger.save_csv(str(CSV_PATH))
    print(f"Saved to {CSV_PATH}")

    plot_free_dynamics(str(CSV_PATH), str(FIG_PATH))

if __name__ == "__main__":
    main()
