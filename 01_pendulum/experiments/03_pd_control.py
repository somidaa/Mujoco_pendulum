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
from src.controller import pd_control

XML_PATH = PROJECT_ROOT / "model" / "pendulum.xml"
OUT_PATH = PROJECT_ROOT / "outputs" / "csv" / "pd_control.csv"

def main():

    model = load_model(str(XML_PATH))
    data = create_data(model)

    data.qpos[0] = np.pi / 4
    data.qvel[0] = 0.0

    q_desired = 0.0
    Kp = 10.0
    Kd = 2.0

    logger = Logger()

    with mujoco.viewer.launch_passive(model, data) as viewer:

        while viewer.is_running() and data.time < 10.0:
            step_start = time.time()

            u = pd_control(data.qpos[0], data.qvel[0], q_desired, Kp, Kd)

            data.ctrl[0] = u

            mujoco.mj_step(model, data)

            logger.record(data)

            viewer.sync()

            elapsed = time.time() - step_start
            time_step = model.opt.timestep

            if elapsed < time_step:
                time.sleep(time_step - elapsed)

    logger.save_csv(str(OUT_PATH))

    print(f"Saved to {OUT_PATH}")

if __name__ == "__main__":
    main()
