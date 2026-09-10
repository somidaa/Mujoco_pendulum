import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

import numpy as np
import mujoco

from src.logger import Logger
from src.model import load_model, create_data
from src.visualization import plot_energy

XML_PATH = PROJECT_ROOT / "model" / "pendulum.xml"
OUT_PATH = PROJECT_ROOT / "outputs" / "csv" / "energy.csv"
FIG_PATH = PROJECT_ROOT / "outputs" / "plots" / "energy.png"

def main():

    model = load_model(str(XML_PATH))
    data = create_data(model)

    data.qpos[0] = np.pi / 4
    data.qvel[0] = 0.0
    data.ctrl[0] = 0.0

    mujoco.mj_forward(model, data)

    logger = Logger()

    while data.time < 10.0:

        logger.record(data)

        mujoco.mj_step(model, data)

    logger.save_csv(str(OUT_PATH))

    print(f"CSV saved to {OUT_PATH}")

    plot_energy(str(OUT_PATH), str(FIG_PATH))

    print(f"Fig saved to {FIG_PATH}")

if __name__ == "__main__":
    main()
