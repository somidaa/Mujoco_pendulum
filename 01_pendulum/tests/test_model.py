import sys
import time
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

import mujoco
import mujoco.viewer

from src.model import load_model, create_data

XML_PATH = PROJECT_ROOT / "model" / "pendulum.xml"

def main():
    model = load_model(str(XML_PATH))
    data = create_data(model)

    print("nq = ", model.nq)
    print("nv = ", model.nv)
    print("nu = ", model.nu)

    print("qpos = ", data.qpos)
    print("qvel = ", data.qvel)
    print("qacc = ", data.qacc)
    print("time = ", data.time)

    with mujoco.viewer.launch_passive(model, data) as viewer:
        while viewer.is_running():
            time.sleep(0.01)

if __name__ == "__main__":
    main()
