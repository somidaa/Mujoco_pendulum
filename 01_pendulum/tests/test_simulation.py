import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

import numpy as np

from src.model import load_model, create_data
from src.simulation import run_simulation

XML_PATH = PROJECT_ROOT / "model" / "pendulum.xml"


def test_time_increases():
    model = load_model(str(XML_PATH))
    data = create_data(model)
    run_simulation(model, data, 1.0)
    assert data.time > 0.5


def test_states_finite():
    model = load_model(str(XML_PATH))
    data = create_data(model)
    data.qpos[0] = np.pi / 4
    run_simulation(model, data, 1.0)
    assert np.all(np.isfinite(data.qpos))
    assert np.all(np.isfinite(data.qvel))


if __name__ == "__main__":
    test_time_increases()
    test_states_finite()
    print("test_simulation OK")
