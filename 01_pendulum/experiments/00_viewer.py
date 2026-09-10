"""Milestone 1 acceptance: load the pendulum and watch it in the viewer.

Run:
    py experiments/00_viewer.py

You should see the pendulum hanging at rest. Drag with the left mouse
button to rotate the camera, scroll to zoom.
"""
import sys
import time
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

import mujoco.viewer  # noqa: E402

from src.model import load_model, create_data, print_model_info  # noqa: E402


def main():
    model = load_model()
    data = create_data(model)

    print_model_info(model)
    print("\nOpening viewer (u = 0, pendulum at rest)...")
    print("Close the viewer window to exit.\n")

    with mujoco.viewer.launch_passive(model, data) as viewer:
        while viewer.is_running():
            time.sleep(0.01)

    print("viewer closed")


if __name__ == "__main__":
    main()
