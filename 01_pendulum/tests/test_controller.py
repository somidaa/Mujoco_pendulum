import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from src.controller import pd_control


def test_pd_control():
    q_desired = 0.0
    Kp = 10.0
    Kd = 2.0

    test_cases = [
        (0.5, 0.0),
        (-0.5, 0.0),
        (0.5, 1.0),
        (0.5, -1.0),
        (0.0, 1.0),
        (0.0, -1.0),
    ]

    for q, q_dot in test_cases:

        u = pd_control(q, q_dot, q_desired, Kp, Kd)

        print(
            f"q={q:.2f},"
            f"qdot={q_dot:.2f},"
            f"u={u:.2f}"
        )

if __name__ == "__main__":
    test_pd_control()
