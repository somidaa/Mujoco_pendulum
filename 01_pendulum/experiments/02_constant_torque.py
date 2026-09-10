import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from src.model import load_model, create_data
from src.logger import Logger
from src.simulation import run_simulation

XML_PATH = PROJECT_ROOT / "model" / "pendulum.xml"
OUT_PATH = PROJECT_ROOT / "outputs" / "csv" / "constant_torque.csv"

def main():
    model = load_model(str(XML_PATH))
    data = create_data(model)

    u = 1.0  # 恒定力矩 (N*m)
    logger = Logger()

    run_simulation(model, data, 10.0, controller=lambda q, dq, t: u, logger=logger)

    logger.save_csv(str(OUT_PATH))
    print(f"Saved to {OUT_PATH}")

if __name__ == "__main__":
    main()
