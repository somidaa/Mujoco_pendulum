"""Make the project root importable for pytest (run ``py -m pytest``
from the 01_pendulum directory)."""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
