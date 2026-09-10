"""Data logging for experiments (Milestone 2).

``DataLogger`` records one row per simulation step::

    time  q  dq  u

plus any extra columns (e.g. kinetic / potential / total energy) and
can dump everything to CSV or return numpy arrays for plotting.
"""
import csv
from pathlib import Path

import numpy as np

DEFAULT_FIELDS = ["time", "q", "dq", "u"]


class DataLogger:
    """Time-series recorder with CSV load / save support."""

    def __init__(self, extra_fields=None):
        self.time = []
        self.q = []
        self.dq = []
        self.u = []
        self.fields = list(DEFAULT_FIELDS)
        for field in extra_fields or []:
            self.add_fields([field])

    # ------------------------------------------------------------------ #
    # recording
    # ------------------------------------------------------------------ #
    def add_fields(self, fields):
        """Register extra columns (e.g. ``kinetic_energy``)."""
        for field in fields:
            if field not in self.fields:
                self.fields.append(field)
                setattr(self, field, [])

    def append(self, time, q, dq, u, **extra):
        """Append one manually-specified row."""
        self.time.append(float(time))
        self.q.append(float(q))
        self.dq.append(float(dq))
        self.u.append(float(u))
        for field in self.fields[4:]:
            getattr(self, field).append(float(extra.get(field, np.nan)))

    def record(self, data, **extra):
        """Append one row read from a MuJoCo ``MjData`` object."""
        self.append(data.time, data.qpos[0], data.qvel[0], data.ctrl[0], **extra)

    def __len__(self):
        return len(self.time)

    # ------------------------------------------------------------------ #
    # export / import
    # ------------------------------------------------------------------ #
    def to_dict(self):
        """Return ``{field: np.ndarray}`` for plotting and analysis."""
        return {f: np.asarray(getattr(self, f), dtype=float) for f in self.fields}

    def save_csv(self, path):
        """Save all columns to a CSV file (creating directories)."""
        path = Path(path)
        path.parent.mkdir(parents=True, exist_ok=True)
        with path.open("w", newline="") as fh:
            writer = csv.writer(fh)
            writer.writerow(self.fields)
            for i in range(len(self)):
                writer.writerow([getattr(self, f)[i] for f in self.fields])
        return path

    @classmethod
    def load_csv(cls, path):
        """Load a CSV written by :meth:`save_csv` back into a DataLogger."""
        logger = cls()
        with open(path, newline="") as fh:
            reader = csv.reader(fh)
            header = next(reader)
            logger.add_fields(header[4:])
            for row in reader:
                values = {name: float(v) for name, v in zip(header, row)}
                logger.append(**values)
        return logger
