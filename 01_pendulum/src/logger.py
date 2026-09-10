import csv
from pathlib import Path


class Logger:
    def __init__(self):
        self.time = []
        self.q = []
        self.dq = []
        self.u = []

        self.kinetic_energy = []
        self.potential_energy = []
        self.total_energy = []

    def record(self, data):
        # data.energy = [potential, kinetic]（MJCF 中开启了 energy flag）
        self.time.append(data.time)
        self.q.append(data.qpos[0])
        self.dq.append(data.qvel[0])
        self.u.append(data.ctrl[0])

        self.potential_energy.append(data.energy[0])
        self.kinetic_energy.append(data.energy[1])
        self.total_energy.append(data.energy[0] + data.energy[1])

    def save_csv(self, path):
        path = Path(path)
        path.parent.mkdir(parents=True, exist_ok=True)

        with path.open('w', newline="") as f:
            writer = csv.writer(f)

            writer.writerow(["time", "q", "dq", "u", "kinetic_energy", "potential_energy", "total_energy"])

            for row in zip(
                self.time,
                self.q,
                self.dq,
                self.u,
                self.kinetic_energy,
                self.potential_energy,
                self.total_energy
            ):
                writer.writerow(row)

    def load_csv(self, path):
        time = []
        q = []
        dq = []
        u = []
        kinetic_energy = []
        potential_energy = []
        total_energy = []

        with open(path, newline="") as f:
            reader = csv.reader(f)
            next(reader)
            for row in reader:
                time.append(float(row[0]))
                q.append(float(row[1]))
                dq.append(float(row[2]))
                u.append(float(row[3]))
                kinetic_energy.append(float(row[4]))
                potential_energy.append(float(row[5]))
                total_energy.append(float(row[6]))

        return time, q, dq, u, kinetic_energy, potential_energy, total_energy
