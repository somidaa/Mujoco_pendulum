"""Mechanical energy of the pendulum (Milestone 4).

Both quantities are computed from MuJoCo's own quantities:

* kinetic:      T = 1/2 * dq^T * M(q) * dq        (M  <- ``data.M``)
* potential:    V = sum_i m_i * g * z_com_i       (COM <- ``data.xipos``)

These formulas reproduce exactly the values MuJoCo reports in
``data.energy`` (``[potential, kinetic]``) when the ``energy`` flag
is enabled in the MJCF.

``data.xipos`` is only valid after ``mj_forward`` / ``mj_step`` has
updated the kinematics, so always run at least one step (or call
``mj_forward``) before reading it.
"""
import numpy as np

import mujoco


def kinetic_energy(model: mujoco.MjModel, data: mujoco.MjData) -> float:
    """T = 1/2 dq^T M(q) dq."""
    nv = model.nv
    M = np.asarray(data.M).reshape(nv, nv)
    return 0.5 * float(data.qvel @ M @ data.qvel)


def potential_energy(model: mujoco.MjModel, data: mujoco.MjData) -> float:
    """V = sum_i m_i * g * z_com_i  (datum: z = 0 in world frame)."""
    g = -float(model.opt.gravity[2])          # 9.81
    return float(np.dot(model.body_mass, g * data.xipos[:, 2]))


def total_energy(model: mujoco.MjModel, data: mujoco.MjData) -> float:
    """Mechanical energy E = T + V."""
    return kinetic_energy(model, data) + potential_energy(model, data)


def compute_energies(model: mujoco.MjModel, data: mujoco.MjData):
    """Return ``(kinetic, potential, total)`` as a tuple of floats."""
    kinetic = kinetic_energy(model, data)
    potential = potential_energy(model, data)
    return kinetic, potential, kinetic + potential


def energy_drift(energies):
    """Absolute and relative drift ``|E(t) - E(0)|`` of an energy series."""
    energies = np.asarray(energies, dtype=float)
    e0 = energies[0]
    abs_drift = np.abs(energies - e0)
    rel_drift = abs_drift / abs(e0) if abs(e0) > 1e-12 else np.zeros_like(abs_drift)
    return abs_drift, rel_drift
