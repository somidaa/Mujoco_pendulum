"""Reusable simulation loop (used by every experiment).

The single entry point of the project::

    run_simulation(model, data, duration, controller=None, logger=None)

The loop is exactly the control loop shape you used for the youBot::

    read state (q, dq)
        -> controller(q, dq, t) -> u
        -> data.ctrl = u
        -> mj_step(model, data)      # MuJoCo performs the integration
        -> log

The only difference from ``NextState()`` is that MuJoCo now does the
dynamics integration for us.
"""
import mujoco

from src.energy import compute_energies
from src.logger import DataLogger


def run_simulation(model, data, duration, controller=None, logger=None,
                   record_energy=False):
    """Run a MuJoCo simulation for ``duration`` seconds.

    Parameters
    ----------
    model : mujoco.MjModel
    data : mujoco.MjData
    duration : float
        simulation time in seconds.
    controller : callable, optional
        ``controller(q, dq, t) -> u``; ``None`` means zero torque
        (free dynamics).
    logger : DataLogger, optional
        recorder; a fresh one is created if ``None``.
    record_energy : bool
        also record kinetic / potential / total energy each step.

    Returns
    -------
    DataLogger
        the logger holding the recorded time series.
    """
    if logger is None:
        logger = DataLogger()
    if record_energy:
        logger.add_fields(["kinetic_energy", "potential_energy", "total_energy"])

    n_steps = int(round(duration / model.opt.timestep))
    for _ in range(n_steps):
        q = float(data.qpos[0])
        dq = float(data.qvel[0])
        u = 0.0 if controller is None else controller(q, dq, data.time)
        data.ctrl[0] = u

        mujoco.mj_step(model, data)

        if record_energy:
            kinetic, potential, total = compute_energies(model, data)
            logger.record(data, kinetic_energy=kinetic,
                          potential_energy=potential, total_energy=total)
        else:
            logger.record(data)
    return logger
