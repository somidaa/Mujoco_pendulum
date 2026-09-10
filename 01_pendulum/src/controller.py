"""Controllers for the pendulum (Milestone 3).

The only control law in Project 1 is the PD controller::

    u = Kp * (q_desired - q) - Kd * dq

* ``Kp`` (proportional gain) acts like a spring pulling ``q`` toward
  ``q_desired``;
* ``Kd`` (derivative gain) acts like a damper opposing the angular
  velocity, reducing overshoot and oscillation.

``PDController`` wraps the same law as a callable so it can be passed
straight into :func:`src.simulation.run_simulation`.
"""
from dataclasses import dataclass


def pd_control(q, qdot, q_desired, kp, kd):
    """PD control law for a single-DoF system.

    Parameters
    ----------
    q : float            current angle (rad)
    qdot : float         current angular velocity (rad/s)
    q_desired : float    desired angle (rad)
    kp : float           proportional gain
    kd : float           derivative gain

    Returns
    -------
    u : float            control torque (N*m)
    """
    error = q_desired - q
    u = kp * error - kd * qdot
    return u


@dataclass
class PDController:
    """Stateful PD controller callable as ``controller(q, dq, t) -> u``."""

    kp: float
    kd: float
    q_desired: float = 0.0

    def __call__(self, q, qdot, t):
        return pd_control(q, qdot, self.q_desired, self.kp, self.kd)
