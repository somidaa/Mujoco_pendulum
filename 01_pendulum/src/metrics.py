"""Performance metrics for the control experiments (Milestone 5).

Given a closed-loop trajectory ``(t, q, u)`` with target ``q_desired``,
compute the standard step-response metrics:

* settling time   -- first time after which |q - qd| stays below a
                     tolerance for a full window;
* overshoot       -- peak error above the initial error, as a fraction
                     of the initial error;
* RMS error       -- root-mean-square of the tracking error;
* control effort  -- RMS of the control torque.
"""
import numpy as np


def rms(values):
    values = np.asarray(values, dtype=float)
    return float(np.sqrt(np.mean(values ** 2))) if len(values) else float("nan")


def settling_time(t, q, q_desired, tol=0.01, window=0.5):
    """First time (s) after which the error stays below ``tol`` for
    ``window`` seconds. Returns NaN if it never settles."""
    t = np.asarray(t, dtype=float)
    err = np.abs(np.asarray(q, dtype=float) - q_desired)
    if len(t) < 2:
        return float("nan")
    dt = t[1] - t[0]
    n_window = max(1, int(round(window / dt)))
    for i in range(len(t) - n_window):
        if np.all(err[i:i + n_window] < tol):
            return float(t[i])
    return float("nan")


def overshoot(q, q_desired, initial_error=None):
    """Overshoot = how far the response swings past the target after
    the first crossing, as a fraction of the initial error.

    A regulator that settles monotonically returns ~0; an
    underdamped one returns e.g. ~0.9 (90% past the target).
    """
    err = np.asarray(q, dtype=float) - q_desired
    if initial_error is None:
        initial_error = err[0] if len(err) else 0.0
    initial_error = abs(initial_error)
    if initial_error < 1e-12:
        return 0.0

    crossing = None
    for i in range(1, len(err)):
        if err[i - 1] * err[i] < 0:
            crossing = i
            break
    if crossing is None:                       # never crossed the target
        return 0.0
    peak_after = np.max(np.abs(err[crossing:]))
    return float(peak_after / initial_error)


def rms_error(t, q, q_desired):
    """RMS of the tracking error over the whole run."""
    err = np.asarray(q, dtype=float) - q_desired
    return rms(err)


def control_effort(u):
    """RMS of the control torque."""
    return rms(u)


def evaluate_run(t, q, u, q_desired, tol=0.01, window=0.5):
    """All metrics in one dict (used by the Milestone-5 sweep)."""
    return {
        "settling_time": settling_time(t, q, q_desired, tol, window),
        "overshoot": overshoot(q, q_desired),
        "rms_error": rms_error(t, q, q_desired),
        "control_effort": control_effort(u),
    }
