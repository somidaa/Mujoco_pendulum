"""Plotting helpers and the pendulum video renderer.

All figures are saved into ``outputs/plots/`` as PNG files (GitHub-
friendly); the PD video is rendered into ``outputs/videos/`` as MP4.

The video is a matplotlib stick-figure animation of the pendulum.
The geometry matches the MuJoCo model: pivot at (0, 1.5), rod length
1.0 m, bob at ``(-L sin q, 1.5 - L cos q)``.
"""
from pathlib import Path

import matplotlib
matplotlib.use("Agg")                      # headless-safe, save to file
import matplotlib.pyplot as plt
import numpy as np

PROJECT_ROOT = Path(__file__).resolve().parents[1]
PLOTS_DIR = PROJECT_ROOT / "outputs" / "plots"
VIDEOS_DIR = PROJECT_ROOT / "outputs" / "videos"

# Time-series colors
C_Q = "#1f77b4"
C_DQ = "#ff7f0e"
C_U = "#d62728"
C_E = "#2ca02c"


def _save(fig, name):
    PLOTS_DIR.mkdir(parents=True, exist_ok=True)
    path = PLOTS_DIR / name
    fig.savefig(path, dpi=150, bbox_inches="tight")
    plt.close(fig)
    print(f"    saved {path}")
    return path


# --------------------------------------------------------------------- #
# free dynamics (Milestone 2)
# --------------------------------------------------------------------- #
def plot_free_dynamics(t, q, dq, stem="free_dynamics"):
    """q(t) and dq(t) for the canonical free run (q0 = 45 deg)."""
    fig, axes = plt.subplots(2, 1, figsize=(9, 6), sharex=True)
    axes[0].plot(t, q, color=C_Q, lw=1.4)
    axes[0].set_ylabel("q (rad)")
    axes[0].set_title("Free dynamics (u = 0), q0 = 45 deg")
    axes[0].grid(alpha=0.3)
    axes[1].plot(t, dq, color=C_DQ, lw=1.4)
    axes[1].set_xlabel("time (s)")
    axes[1].set_ylabel("dq (rad/s)")
    axes[1].grid(alpha=0.3)
    _save(fig, f"{stem}.png")

    # individual q and dq figures, matching the plan's file names
    fig, ax = plt.subplots(figsize=(9, 3.5))
    ax.plot(t, q, color=C_Q, lw=1.4)
    ax.set_xlabel("time (s)")
    ax.set_ylabel("q (rad)")
    ax.grid(alpha=0.3)
    _save(fig, f"{stem}_q.png")

    fig, ax = plt.subplots(figsize=(9, 3.5))
    ax.plot(t, dq, color=C_DQ, lw=1.4)
    ax.set_xlabel("time (s)")
    ax.set_ylabel("dq (rad/s)")
    ax.grid(alpha=0.3)
    _save(fig, f"{stem}_dq.png")


def plot_initial_angle_comparison(series, stem="free_dynamics_initial_angle"):
    """Overlay q(t) for several initial angles (experiment 2.2)."""
    fig, ax = plt.subplots(figsize=(9, 4.5))
    for label, (t, q) in series.items():
        ax.plot(t, q, lw=1.3, label=f"q0 = {label}")
    ax.set_xlabel("time (s)")
    ax.set_ylabel("q (rad)")
    ax.set_title("Free dynamics: initial-angle comparison")
    ax.legend()
    ax.grid(alpha=0.3)
    _save(fig, f"{stem}.png")


# --------------------------------------------------------------------- #
# constant torque (Milestone 5, Experiment B)
# --------------------------------------------------------------------- #
def plot_constant_torque(series, stem="constant_torque"):
    """Overlay q(t) for several constant torque values; show that a
    constant torque shifts the equilibrium angle."""
    fig, ax = plt.subplots(figsize=(9, 4.5))
    for label, (t, q) in series.items():
        ax.plot(t, q, lw=1.3, label=f"u = {label} N*m")
    ax.set_xlabel("time (s)")
    ax.set_ylabel("q (rad)")
    ax.set_title("Constant torque: equilibrium shift")
    ax.legend()
    ax.grid(alpha=0.3)
    _save(fig, f"{stem}.png")

    # single canonical run with q and u panels
    fig, axes = plt.subplots(2, 1, figsize=(9, 6), sharex=True)
    axes[0].plot(t, q, color=C_Q, lw=1.4)
    axes[0].set_ylabel("q (rad)")
    axes[0].set_title("Constant torque (u = 1.0 N*m)")
    axes[0].grid(alpha=0.3)
    axes[1].plot(t, np.full_like(t, 1.0), color=C_U, lw=1.4)
    axes[1].set_xlabel("time (s)")
    axes[1].set_ylabel("u (N*m)")
    axes[1].grid(alpha=0.3)
    _save(fig, f"{stem}_single.png")


# --------------------------------------------------------------------- #
# PD control (Milestone 3 / Experiment C)
# --------------------------------------------------------------------- #
def plot_pd_response(t, q, u, q_desired=0.0, stem="pd_response",
                     title="PD control (Kp = 20, Kd = 6)"):
    fig, axes = plt.subplots(2, 1, figsize=(9, 6), sharex=True)
    axes[0].plot(t, q, color=C_Q, lw=1.4, label="q(t)")
    axes[0].axhline(q_desired, color="k", ls="--", lw=1, label="q_desired")
    axes[0].set_ylabel("q (rad)")
    axes[0].set_title(title)
    axes[0].legend()
    axes[0].grid(alpha=0.3)
    axes[1].plot(t, u, color=C_U, lw=1.4)
    axes[1].set_xlabel("time (s)")
    axes[1].set_ylabel("u (N*m)")
    axes[1].grid(alpha=0.3)
    _save(fig, f"{stem}.png")


def plot_pd_gain_comparison(series, gain_name, stem):
    """Overlay q(t) while sweeping one gain (Kp or Kd)."""
    fig, ax = plt.subplots(figsize=(9, 4.5))
    for label, (t, q) in series.items():
        ax.plot(t, q, lw=1.3, label=f"{gain_name} = {label}")
    ax.axhline(0.0, color="k", ls="--", lw=1)
    ax.set_xlabel("time (s)")
    ax.set_ylabel("q (rad)")
    ax.set_title(f"PD control: {gain_name} sweep (q0 = 45 deg)")
    ax.legend()
    ax.grid(alpha=0.3)
    _save(fig, f"{stem}.png")


def plot_pd_metrics_table(rows, stem="pd_metrics"):
    """Render the Kp x Kd sweep as a table figure (settling time)."""
    kps = sorted({r["kp"] for r in rows})
    kds = sorted({r["kd"] for r in rows})
    grid = np.full((len(kps), len(kds)), np.nan)
    for r in rows:
        grid[kps.index(r["kp"]), kds.index(r["kd"])] = r["settling_time"]

    fig, ax = plt.subplots(figsize=(6.5, 4.5))
    im = ax.imshow(grid, cmap="viridis", aspect="auto")
    ax.set_xticks(range(len(kds)), [f"{k:g}" for k in kds])
    ax.set_yticks(range(len(kps)), [f"{k:g}" for k in kps])
    ax.set_xlabel("Kd")
    ax.set_ylabel("Kp")
    ax.set_title("Settling time (s) vs (Kp, Kd); gray = never settled")
    for i in range(len(kps)):
        for j in range(len(kds)):
            v = grid[i, j]
            txt = f"{v:.2f}" if not np.isnan(v) else "-"
            ax.text(j, i, txt, ha="center", va="center",
                    color="white" if not np.isnan(v) and v > np.nanmedian(grid) else "black")
    fig.colorbar(im, ax=ax, label="settling time (s)")
    _save(fig, f"{stem}.png")


# --------------------------------------------------------------------- #
# energy (Milestone 4)
# --------------------------------------------------------------------- #
def plot_energy(t, kinetic, potential, total, e0, stem="energy"):
    """T, V (shifted), E (shifted) and the absolute drift |E - E0|."""
    fig, axes = plt.subplots(3, 1, figsize=(9, 8), sharex=True)
    axes[0].plot(t, kinetic, color="#9467bd", lw=1.2)
    axes[0].set_ylabel("kinetic (J)")
    axes[0].grid(alpha=0.3)
    axes[1].plot(t, potential - e0, color="#ff7f0e", lw=1.2)
    axes[1].set_ylabel("V - V(0) (J)")
    axes[1].grid(alpha=0.3)
    drift = np.abs(np.asarray(total) - e0)
    axes[2].plot(t, drift, color=C_E, lw=1.2)
    axes[2].set_ylabel("|E - E(0)| (J)")
    axes[2].set_xlabel("time (s)")
    axes[2].grid(alpha=0.3)
    axes[0].set_title(f"Energy (free run, dt = 0.002 s, RK4); "
                      f"max drift = {drift.max():.2e} J")
    _save(fig, f"{stem}.png")


def plot_energy_drift(series, stem="energy_drift"):
    """|E(t) - E(0)| for several integration timesteps."""
    fig, ax = plt.subplots(figsize=(9, 5))
    for dt, (t, drift) in series.items():
        ax.semilogy(t, drift, lw=1.4, label=f"dt = {dt} s")
    ax.set_xlabel("time (s)")
    ax.set_ylabel("|E(t) - E(0)| (J)")
    ax.set_title("Numerical energy drift vs timestep (RK4)")
    ax.legend()
    ax.grid(alpha=0.3, which="both")
    _save(fig, f"{stem}.png")


# --------------------------------------------------------------------- #
# video (Milestone 5)
# --------------------------------------------------------------------- #
def pendulum_pose(q, pivot=(0.0, 1.5), length=1.0):
    """Bob position (x, z) for angle q -- matches the MuJoCo model."""
    x = pivot[0] - length * np.sin(q)
    z = pivot[1] - length * np.cos(q)
    return x, z


def render_pendulum_video(t, q, out_path=None, length=1.0,
                          pivot=(0.0, 1.5), fps=30):
    """Render a stick-figure animation of the pendulum to an MP4 file.

    Uses the ffmpeg binary bundled with ``imageio-ffmpeg``, so no
    system ffmpeg install is needed.
    """
    import imageio_ffmpeg
    from matplotlib.animation import FFMpegWriter

    if out_path is None:
        out_path = VIDEOS_DIR / "pendulum.mp4"
    out_path = Path(out_path)
    out_path.parent.mkdir(parents=True, exist_ok=True)

    t = np.asarray(t, dtype=float)
    q = np.asarray(q, dtype=float)

    # subsample to fps frames per second of simulated time
    if len(t) > 1:
        step = max(1, int(round(1.0 / ((t[1] - t[0]) * fps))))
        idx = np.arange(0, len(t), step)
    else:
        idx = np.arange(len(t))

    fig, ax = plt.subplots(figsize=(5, 6))
    ax.set_xlim(pivot[0] - length - 0.25, pivot[0] + length + 0.25)
    ax.set_ylim(pivot[1] - length - 0.35, pivot[1] + 0.25)
    ax.set_aspect("equal")
    ax.set_title("PD control of the MuJoCo pendulum")
    ax.grid(alpha=0.3)
    (rod_line,) = ax.plot([], [], color=C_Q, lw=3, zorder=2)
    (bob_dot,) = ax.plot([], [], "o", color=C_U, ms=12, zorder=3)
    (pivot_dot,) = ax.plot([pivot[0]], [pivot[1]], "ko", ms=8, zorder=4)
    (trail,) = ax.plot([], [], color=C_Q, lw=1, alpha=0.4, zorder=1)
    time_text = ax.text(0.02, 0.95, "", transform=ax.transAxes)

    trail_len = 60
    trail_x, trail_z = [], []

    def update(i):
        qi = q[idx[i]]
        bx, bz = pendulum_pose(qi, pivot, length)
        rod_line.set_data([pivot[0], bx], [pivot[1], bz])
        bob_dot.set_data([bx], [bz])
        trail_x.append(bx)
        trail_z.append(bz)
        if len(trail_x) > trail_len:
            trail_x.pop(0)
            trail_z.pop(0)
        trail.set_data(trail_x, trail_z)
        time_text.set_text(f"t = {t[idx[i]]:.1f} s")
        return rod_line, bob_dot, trail, time_text

    plt.rcParams["animation.ffmpeg_path"] = imageio_ffmpeg.get_ffmpeg_exe()
    anim = matplotlib_animation(fig, update, frames=len(idx), blit=True)
    writer = FFMpegWriter(fps=fps, bitrate=1200)
    anim.save(out_path, writer=writer)
    plt.close(fig)
    print(f"    saved {out_path}")
    return out_path


def matplotlib_animation(fig, func, frames, blit):
    from matplotlib.animation import FuncAnimation
    return FuncAnimation(fig, func, frames=frames, interval=1000 // 30,
                         blit=blit)
