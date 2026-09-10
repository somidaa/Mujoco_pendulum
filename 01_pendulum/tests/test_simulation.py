"""Milestone 2 + 4 tests: simulation stepping, logging and energy."""
import numpy as np
import mujoco

from src.model import load_model, create_data
from src.simulation import run_simulation
from src.energy import compute_energies, energy_drift


def _reset(model, data, q0=np.pi / 4):
    mujoco.mj_resetData(model, data)
    data.qpos[0] = q0
    data.qvel[0] = 0.0
    data.ctrl[0] = 0.0
    mujoco.mj_forward(model, data)


def test_time_advances():
    model = load_model()
    data = create_data(model)
    _reset(model, data)
    logger = run_simulation(model, data, duration=1.0)
    assert abs(data.time - 1.0) < model.opt.timestep
    assert abs(logger.time[-1] - 1.0) < model.opt.timestep


def test_states_remain_finite():
    model = load_model()
    data = create_data(model)
    _reset(model, data, q0=2.5)
    logger = run_simulation(model, data, duration=5.0)
    d = logger.to_dict()
    assert np.all(np.isfinite(d["q"]))
    assert np.all(np.isfinite(d["dq"]))
    assert np.all(np.isfinite(d["u"]))


def test_free_dynamics_conserves_energy():
    model = load_model()
    data = create_data(model)
    _reset(model, data)
    logger = run_simulation(model, data, duration=5.0, record_energy=True)
    d = logger.to_dict()
    abs_drift, _ = energy_drift(d["total_energy"])
    # RK4 with dt = 0.002 s keeps the drift tiny for a 5 s free run
    assert abs_drift.max() < 1e-2


def test_our_energy_matches_engine_energy():
    model = load_model()
    data = create_data(model)
    _reset(model, data)
    for _ in range(200):
        mujoco.mj_step(model, data)
        kinetic, potential, total = compute_energies(model, data)
        engine = np.asarray(data.energy)          # [potential, kinetic]
        assert abs(kinetic - engine[1]) < 1e-7
        assert abs(potential - engine[0]) < 1e-7
        assert abs(total - (engine[0] + engine[1])) < 1e-7


def test_constant_torque_shifts_equilibrium():
    model = load_model()
    data = create_data(model)
    _reset(model, data, q0=0.0)
    logger = run_simulation(model, data, duration=10.0,
                            controller=lambda q, dq, t: 1.0)
    d = logger.to_dict()
    # a constant positive torque pushes the pendulum to q* > 0
    assert float(np.mean(d["q"][-1000:])) > 0.05


def test_logger_round_trip(tmp_path):
    from src.logger import DataLogger
    model = load_model()
    data = create_data(model)
    _reset(model, data)
    logger = run_simulation(model, data, duration=0.5)
    path = tmp_path / "run.csv"
    logger.save_csv(path)
    loaded = DataLogger.load_csv(path)
    assert len(loaded) == len(logger)
    assert np.allclose(loaded.q, logger.q, atol=1e-12)
