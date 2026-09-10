"""Milestone 1 acceptance tests: model loading and MjModel / MjData."""
import numpy as np

from src.model import load_model, create_data, MODEL_PATH


def test_model_loads_from_xml():
    model = load_model(MODEL_PATH)
    assert model is not None


def test_dimensions_of_single_pendulum():
    model = load_model(MODEL_PATH)
    # one hinge joint -> 1 position, 1 velocity, 1 control input
    assert model.nq == 1
    assert model.nv == 1
    assert model.nu == 1


def test_data_creation_and_shapes():
    model = load_model(MODEL_PATH)
    data = create_data(model)
    assert data.qpos.shape == (model.nq,)
    assert data.qvel.shape == (model.nv,)
    assert data.ctrl.shape == (model.nu,)
    assert data.time == 0.0


def test_default_pose_is_hanging_down():
    model = load_model(MODEL_PATH)
    # q = 0 -> rod points straight down
    assert abs(model.qpos0[0]) < 1e-9


def test_initial_state_can_be_set():
    model = load_model(MODEL_PATH)
    data = create_data(model)
    data.qpos[0] = np.pi / 4
    data.qvel[0] = 0.5
    assert abs(data.qpos[0] - np.pi / 4) < 1e-12
    assert abs(data.qvel[0] - 0.5) < 1e-12


def test_gravity_is_earth_like():
    model = load_model(MODEL_PATH)
    assert abs(model.opt.gravity[2] + 9.81) < 1e-6
