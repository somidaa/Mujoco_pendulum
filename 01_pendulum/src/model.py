"""MJCF model loading helpers (Milestone 1).

Responsibilities
----------------
* load an MJCF XML file into a :class:`mujoco.MjModel`
* create an :class:`mujoco.MjData` state object for a model
* print a compact summary of the model's structure

Concepts
--------
``MjModel``  -- the robot's *structure*: bodies, joints, geoms, masses,
inertias, actuators, solver options ...
``MjData``   -- the robot's *state* at the current instant: qpos, qvel,
ctrl, time, contacts, energy ...
"""
import os
from pathlib import Path

import mujoco

# Path of the pendulum MJCF file, resolved relative to this file
# so that the code works from any working directory.
MODEL_PATH = Path(__file__).resolve().parents[1] / "model" / "pendulum.xml"


def load_model(xml_path: "os.PathLike | str" = MODEL_PATH) -> mujoco.MjModel:
    """Load an MJCF XML file and return the compiled MuJoCo model."""
    return mujoco.MjModel.from_xml_path(str(xml_path))


def create_data(model: mujoco.MjModel) -> mujoco.MjData:
    """Create the simulation state (MjData) for a given model."""
    return mujoco.MjData(model)


def print_model_info(model: mujoco.MjModel) -> None:
    """Print the quantities every MuJoCo beginner has to know."""
    print("=" * 52)
    print("MuJoCo model summary")
    print("=" * 52)
    print(f"  nq        = {model.nq}   # generalized positions  (qpos = [q])")
    print(f"  nv        = {model.nv}   # generalized velocities (qvel = [dq])")
    print(f"  nu        = {model.nu}   # control inputs         (ctrl = [u])")
    print(f"  nbody     = {model.nbody}   # bodies")
    print(f"  ngeom     = {model.ngeom}   # geometries")
    print(f"  njnt      = {model.njnt}   # joints")
    print(f"  nactuator = {model.nu}   # actuators")
    print(f"  timestep  = {model.opt.timestep}")
    print(f"  integrator= {model.opt.integrator}")
    print(f"  gravity   = {model.opt.gravity.tolist()}")
    print(f"  qpos0     = {model.qpos0.tolist()}  # default joint configuration")
    print("=" * 52)
