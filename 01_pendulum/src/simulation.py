import mujoco


def run_simulation(model, data, duration, controller=None, logger=None):
    """推进 duration 秒仿真；controller(q, dq, t) -> u，None 表示自由运动。"""
    n_steps = int(duration / model.opt.timestep)

    for _ in range(n_steps):
        q = data.qpos[0]
        dq = data.qvel[0]

        if controller is None:
            u = 0.0
        else:
            u = controller(q, dq, data.time)

        data.ctrl[0] = u
        mujoco.mj_step(model, data)

        if logger is not None:
            logger.record(data)

    return logger
