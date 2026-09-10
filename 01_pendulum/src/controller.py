
def pd_control(q, qdot, q_desired, kp, kd):
    # u = Kp*(qd - q) - Kd*dq
    error = q_desired - q

    u = kp * error - kd * qdot

    return u
