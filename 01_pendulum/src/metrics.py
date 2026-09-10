import numpy as np


def rms(values):
    # 均方根
    return float(np.sqrt(np.mean(np.asarray(values) ** 2)))


def settling_time(t, q, q_desired, tol=0.01, window=0.5):
    # 误差进入 tol 并保持 window 秒的最早时刻，未收敛返回 nan
    err = np.abs(np.asarray(q) - q_desired)
    dt = t[1] - t[0]
    n = int(window / dt)
    for i in range(len(t) - n):
        if np.all(err[i:i + n] < tol):
            return t[i]
    return float("nan")


def overshoot(q, q_desired):
    # 首次穿越目标后超出目标的最大值 / 初始误差
    err = np.asarray(q) - q_desired
    e0 = abs(err[0])
    if e0 < 1e-12:
        return 0.0
    for i in range(1, len(err)):
        if err[i - 1] * err[i] < 0:
            return float(np.max(np.abs(err[i:])) / e0)
    return 0.0


def control_effort(u):
    # 控制量 RMS
    return rms(u)
