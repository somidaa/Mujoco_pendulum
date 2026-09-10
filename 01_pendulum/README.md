# Project 1 · MuJoCo Pendulum Dynamics & Control Lab

单摆动力学与控制实验：MJCF 建模 → MuJoCo 仿真 → 自由摆动 → PD 控制 → 能量分析。

## 目录结构

```
01_pendulum/
├── model/pendulum.xml      # MJCF 模型
├── src/
│   ├── model.py            # 加载 MjModel / MjData
│   ├── simulation.py       # run_simulation() 统一仿真循环
│   ├── controller.py       # pd_control()
│   ├── logger.py           # Logger 数据记录 + CSV
│   ├── energy.py           # get_energy()
│   └── visualization.py    # 绘图
├── experiments/
│   ├── 01_free_dynamics.py   # 自由动力学（viewer 实时显示）
│   ├── 02_constant_torque.py # 恒力矩
│   ├── 03_pd_control.py      # PD 控制（viewer 实时显示）
│   └── 04_energy_analysis.py # 能量分析
├── tests/                  # 模型 / 仿真 / 控制器验证
├── outputs/                # csv / plots
└── requirements.txt
```

## 快速开始

```powershell
pip install -r requirements.txt
py experiments/01_free_dynamics.py    # 自由动力学
py experiments/02_constant_torque.py  # 恒力矩
py experiments/03_pd_control.py       # PD 控制
py experiments/04_energy_analysis.py  # 能量分析
py -m pytest tests
```

## 核心概念

| 对象 | 含义 |
|---|---|
| `MjModel` | 模型结构（body / joint / geom / actuator） |
| `MjData` | 当前状态（qpos / qvel / ctrl / time） |
| `mj_step()` | 推进一步物理仿真 |

## 关键点

- 控制律：`u = Kp*(qd - q) - Kd*dq`
- 能量：`data.energy = [potential, kinetic]`（XML 中开启了 energy flag）
- 脚本通过 `__file__` 定位项目根目录，任意目录可运行
