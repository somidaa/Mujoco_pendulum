# Project 1 · MuJoCo Pendulum Dynamics & Control Lab

单摆动力学与控制实验：MJCF 建模 → MuJoCo 仿真 → 自由摆动 → PD 控制 → 能量分析。

## 目录结构

```
01_pendulum/
├── model/pendulum.xml      # MJCF 模型（0.3 kg 细杆，悬挂点 z=1.5）
├── src/
│   ├── model.py            # 加载 MjModel / MjData
│   ├── simulation.py       # run_simulation() 统一仿真循环
│   ├── controller.py       # pd_control()
│   ├── logger.py           # Logger 数据记录 + CSV
│   ├── energy.py           # get_energy()
│   ├── metrics.py          # settling time / overshoot / RMS / effort
│   └── visualization.py    # 绘图
├── experiments/
│   ├── 01_free_dynamics.py   # 自由动力学（viewer 实时显示）
│   ├── 02_constant_torque.py # 恒力矩
│   ├── 03_pd_control.py      # PD 控制（viewer 实时显示）
│   ├── 04_energy_analysis.py # 能量分析（4.1 能量记录 + 4.2 timestep 漂移）
│   ├── 05_parameter_sweep.py # Kp×Kd 16 组合扫描 + 指标表 + 对比图
│   └── 06_period_vs_angle.py # 15/30/45/60° 初始角周期对比
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
py experiments/04_energy_analysis.py  # 能量分析 + timestep 漂移
py experiments/05_parameter_sweep.py  # Kp×Kd 参数扫描
py experiments/06_period_vs_angle.py  # 初始角周期对比
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

## 实验说明

- **01 自由动力学**：45° 释放无阻尼摆动，输出 `free_dynamics.csv` + 曲线
- **02 恒力矩**：u=1.0 N·m，摆停在平衡角 q\*=asin(u/mgLc)
- **03 PD 控制**：Kp=10, Kd=2，调节到 0 rad，输出 `pd_control.csv`
- **04 能量分析**：记录 T/V/E 验证守恒；改变 timestep 观察数值漂移（dt 越大漂移越大）
- **05 参数扫描**：Kp∈[1,5,10,20] × Kd∈[0.1,0.5,1,2]，输出 16 个 CSV + `pd_metrics.csv` + Kp/Kd 对比图
  - Kd 太小 → 欠阻尼振荡、难收敛（超调 ~90%）
  - Kd 合适 → 快且稳（如 Kp=20/Kd=2：0.44 s 收敛，超调 5.8%）
  - Kd 太大 → 过阻尼变慢（Kp=1/Kd=2：3.38 s）
- **06 初始角周期对比**：15/30/45/60° 自由摆动，输出 4 个 CSV + 对比图
  - 周期随摆角增大而变长（1.675 s → 1.790 s），非线性摆特性
