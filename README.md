# Mujoco Pendulum

MuJoCo 学习路线 Project 1：**单摆动力学与控制实验**（Dynamics & Control Lab）。

从 Modern Robotics / youBot 项目平滑过渡到 MuJoCo：自己定义 MJCF 模型、运行动力学仿真、读取物理状态、实现 PD 控制器、记录数据、设计实验、分析结果 —— 不依赖任何 RL 框架。

## 目录结构

```
mujoco_learning/
└── 01_pendulum/          # Project 1（本仓库当前内容）
    ├── model/            # pendulum.xml —— MJCF 模型
    ├── src/              # model / simulation / controller / logger / energy / metrics / visualization
    ├── experiments/      # 00_viewer · 01_free_dynamics · 02_constant_torque · 03_pd_control · 04_energy_analysis
    ├── tests/            # pytest（20 用例）
    └── outputs/          # csv / plots / videos
```

## 快速开始

```powershell
cd 01_pendulum
py -m pip install -r requirements.txt
py experiments/00_viewer.py        # 打开 viewer 看单摆
py experiments/01_free_dynamics.py # 自由动力学
py experiments/03_pd_control.py    # PD 控制 + 参数扫描 + 动画视频
py -m pytest tests -v              # 跑测试
```

## 结果预览

| 实验 | 一句话结论 |
|---|---|
| 自由动力学 | 无阻尼振荡，周期随初始摆角增大（15°→1.96 s，60°→2.10 s） |
| 恒力矩 | 平衡角 q\* = asin(u/mgLc)，实测与解析预测吻合 |
| PD 控制 | Kp 增大 → 更快但更振荡；Kd 增大 → 阻尼增强；Kp=20/Kd=6 时 1.06 s 收敛、超调 7.8% |
| 能量分析 | RK4 下 dt=0.002 的 20 s 能量漂移仅 ~4.6e-8 J，dt 越大漂移越大 |

## 路线图

- [x] **01_pendulum**（本仓库）：MJCF 建模、自由动力学、PD 控制、能量分析、参数扫描
- [ ] 02_cartpole：状态空间与控制
- [ ] 03_panda：运动学 + 动力学 + 操作空间控制
- [ ] MuJoCo + Gymnasium + RL（TD / Q-learning / Policy Gradient / PPO）

详细文档见 [`01_pendulum/README.md`](01_pendulum/README.md)。
