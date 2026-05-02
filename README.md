# AUR-Cascaded-Control

> A Cascaded Trajectory Planning and Dynamic Control Framework for Autonomous Underwater Robots (AUR) in Complex Marine Environments.
>
> 面向复杂海域环境（时变海流、密集静态雷区与多维动态船只）的自主水下机器人（AUR）级联轨迹规划与动力学控制框架。

![System](https://img.shields.io/badge/System-AUR_Cascaded_Control-blue)
![Python](https://img.shields.io/badge/Python-3.8+-green)
![Status](https://img.shields.io/badge/Status-Active_Research-orange)

## Project Overview

本项目针对自主水下机器人（AUR）在未知、复杂及高动态海洋环境下的导航与控制痛点，构建了一套全局-局部级联的“规划—制导—控制”闭环系统。系统通过解耦几何轨迹规划与底层非线性物理约束，彻底解决了传统局部势场法易陷入局部极小值（死锁）的问题，并在电机饱和极限下实现了极度平滑的抗流扰动控制。

## Core Features

* **Cascaded Architecture & AUR Dynamics (级联架构与系统建模)**
  上层通过拓扑路点（Waypoints）提供无碰撞全局引导，彻底规避局部极小值陷阱；底层实装 Fossen 3-DOF 欠驱动非线性动力学模型（包含附加质量与二次非线性阻尼），解决上层几何规划与底层物理执行之间的动力学约束失配问题。
  
* **Flexible Potential Field RHP (局部轨迹优化)**
  设计 Receding Horizon Planner (RHP) 在线规划模块。摒弃传统的硬性碰撞惩罚，创新引入分段多项式柔性排斥势场（Piecewise Polynomial Potential Field），保留平滑的脱困梯度。结合横向循迹误差（CTE）与航向约束，实现复杂动态避障与高精度车道保持的软切换。

* **NLESO & Jerk-Limited RG (非线性控制与边界治理)**
  控制层提出非线性扩张状态观测器（NLESO），将时变洋流与未建模非线性阻尼集总为“总扰动”进行实时观测与前馈补偿，大幅提升鲁棒性；首创带加加速度（Jerk）饱和约束的参考治理器（RG），结合电机剩余扭矩动态整形上层阶跃指令，彻底消除执行器饱和与高频震荡（Chattering）。

* **Roadmap: Distributed Sim2Real (迈向高保真物理仿真)**
  后续规划基于 Ubuntu 20.04 + ROS Noetic 中间件重构算法框架，将规划与控制解耦为分布式节点；引入 MuJoCo 210 构建高保真 AUR 刚体动力学与碰撞模型，为 RL（强化学习）训练与 Sim2Real（仿真到实体）部署奠定工程基础。

## Repository Structure

```text
AUR-Cascaded-Control/
├── README.md                # 项目核心文档
├── requirements.txt         # 依赖配置
├── config/
│   └── env_config.py        # 复杂海洋环境配置 (发卡弯、死亡峡谷等)
├── core/
│   ├── dynamics.py          # AUR 3-DOF Fossen 动力学模型
│   ├── controllers.py       # NLESO 观测器、Jerk 治理器(RG)、滤波器
│   └── planners.py          # 包含柔性势场与 CTE 约束的 RHP 规划器
├── utils/
│   └── visualization.py     # 高清多面板可视化渲染引擎
├── main_sim.py              # 主仿真执行程序
└── demo_scenarios.ipynb     # 供快速演示与交互分析的 Jupyter Notebook
```

## Quick Start
1. Clone the repository
https://github.com/WuyangChen422/AUR-Cascaded-Control.git

2. Install dependencies
pip install -r requirements.txt

3. Run core simulation
python main_sim.py

## Extreme Scenario Showcases
系统目前已通过多项极度压榨系统极限的标准测试,包括但不限于：

The Slalom Intercept (连续发卡弯与斜向截击) [Current Default]
巨大的 W 型折返航线，测试大角度转弯（Heading Error）与斜向动态预测。AUR 需连续执行大角度折返，同时规避精确撞向“弯心（Apex）”的动态截击船只。

The Narrow Canyon (死亡峡谷)
在受限墙壁组成的狭窄直线走廊中，测试 CTE 横向约束与极高频侧向穿插船只的极限拉扯。

(注：可在 config/env_config.py 中切换对应场景的参数配置。)

## Data Visualization Interpretation
仿真结束后，系统将自动输出 6 块面板的综合分析图：

Top Panels (Global View): 最终的 AUR 全局航行轨迹以及在危险接近时刻的定格快照。

Bottom Left (ESO Synergy): 展示 NLESO 如何精确追踪未知海洋流场并实现完美前馈。

Bottom Middle (Yaw Torque): 证明在极端的动态避障下，电机的底层物理输出依然极其平滑，峰值严格控制在物理阈值内。

Bottom Right (Governor Shaping): RG (参考治理器) 在发生激烈避障时如何对角速度指令进行动态限幅与削顶。

## Acknowledgments
Instructed by prof.Wang Jun, Chair Professor of Computational Intelligence in the Department of Computer Science and School of Data Science at City University of Hong Kong.