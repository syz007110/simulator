# Reference

## 技术栈版本

| 层级 | 技术 | 版本要求 |
|------|------|---------|
| 核心语言 | C++ | C++20 |
| 脚本/调试 | Python | 3.11+ |
| 线性代数 | Eigen3 | 3.4+ |
| Python绑定 | pybind11 | 2.11+ |
| 可视化 | MeshCat | latest |
| 构建系统 | CMake | 3.24+ |
| 单元测试 | GoogleTest | 1.14+ |
| 依赖管理 | vcpkg | latest |
| URDF解析 | tinyxml2 | 9.0+ |

---

## 学习教材

| 资料 | 覆盖模块 | 说明 |
|------|---------|------|
| 《Modern Robotics》Lynch & Park | 运动学、动力学全覆盖 | 主教材，有配套Python代码 |
| 《Robotics: Modelling, Planning and Control》Siciliano | 控制部分更深入 | 控制阶段参考 |
| 《A Mathematical Introduction to Robotic Manipulation》Murray | 数学基础、PoE | Phase 1-2 参考 |

---

## 参考实现

| 项目 | 用途 | 说明 |
|------|------|------|
| [Pinocchio](https://github.com/stack-of-tasks/pinocchio) | 动力学实现参考 | C++，RNEA/CRBA标准实现 |
| [Drake](https://github.com/RobotLocomotion/drake) | 工程架构参考 | 大型C++机器人库，学习接口设计 |
| [Modern Robotics Code](https://github.com/NxRLab/ModernRobotics) | 算法参考 | 教材配套Python实现 |
| [MuJoCo](https://mujoco.org/) | 仿真物理参考 | 接触力、柔性体 |

---

## 常用资源

- URDF 示例：[ros/urdf_tutorial](https://github.com/ros/urdf_tutorial)
- UR5 模型：[fmauch/universal_robot](https://github.com/fmauch/universal_robot)
- Franka 模型：[frankaemika/franka_ros](https://github.com/frankaemika/franka_ros)
