# SimRobot — 自建机器人仿真库

## 项目目标

完全自建一个机器人库，在构建过程中系统学习机器人学知识。

**核心功能范围：**
- 运动学：正运动学(FK)、逆运动学(IK)、雅可比矩阵
- 动力学：逆动力学(RNEA)、正动力学(CRBA)
- 运动控制：轨迹规划、关节空间控制、任务空间控制
- 整机仿真：数值积分驱动的仿真主循环 + 3D可视化

**未来扩展：** ROS2 bridge、Isaac Sim 接口

---

## 项目结构

```
Simulator/
├── CLAUDE.md
├── CMakeLists.txt
├── vcpkg.json
├── src/
│   ├── core/math/          # SO3, SE3, Quaternion, Twist
│   ├── model/              # RigidBody, Joint, Link, KinematicChain, URDFParser
│   ├── kinematics/         # FK, IK, Jacobian
│   ├── dynamics/           # RNEA, CRBA, ABA
│   ├── control/            # 轨迹生成, PD控制, 计算力矩
│   ├── simulation/         # 仿真主循环, 数值积分
│   └── interface/          # ROS2/Isaac预留接口
├── python/                 # pybind11 绑定
├── tests/
├── examples/
└── docs/
```

---

## 构建命令

```bash
cmake -B build -DCMAKE_BUILD_TYPE=Debug
cmake --build build -j8
ctest --test-dir build --output-on-failure
```

---

## 依赖原则

**允许使用（底层计算库）：**
- Eigen3 — 线性代数
- GoogleTest — 单元测试
- pybind11 — Python 绑定
- tinyxml2 — URDF 文件解析
- MeshCat — 3D 可视化

**禁止使用（机器人相关的开源库）：**
- 不使用 Pinocchio、KDL、RBDL、MoveIt、drake 等机器人库
- 运动学、动力学、控制、仿真逻辑**全部自建**
- Pinocchio/Drake 等只作为算法验证的**对照参考**，不作为依赖引入

---

## 设计原则

- 物理量有明确语义（单位写在注释：rad, rad/s, N·m）
- 接口与实现分离（`IDynamics`, `IController` 等纯虚接口）
- 不可变模型 + 可变状态：`const RobotModel` 只读，`RobotState` 运行时更新
- 优先用指数积(PoE)而非DH参数
- 每个模块完成后用 `/simplify` 检查代码质量
- 测试驱动：先写测试用例，再实现

---

## 技术栈

C++20 · Eigen3 3.4+ · Python 3.11+ · pybind11 · MeshCat · CMake 3.24+ · GoogleTest · vcpkg · tinyxml2

## 参考资料

- 《Modern Robotics》Lynch & Park — 主教材（运动学/动力学）
- 《Robotics: Modelling, Planning and Control》Siciliano — 控制部分
- Pinocchio / Drake — 仅作算法验证对照，不引入为依赖

---

## 开发进度

@docs/Project progress.md
