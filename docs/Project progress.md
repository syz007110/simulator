# Project Progress

## 当前阶段：Phase 1 — 数学基础层

---

## Phase 1 — 数学基础层
**目标：** 建立机器人学数学工具箱
**学习对应：** 《Modern Robotics》Ch1-2，李群/李代数基础

- [ ] SO3：旋转矩阵、指数映射/对数映射
- [ ] SE3：齐次变换矩阵 T = [R|p]
- [ ] Quaternion：四元数表示旋转，插值(SLERP)
- [ ] Twist / Wrench：旋量（空间速度/力）
- [ ] 完整单元测试覆盖

---

## Phase 2 — 机器人模型层
**目标：** 加载并表示一个机器人
**学习对应：** DH参数 vs 指数积(PoE)，URDF格式

- [ ] 刚体（质量、质心、惯量张量）
- [ ] 关节类型（revolute / prismatic / fixed）
- [ ] 运动链（KinematicChain）
- [ ] URDF解析器

---

## Phase 3 — 运动学层
**目标：** FK + IK + Jacobian
**学习对应：** 《Modern Robotics》Ch4-6

- [ ] 基于PoE的正运动学
- [ ] 几何雅可比 & 解析雅可比
- [ ] 数值IK（阻尼最小二乘 DLS）
- [ ] 解析IK（特定构型，如6R腕部偏置臂）

---

## Phase 4 — 动力学层
**目标：** 计算关节力矩和加速度
**学习对应：** 《Modern Robotics》Ch8

- [ ] RNEA：递归牛顿欧拉（逆动力学 τ = ID(q, q̇, q̈)）
- [ ] CRBA：复合刚体算法（惯量矩阵 M(q)）
- [ ] 重力补偿 g(q)
- [ ] 正动力学：q̈ = M⁻¹(τ - C·q̇ - g)

---

## Phase 5 — 运动控制层
**目标：** 机器人按轨迹运动
**学习对应：** 《Robot Modeling and Control》Ch8-9

- [ ] 轨迹生成：五次多项式（满足速度/加速度约束）
- [ ] 轨迹生成：三次样条、梯形速度(LSPB)
- [ ] 关节空间 PD 控制
- [ ] 计算力矩控制（前馈动力学补偿）
- [ ] 任务空间操作空间控制

---

## Phase 6 — 仿真集成
**目标：** 完整仿真机器人跑轨迹

- [ ] RK4 数值积分
- [ ] 仿真主循环（固定步长）
- [ ] MeshCat 3D 可视化
- [ ] 里程碑：加载 UR5 URDF，控制器跟踪轨迹，实时可视化

---

## Phase 7 — 外部接口（按需）

- [ ] ROS2 Bridge（robot_state_publisher 兼容，TF2发布）
- [ ] Isaac Sim 接口（关节状态读写）
