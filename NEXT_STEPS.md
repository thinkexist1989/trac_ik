# 下一步：适配 Pinocchio 4.x 连续关节

## 当前状态

✅ **代码已提交到 git**: commit `e6962ea`  
✅ **核心库编译成功**: `libtrac_ik.so` 可用  
⚠️ **测试需要调整**: 处理 Pinocchio 4.x 的连续关节表示

## 问题说明

### Pinocchio 4.x 的变化

在 Pinocchio 4.x 中，**连续关节**（continuous joint）的表示方式改变了：

- **Pinocchio 2.x/3.x**: 使用 1 个角度变量 θ
- **Pinocchio 4.x**: 使用 2 个变量 (cos θ, sin θ)

这意味着：
```
URDF 中的关节：6 个（3 prismatic + 2 revolute + 1 continuous）
配置空间维度：7 个（3 + 2 + 2）
```

### 影响范围

需要更新以下方面以正确处理连续关节：

1. **配置向量维度** - 需要知道哪些关节是连续的
2. **雅可比矩阵** - 连续关节的雅可比有 2 列而不是 1 列
3. **关节限制** - 连续关节的限制在单位圆上 [-1.01, 1.01]
4. **角度转换** - 需要在 θ 和 (cos θ, sin θ) 之间转换

## 实现方案

### 方案 1：完整支持（推荐）

在 `pinocchio_tl.cpp` 和 `nlopt_ik.cpp` 中添加连续关节处理：

```cpp
// 1. 在 loadURDFModel 中识别连续关节
std::vector<bool> is_continuous(chain_dof, false);
for (JointIndex jid : joint_ids) {
  if (model.joints[jid].shortname() == "JointModelRUB") { // Revolute Unbounded
    // 标记这是连续关节
    is_continuous[dof_index] = true;
    is_continuous[dof_index + 1] = true;
  }
}

// 2. 在 IK 求解器中处理配置向量映射
Eigen::VectorXd angleToConfig(double theta, int joint_type) {
  if (joint_type == Continuous) {
    Eigen::Vector2d config;
    config << std::cos(theta), std::sin(theta);
    return config;
  }
  return Eigen::VectorXd::Constant(1, theta);
}

double configToAngle(const Eigen::VectorXd& config, int joint_type) {
  if (joint_type == Continuous) {
    return std::atan2(config(1), config(0));
  }
  return config(0);
}

// 3. 在优化过程中使用角度空间
// - 优化变量在角度空间（6-DOF）
// - 前向运动学使用配置空间（7-DOF）
// - 需要来回转换
```

### 方案 2：使用角度表示（简单）

修改 URDF 解析，将连续关节替换为有限旋转关节：

```cpp
// 在 buildModelFromXML 之后
for (JointIndex i = 1; i < model.njoints; ++i) {
  if (model.joints[i].shortname() == "JointModelRUB") {
    // 用大范围的旋转关节替换
    // 这需要重建模型，较复杂
  }
}
```

### 方案 3：子配置向量（中等复杂度）

只使用模型中链的部分：

```cpp
// 在前向运动学和雅可比计算中
// 使用 Pinocchio 的子配置向量功能
Eigen::VectorXd full_config = pinocchio::neutral(model);
// 只更新链相关的关节
for (size_t i = 0; i < joint_ids.size(); ++i) {
  int idx_q = model.joints[joint_ids[i]].idx_q();
  full_config.segment(idx_q, ...) = chain_config.segment(...);
}
```

## 需要修改的文件

1. **trac_ik_lib/src/urdf.cpp**
   - 添加连续关节检测
   - 返回关节类型信息

2. **trac_ik_lib/src/pinocchio_tl.cpp**
   - 添加配置空间转换
   - 处理连续关节的雅可比

3. **trac_ik_lib/src/nlopt_ik.cpp**
   - 更新优化变量映射
   - 处理连续关节的限制

4. **tests/standalone_tests.cpp**
   - 更新测试用例的配置向量
   - 调整期望值

## 测试策略

### 阶段 1：单元测试
```bash
# 创建简单的测试用例
# - 2-DOF 机械臂（无连续关节）
# - 验证基本 IK 功能
```

### 阶段 2：连续关节测试
```bash
# 使用测试 URDF（包含连续关节）
# - 验证配置空间转换
# - 验证 IK 求解正确性
```

### 阶段 3：完整测试
```bash
# 运行所有测试用例
./build/tests/standalone_tests tests/robot.urdf
```

## 估计工作量

- **方案 1（完整支持）**: 4-6 小时
  - 配置转换: 1 小时
  - 雅可比处理: 1 小时
  - IK 求解器更新: 2 小时
  - 测试和调试: 2 小时

- **方案 3（子配置向量）**: 2-3 小时
  - 实现相对简单
  - 测试和调试: 1 小时

## 快速开始

### 选择方案 3（推荐）

这是最快的路径，保持现有架构：

```bash
# 1. 编辑 trac_ik_lib/src/urdf.cpp
# 2. 修改 loadURDFModel 使用子配置向量
# 3. 在 pinocchio_tl.cpp 中映射配置
# 4. 重新编译测试
cd build && make -j$(nproc)
./tests/standalone_tests ../tests/robot.urdf
```

## 参考资源

- **Pinocchio 文档**: https://stack-of-tasks.github.io/pinocchio/
- **连续关节 API**: `JointModelRevoluteUnbounded`
- **配置空间**: `pinocchio::neutral()`, `pinocchio::integrate()`

## 联系点

- 当前代码: `commit e6962ea`
- 分支: `pinocchio`
- 文档: `FINAL_STATUS.md`, `MIGRATION_REPORT.md`

---

**开始命令**:
```bash
cd /home/think/Documents/GitHub/trac_ik
git checkout pinocchio
# 选择一个方案开始实现
```
