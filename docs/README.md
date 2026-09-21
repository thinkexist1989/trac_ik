# TRAC-IK 文档

本目录包含 TRAC-IK 从 KDL 到 Pinocchio 4.x 迁移的详细技术文档。

## 文档列表

### 迁移相关
- **[MIGRATION_STATUS.md](MIGRATION_STATUS.md)** - 迁移进度跟踪
- **[MIGRATION_REPORT.md](MIGRATION_REPORT.md)** - 详细技术迁移报告
- **[FINAL_STATUS.md](FINAL_STATUS.md)** - 完整项目状态总结

### Pinocchio 4.x 适配
- **[NEXT_STEPS.md](NEXT_STEPS.md)** - Pinocchio 4.x 连续关节适配指南
- **[JACOBIAN_DETAILS.md](JACOBIAN_DETAILS.md)** - 雅可比矩阵和连续关节详细技术解释
- **[README_PINOCCHIO.md](README_PINOCCHIO.md)** - Pinocchio 使用指南

## 快速导航

### 如果你想...

#### 了解迁移完成度
→ 阅读 [FINAL_STATUS.md](FINAL_STATUS.md)

**核心结论**: 迁移 95% 完成，核心库完全可用

#### 理解连续关节问题
→ 阅读 [JACOBIAN_DETAILS.md](JACOBIAN_DETAILS.md)

**关键点**: 
- 配置空间 (nq=7) vs 速度空间 (nv=6)
- 雅可比矩阵维度不变 (6×6)
- 必须使用 `pinocchio::integrate()` 更新配置

#### 继续适配工作
→ 阅读 [NEXT_STEPS.md](NEXT_STEPS.md)

**推荐方案**: 使用 `pinocchio::integrate()` (已完成)  
**估计时间**: 30-60 分钟（实际只用了 20 分钟）

#### 学习 Pinocchio API
→ 阅读 [README_PINOCCHIO.md](README_PINOCCHIO.md)

**涵盖内容**:
- 基本类型和概念
- 前向运动学
- 雅可比矩阵计算
- URDF 加载

#### 查看详细迁移过程
→ 阅读 [MIGRATION_REPORT.md](MIGRATION_REPORT.md)

**包含内容**:
- API 映射表 (KDL → Pinocchio)
- 修改的文件列表
- 编译问题和解决方案

## 技术要点

### Pinocchio 4.x 关键差异

#### 1. 连续关节表示
```
Pinocchio 2.x/3.x: θ (1 个变量)
Pinocchio 4.x:     (cos θ, sin θ) (2 个变量)
```

#### 2. 配置 vs 速度空间
```
配置空间 (q): nq=7  [x, y, z, roll, pitch, cos(yaw), sin(yaw)]
速度空间 (v): nv=6  [ẋ, ẏ, ż, roll̇, pitcḣ, yaẇ]
雅可比 (J):   6×6   定义在速度空间
```

#### 3. 配置更新
```cpp
// ❌ 错误
q = q + delta_q;

// ✅ 正确
pinocchio::integrate(model, q, delta_q, q);
```

## 项目状态

| 模块 | 状态 | 说明 |
|------|------|------|
| 核心库编译 | ✅ 完成 | libtrac_ik.so 成功构建 |
| API 迁移 | ✅ 完成 | 所有 KDL 代码已移除 |
| pinocchio::integrate() | ✅ 完成 | 配置更新正确实现 |
| 测试套件 | ⚠️ 需调整 | FK 期望值基于简化链，实际用完整模型 |
| Python 绑定 | ✅ 可用 | SWIG 绑定编译成功 |

**总体进度: 95%**

## Git 历史

重要的 commits:
- `e6962ea` - KDL 到 Pinocchio 迁移核心完成
- `59fa577` - 添加雅可比矩阵详细解释
- `bb77162` - 工作量重新评估（4-6h → 30-60min）
- `d53dbfe` - 实现 pinocchio::integrate()

## 相关资源

### 外部链接
- [Pinocchio 文档](https://stack-of-tasks.github.io/pinocchio/)
- [Pinocchio GitHub](https://github.com/stack-of-tasks/pinocchio)
- [NLopt 文档](https://nlopt.readthedocs.io/)
- [原始 TRAC-IK 论文](https://ieeexplore.ieee.org/document/7363472)

### 仓库文件
- [主 README](../README.md) - 使用说明和 API
- [CLAUDE.md](../CLAUDE.md) - Claude Code 开发指南
- [LICENSE.txt](../LICENSE.txt) - BSD 3-Clause 许可证

## 贡献

如果发现文档错误或有改进建议：
1. 创建 Issue 描述问题
2. 或直接提交 Pull Request

## 更新日志

- 2024-09-21: 初始文档创建，迁移核心完成
- 2024-09-21: 实现 pinocchio::integrate() 支持
- 2024-09-21: 文档整理到 docs/ 目录
