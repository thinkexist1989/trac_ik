# TRAC-IK standalone

独立的 C++17 / Python 3 逆运动学求解器，不需要 ROS、ROS master、catkin、ament、MoveIt 或参数服务器。
保留 KDL + NLopt 双线程求解及 Speed、Distance、Manip1、Manip2、Manip3 模式。

## 依赖与编译

Ubuntu / Debian：

```sh
sudo apt-get install build-essential cmake pkg-config libeigen3-dev \
  liborocos-kdl-dev libnlopt-cxx-dev liburdfdom-dev swig python3-dev
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
ctest --test-dir build --output-on-failure
```

`-DTRAC_IK_BUILD_PYTHON=OFF` 可关闭 Python 绑定；`-DTRAC_IK_BUILD_EXAMPLES=OFF` 可关闭示例。
URDF 通过独立的 urdfdom 解析，不依赖 ROS 的 urdf/kdl_parser 包。

本次环境的 NLopt 未安装到系统，而是将发行版 deb 解压至 `/tmp/trac-ik-deps/root`；复现当前构建时加：

```sh
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_PREFIX_PATH=/tmp/trac-ik-deps/root/usr
```

该临时路径不是项目运行要求；正常安装依赖后不需要此参数。

## C++

```cpp
#include <trac_ik/trac_ik.hpp>
// xml 为 URDF 文件内容，不是文件路径或 ROS 参数名。
TRAC_IK::TRAC_IK solver("base", "tip", xml, 0.005, 1e-5, TRAC_IK::Speed);
// 也可直接传入 KDL::Chain 和 KDL::JntArray 上下限。
int rc = solver.CartToJnt(seed, target_frame, result);
// rc >= 0 表示成功；负值表示失败。
```

安装和下游 CMake：

```sh
cmake --install build --prefix "$HOME/.local"
```

```cmake
find_package(trac_ik CONFIG REQUIRED)
target_link_libraries(my_app PRIVATE trac_ik::trac_ik)
```

下游配置时使用 `-DCMAKE_PREFIX_PATH=安装前缀`。非系统位置的依赖也应加入该路径，
并在运行时通过系统动态链接配置或 `LD_LIBRARY_PATH` 提供。

## Python

```sh
export PYTHONPATH="$PWD/build/python${PYTHONPATH:+:$PYTHONPATH}"
```

```python
from pathlib import Path
from trac_ik_python.trac_ik import IK
ik = IK("base", "tip", urdf_string=Path("tests/robot.urdf").read_text())
solution = ik.get_ik([0] * ik.number_of_joints, 0.1, 0.2, 0.3, 0, 0, 0, 1)
print(solution)  # 关节值；不可达时为 None
```

`urdf_string` 必须显式提供。Python 模式名为 `Speed`、`Distance`、
`Manipulation1`、`Manipulation2`、`Manipulation3`。无效模型、限位和模式抛出 `ValueError`。
Python 包及原生扩展由 CMake 一起安装到 `lib/pythonX.Y/site-packages`；可用
`TRAC_IK_PYTHON_INSTALL_DIR` 调整位置，非系统安装时需设置对应的 `PYTHONPATH`。
`setup.py` 仅描述包元数据，不负责构建原生扩展。

## 示例与测试

```sh
build/trac_ik_examples/ik_tests tests/robot.urdf base tip 100
ctest --test-dir build -V
```

示例生成固定随机种子的可达目标，用 FK 检查 IK 解的位姿误差和关节限位。
测试覆盖五种求解模式、独立 KDL 构造、URDF 解析、固定/旋转/连续/移动关节、
安全限位、不可达目标、超时、无效输入和 Python 绑定。

## 接口迁移与边界

- 已移除 ROS Node、Logger、参数服务器接口；计时使用 `std::chrono::steady_clock`。
- URDF 支持从祖先 base 到后代 tip 的串联链；拒绝 mimic、floating、planar 关节及无活动关节链。
- 一个 solver 实例不能同时被多个外部线程调用；并行请求请使用不同实例。
- 仓库仅保留独立核心库、Python 绑定、示例和测试；历史 ROS 清单、launch、元包及 MoveIt 插件已删除。
