# PIN-IK C++ Library

PIN-IK (Pinocchio Inverse Kinematics) is a standalone C++17 IK solver based on Pinocchio 4.x.

See [the repository README](../README.md) for dependencies, build, install and API usage.

The library can also be built directly with:
```bash
cmake -S pin_ik_lib -B build-lib -DCMAKE_PREFIX_PATH=/opt/openrobots
cmake --build build-lib -j$(nproc)
```
