# PIN-IK Python 3 Bindings

Standalone Python bindings for PIN-IK (Pinocchio Inverse Kinematics).

Build with CMake from the repository root. See [the repository README](../README.md).

The URDF XML must be supplied explicitly; no ROS parameter server is used.

## Usage

```python
from pin_ik_python.pin_ik import IK

solver = IK("base_link", "tip_link", urdf_string="...", 
            timeout=0.005, epsilon=1e-5, solve_type="Speed")

solution = solver.CartToJnt(seed, x, y, z, qx, qy, qz, qw)
```
