# UR10 Custom Description - ZED Camera and Gas Manifold Integration

This README documents the recent changes made to the `ur10_custom_description` package to integrate the **ZED camera** and **gas manifold** STL meshes onto the UR10 robotic arm. These changes are intended for visualization and simulation using **RViz** (not Gazebo), and support further development in motion planning and scene interaction.

---

## 🛠️ Project Structure

**Main package**: `ur10_custom_description`

```
ur10_custom_description/
├── meshes/
│   ├── zed_camera.stl
│   └── manifold_gas_nozzle.stl
├── urdf/
│   ├── ur10_custom.urdf.xacro
│   └── tools/
│       ├── zed_camera.urdf.xacro
│       ├── gas_manifold.urdf.xacro
│       └── welding_tool.urdf.xacro
└── launch/
    └── view_robot.launch.py
```

---

## 🔄 Description of Changes

### 1. Added Custom Tools Directory
- Created `urdf/tools/` and added:
  - `zed_camera.urdf.xacro`
  - `gas_manifold.urdf.xacro`

### 2. Integrated STL Meshes
- Moved `zed_camera.stl` and `manifold_gas_nozzle.stl` into the package's `meshes/` directory.

### 3. Updated `ur10_custom.urdf.xacro`
- Included tool macros at the top:
  ```xml
  <xacro:include filename="$(find ur10_custom_description)/urdf/tools/zed_camera.urdf.xacro"/>
  <xacro:include filename="$(find ur10_custom_description)/urdf/tools/gas_manifold.urdf.xacro"/>
  ```
- Instantiated them at the end:
  ```xml
  <xacro:zed_camera parent_link="wrist_3_link"/>
  <xacro:gas_manifold parent_link="wrist_3_link"/>
  ```

### 4. Defined Transform and Orientation
- Inside each tool Xacro:
  - Used `<origin xyz=... rpy=...>` to fine-tune position.
  - `scale="0.001 0.001 0.001"` used to reduce STL size appropriately.

Example (from `zed_camera.urdf.xacro`):
```xml
<origin xyz="0.03 0 0.05" rpy="0 0 0"/>
```

### 5. Launch File (Optional)
Created `launch/view_robot.launch.py` to visualize model in RViz:
```python
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': open(
                os.path.join(
                    get_package_share_directory('ur10_custom_description'),
                    'urdf',
                    'ur10_custom.urdf.xacro')
                .read()}],
            output='screen'
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen'
        )
    ])
```

---

## ✅ Usage Instructions

### 1. Build & Source
```bash
cd ~/ws_jesus
colcon build --symlink-install --packages-select ur10_custom_description
source install/setup.bash
```

### 2. Launch RViz with Custom Description
```bash
ros2 launch ur10_custom_description view_robot.launch.py
```

If using MoveIt:
```bash
ros2 launch ur_moveit_config ur_moveit.launch.py ur_type:=ur10 launch_rviz:=true use_fake_hardware:=true use_sim_time:=true
```

---

## ⚙️ Debug Tips

- If meshes do not appear:
  - Ensure STL files are in `meshes/`
  - Check filenames and paths in Xacro
  - Use proper scale (usually `0.001`)
- If model looks misaligned:
  - Use `rpy="..."` to rotate
  - Remember: `xyz` is in **local parent frame**, not world

---

## 🧠 Notes

- The package `ur_description` is **not used** anymore. This setup is fully local and self-contained.
- Gazebo support has been skipped for now, as visualization is focused on RViz.

---
