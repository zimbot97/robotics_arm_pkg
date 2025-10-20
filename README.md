# 🤖 robotics_arm_pkg

**`robotics_arm_pkg`** is a **ROS 2 metapackage** that combines all components required for simulation and real-world control of a custom robotic arm.  
It integrates MoveIt motion planning, robot description files, servo hardware control, and launch configurations — designed for standard PWM servos like **MG996R**.

---

## 📦 Package Overview

| Package | Description |
|----------|-------------|
| **`robotics_arm_moveit`** | MoveIt configuration for motion planning, visualization, and real-hardware integration. |
| **`robotics_description`** | Contains URDF/Xacro files, 3D meshes, and robot model assets. |
| **`robotics_arm_control`** | Hardware interface and Arduino communication layer for PWM servo control. |
| **`robotics_arm_bringup`** | Launch and parameter configurations for simulation and real-arm operation. |

---

## 🧩 Features

- 🦾 **MoveIt + RViz2 Integration** — Full motion planning pipeline and trajectory visualization  
- ⚙️ **Arduino-Based Servo Control** — Compatible with standard PWM servos (MG996R)  
- 🧱 **Modular ROS 2 Design** — Easy customization and extension  
- 🧠 **Simulation + Real Hardware** — Single framework for both virtual and physical testing  
- 🧩 **URDF and Xacro** — Supports joint visualization with `joint_state_publisher_gui`

---

## 🚀 Getting Started

### 1️⃣ Clone the Repository
```bash
cd ~/ros2_ws/src
git clone https://github.com/zimbot97/robotics_arm_pkg.git
```

### 2️⃣ Build the Workspace
```bash
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

---

### 3️⃣ Run URDF + Joint State Publisher GUI
Use this for **basic visualization and joint testing** in RViz:
```bash
ros2 launch robotics_arm_bringup view_robot.launch.py
```

🎥 **Demo Video:** [URDF Visualization & Joint Control [![Watch the video](https://img.youtube.com/vi/IXoSNhc9wtE/maxresdefault.jpg)](https://www.youtube.com/watch?v=IXoSNhc9wtE)


---

### 4️⃣ Run MoveIt + Real Servo Arm
For **MoveIt motion planning and execution on the physical arm**, ensure your Arduino is flashed with the servo control code from `robotics_arm_control/arduino/`.

Then launch:
```bash
ros2 launch robotics_arm_bringup bringup.launch.py use_sim:=false
```

🎥 **Demo Video:** [MoveIt Planning with Real Robotic Arm][![Watch the video](https://img.youtube.com/vi/SRe8D7kJSTs/maxresdefault.jpg)](https://www.youtube.com/watch?v=SRe8D7kJSTs)


---

## 🧠 Notes

- Default servo model: **MG996R (PWM control)**  
- Servo feedback not available — MoveIt visualization shows desired motion  
- Tested on **ROS 2 Humble** (compatible with Foxy and Iron)  
- Works in both **simulation** and **real-world** setups  

---

## 🔗 Repository

GitHub: [https://github.com/zimbot97/robotics_arm_pkg](https://github.com/zimbot97/robotics_arm_pkg)

---

## 🧑‍💻 Author

Developed by **Brian Lai**  
Robotics & Automation Engineer | ROS Developer  

---
