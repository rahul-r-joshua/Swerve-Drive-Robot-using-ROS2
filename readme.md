# 🤖 Swerve Drive Robot using ROS2

This repository contains a ROS2 workspace implementing a **swerve-drive robot** with simulated and teleoperated control.  

---

## 📌 Prerequisites

Make sure you have the following installed:

- 🐧 **Ubuntu 22.04** or later  
- 🌐 **ROS2 Humble** (or appropriate ROS2 distro)  
  👉 Installation instructions: [ROS2 Installation Guide](https://docs.ros.org/en/humble/Installation.html)

---

## ⚙️ Installation 🚀

### 1️⃣ Create a ROS2 workspace
```bash
mkdir -p ~/rocker_ws
```
 
## 2️⃣ Clone the repository 📥
   ```bash
   cd ~/rocker_ws/
   git clone https://github.com/rahul-r-joshua/Swerve-Drive-Robot-using-ROS2.git
```

## 3️⃣ Build the workspace 🏗️

```bash
colcon build --symlink-install
```

## 4️⃣ Source the workspace ⚡

```bash
source install/setup.bash
```

## 🖥️ Terminal 1

```bash
source install/setup.bash

ros2 launch rocker_bringup simulated_robot.launch.py use_sim_time:=true
```

## 🖥️ Terminal 2

```bash
source install/setup.bash

ros2 run rocker_controller simple_follow_path.py 
```

## 🖥️ Terminal 3

```bash
source install/setup.bash

ros2 run rocker_controller simple_pub_circle.py 
```

## 🎮 To Use with Teleop

```bash
source install/setup.bash

ros2 run teleop_twist_keyboard teleop_twist_keyboard cmd_vel:=key_vel
```
## 🚀 Usage and Commands

Use the following commands to launch and interact with the robot:

| 🔍 Description                    | 💻 Command                                                                 |
|---------------------------------|---------------------------------------------------------------------------|
| 🕹️ Launch the robot in simulation | `ros2 launch rocker_bringup simulated_robot.launch.py use_sim_time:=true` |
| ▶️ Start follow path controller   | `ros2 run rocker_controller simple_follow_path.py`                        |
| 🔄 Start random circle trajectory | `ros2 run rocker_controller simple_pub_circle.py`                         |
| ⌨️ Operate via keyboard (teleop)  | `ros2 run teleop_twist_keyboard teleop_twist_keyboard cmd_vel:=key_vel`    |
| 🎮 Operate via joystick (teleop)  | Plug in a joystick and press **Button 4 (top-left corner)** to enable movement (deadman lock). |

---


### 🎮 Joystick and Twist Mux

- ⏸️ **Deadman Lock**: The joystick requires you to hold **Button 4 (top-left corner)**.  
  If not pressed, motion commands will be ignored for safety. 🛑

- 🎛️ **Twist Mux Priorities (highest → lowest)**:  
  1. 🚨 **Safety Stop**  
  2. 🎮 **Joystick**  
  3. ⌨️ **Keyboard**  
  4. 🔄 Other `cmd_vel` inputs  

---

### 🛑 Safety Stop

You can test the **safety stop** override using the following commands:

- ✅ **Enable safety stop (robot will NOT move):**
  ```bash
  ros2 topic pub /safety_stop std_msgs/msg/Bool "data: true"
  ```

- **▶️ Disable safety stop (robot WILL move):**
  ```bash
  ros2 topic pub /safety_stop std_msgs/msg/Bool "data: false"
  ```

  
## 👨‍👩‍👧 Contributors

- 🧑‍🏫 **Dr. Santhakumar Mohan** (Professor, IIT Palakkad) – Overall Supervision

- 🧑‍🏫 **Ajay Krishnan** (Ph.D., IIT Palakkad) – Guidance & Mentorship 📧 [GitHub Profile](https://github.com/Ajayazimo)

- 👩‍💻 **Sivani GP** – CAD Modelling & URDF Development 📧 [GitHub Profile](https://github.com/sivani-gp)
