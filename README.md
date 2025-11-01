# 🚀 Custom DWA Local Planner in ROS2 Humble

This project implements a **Custom Dynamic Window Approach (DWA)** local planner for a TurtleBot3 in **ROS2 Humble**.  
The planner generates real-time velocity commands (`/cmd_vel`) for goal navigation and obstacle avoidance, fully written from scratch without using `nav2_dwb_controller`.

---

## 🧩 Objective

To design and implement a DWA-based local planner that:
- Samples velocity commands within dynamic limits  
- Predicts trajectories for each velocity sample  
- Evaluates them using a custom cost function (goal distance, obstacle avoidance, smoothness)  
- Publishes the best velocity command to `/cmd_vel`  
- Visualizes trajectories in **RViz**

---

## ⚙️ System Requirements

| Component | Version |
|------------|----------|
| Ubuntu | 22.04 LTS |
| ROS2 | Humble Hawksbill |
| Gazebo | Classic |
| TurtleBot3 | burger / waffle |
| Python | ≥ 3.8 |

---

## 🧮 Setup Instructions

### 1️⃣ Install ROS2 Humble and TurtleBot3 Simulation
```bash
sudo apt update
sudo apt install ros-humble-desktop -y
sudo apt install -y ros-humble-gazebo-ros-pkgs ros-humble-gazebo-ros2-control
echo "export TURTLEBOT3_MODEL=burger" >> ~/.bashrc
source ~/.bashrc
```

### 2️⃣ Clone Dependencies
```bash
cd ~/ros2_ws/src
git clone -b humble https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git
git clone -b humble https://github.com/ROBOTIS-GIT/turtlebot3.git
git clone -b humble https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git
git clone -b humble https://github.com/ROBOTIS-GIT/DynamixelSDK.git
```

### 3️⃣ Install Dependencies
```bash
sudo apt install -y python3-colcon-common-extensions python3-pip python3-rosdep
sudo rosdep init || true
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

### 4️⃣ Clone the Custom DWA Planner
```bash
cd ~/ros2_ws/src
git clone https://github.com/naitikpahwa18/dwa_planner.git
```

### 5️⃣ Update Bashrc with Gazebo Environment Variables
```bash
echo "export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/usr/share/gazebo-11/models" >> ~/.bashrc
echo "export GAZEBO_PLUGIN_PATH=$GAZEBO_PLUGIN_PATH:/opt/ros/humble/lib" >> ~/.bashrc
echo "export GAZEBO_RESOURCE_PATH=$GAZEBO_RESOURCE_PATH:/usr/share/gazebo-11" >> ~/.bashrc
source ~/.bashrc
```

---

## 🧱 Build & Run

### 🏗️ Build the Package
```bash
cd ~/ros2_ws
colcon build --packages-select dwa_planner --symlink-install
source install/setup.bash
```

### 🚀 Run Simulation

**Terminal 1 – Launch TurtleBot3 in Gazebo**
```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

**Terminal 2 – Run the DWA Planner Node**
```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 run dwa_planner dwa_node
```

**Terminal 3 – Visualize in RViz**
```bash
rviz2
```
In RViz:
- Fixed Frame → `odom`
- Add topics:
  - `/scan` → LaserScan  
  - `/odom` → Odometry  
  - `/cmd_vel` → Velocity Command  
  - `/dwa_markers` → MarkerArray (visualized trajectories)

---

## 🎯 Setting a Goal

Use parameters to set the goal dynamically:
```bash
ros2 param set /dwa_planner goal_x 2.0
ros2 param set /dwa_planner goal_y 1.0
```

> You can modify these coordinates as per your test environment.

---

## 🧮 Core Algorithm

1. **Velocity Sampling** — Generate a set of linear & angular velocity pairs within dynamic limits.  
2. **Trajectory Prediction** — Simulate the robot’s motion for each pair over a short horizon.  
3. **Cost Evaluation** — Score each trajectory using a multi-term cost function:  
   - Distance to goal  
   - Proximity to obstacles  
   - Smoothness (low angular velocity penalty)  
4. **Best Command Selection** — Choose the velocity with the minimum total cost and publish to `/cmd_vel`.

> The implementation has been optimized for stable, smooth navigation and reduced oscillations near obstacles.

---

## 🧾 Topics Used

| Topic | Type | Description |
|--------|------|-------------|
| `/scan` | `sensor_msgs/LaserScan` | Obstacle detection |
| `/odom` | `nav_msgs/Odometry` | Robot position and orientation |
| `/cmd_vel` | `geometry_msgs/Twist` | Velocity commands sent to robot |
| `/dwa_markers` | `visualization_msgs/MarkerArray` | RViz visualization of trajectories |

---

## 🧪 Expected Output

- TurtleBot3 navigates toward the goal while avoiding obstacles.  
- Smooth trajectory visualization in RViz.  
- Debugging info printed in terminal (trajectory evaluation, best velocities, etc.).

---

## ⚠️ Common Issues

| Issue | Cause | Fix |
|--------|--------|-----|
| **Terminal 1 stuck at “spawn_entity”** | Environment variables or package sourcing not set correctly | Ensure the following lines exist in your `~/.bashrc`:<br><br>`source /opt/ros/humble/setup.bash`<br>`source ~/ros2_ws/install/setup.bash`<br><br>`export TURTLEBOT3_MODEL=burger`<br>`export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/usr/share/gazebo-11/models`<br>`export GAZEBO_PLUGIN_PATH=$GAZEBO_PLUGIN_PATH:/opt/ros/humble/lib`<br>`export GAZEBO_RESOURCE_PATH=$GAZEBO_RESOURCE_PATH:/usr/share/gazebo-11` |
| **Planner works only for certain goal points** | Algorithm parameters tuned for specific map/goal | Adjust cost weights, velocity limits, and goal tolerance in `dwa_node.py` for better generalization |
| **Robot collides with obstacles** | Safety distance too low in obstacle cost calculation | Increase the obstacle cost or minimum safety distance parameter in `dwa_node.py` |
| **Robot slows excessively near obstacles** | Cost weights imbalance | Tune the weights for `heading_weight`, `obs_weight`, and `velocity_weight` in `dwa_node.py` |
| **Build errors or missing packages** | Stale build, missing dependencies | Clean the workspace and rebuild:<br>`rm -rf build/ install/ log/`<br>`colcon build --packages-select dwa_planner`<br>`source install/setup.bash` |

---

## 🧠 Acknowledgements

- [ROS2 Documentation](https://docs.ros.org/en/humble/)
- [TurtleBot3 Simulation Manual](https://emanual.robotis.com/docs/en/platform/turtlebot3/overview/)
- [Dynamic Window Approach Alogrithm](https://www.youtube.com/watch?v=16TJ6NiHo6A)
