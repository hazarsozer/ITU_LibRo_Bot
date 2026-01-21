# ITU LibRo: Autonomous Library Robot with Visual Perception

## 📂 Project Structure
* **`scripts/`**: Contains the main `libro_visual_nav.py` controller and YOLO weights.
* **`maps/`**: Contains the `.yaml` and `.pgm` map files for Navigation 2.
* **`models/`**: Contains custom assets (e.g., `itu_walls`) required for the simulation.
* **`worlds/`**: Contains the final `library_final.world` simulation environment.

## 📦 Prerequisites
1. **ROS 2 Humble** & **Gazebo** installed.
2. **TurtleBot3 Packages** installed.
3. **Python Dependencies**:
   ```bash
   pip3 install -r requirements.txt
   ```

## 🚀 How to Run (4 Terminal Setup)

**NOTE:** Please update the paths below (`~/Projects/Robotics/libro_ws/src/...`) to match the location of this package on your machine.

### Terminal 1: Simulation World

This loads Gazebo with our custom library world and models.

```bash
export TURTLEBOT3_MODEL=waffle_pi
# CRITICAL: Adds our custom walls to Gazebo's model path
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/Projects/Robotics/libro_ws/src/libro_perception/models
# Launches the world
ros2 launch gazebo_ros gazebo.launch.py world:=~/$YOURPATH/libro_ws/src/libro_perception/worlds/library_final.world
```

### Terminal 2: State Publisher

Initiates the Robot State Publisher.

```bash
export TURTLEBOT3_MODEL=waffle_pi
ros2 run robot_state_publisher robot_state_publisher --ros-args -p use_sim_time:=true -p robot_description:="$(xacro /opt/ros/humble/share/turtlebot3_description/urdf/turtlebot3_waffle_pi.urdf)"
```

### Terminal 3: Navigation Stack (Nav2)

This loads the map and localization system.

```bash
export TURTLEBOT3_MODEL=waffle_pi
# Loads our specific map from the 'maps' folder
ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True map:=~/$YOURPATH/libro_ws/src/libro_perception/maps/library_final_map.yaml
```

### Terminal 4: Main Controller

This runs the visual perception and navigation logic.

```bash
# Navigate to the scripts folder first so YOLO finds the weights
cd ~/$YOURPATH/libro_ws/src/libro_perception/scripts/
python3 libro_navigation.py
```
