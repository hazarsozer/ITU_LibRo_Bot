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
ros2 launch gazebo_ros gazebo.launch.py world:=~/Projects/Robotics/libro_ws/src/libro_perception/worlds/library_final.world
```

### Terminal 2: Spawn Robot

This spawns the TurtleBot3 Waffle Pi at the library entrance.

```bash
export TURTLEBOT3_MODEL=waffle_pi
ros2 run gazebo_ros spawn_entity.py -file `ros2 pkg prefix turtlebot3_gazebo`/share/turtlebot3_gazebo/models/turtlebot3_waffle_pi/model.sdf -entity turtlebot3_waffle_pi -x -8.74 -y 2.83 -z 0 &
ros2 run robot_state_publisher robot_state_publisher --ros-args -p use_sim_time:=true -p robot_description:="$(xacro /opt/ros/humble/share/turtlebot3_description/urdf/turtlebot3_waffle_pi.urdf)"
```

### Terminal 3: Navigation Stack (Nav2)

This loads the map and localization system.

```bash
export TURTLEBOT3_MODEL=waffle_pi
# Loads our specific map from the 'maps' folder
ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True map:=~/Projects/Robotics/libro_ws/src/libro_perception/maps/library_final_map.yaml
```

### Terminal 4: Main Controller

This runs the visual perception and navigation logic.

```bash
# Navigate to the scripts folder first so YOLO finds the weights
cd ~/Projects/Robotics/libro_ws/src/libro_perception/scripts/
python3 libro_visual_nav.py
```

### ✅ Final Verification

1. **Open `README.md`** and paste the text above.
2. **Save and Close.**
3. **Run Git Commands:**

```bash
git add .
git commit -m "Updated README with correct paths for maps, models, and scripts"
git push origin main
```

Now your submission is clean, documented, and points to all the correct files! 🎓

