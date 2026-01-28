# 📚 ITU LibRo: Autonomous Library Guide-Bot

**A ROS 2 Humble autonomous mobile robot designed to guide users through the ITU Mustafa Inan Library.**

This project integrates **Deep Learning (YOLOv8)** for dynamic safety, **Computer Vision (ArUco)** for precise localization, and the **Nav2 Stack** for intelligent pathfinding.

---

## 🎥 Project Demo

[![Watch the Demo](https://img.youtube.com/vi/xhuddK0yB_I/0.jpg)](https://www.youtube.com/watch?v=xhuddK0yB_I)

*(Click the image above to watch the full system demonstration)*

---

## 🌟 Key Features

### 1. 🧠 Intelligent Navigation
Uses the **ROS 2 Navigation Stack (Nav2)** to plan optimal paths through a custom-mapped environment. The robot can navigate complex aisles and dynamically re-plan routes if a path is blocked by static obstacles.

### 2. 👁️ Visual Perception & Safety (YOLOv8)
Unlike standard lidar-only robots, LibRo uses a **RGB Camera** with **YOLOv8** to detect humans.
* **Safety Stop:** If a human steps in front of the robot, it triggers an "Emergency Brake" visual safety protocol.
* **Dynamic Handling:** If the human doesn't move, the robot intelligently decides to reroute around them.

### 3. 📍 Visual Localization & Recovery (ArUco)
Solves the "Kidnapped Robot Problem" using visual landmarks.
* **Verification:** Upon arrival, the robot scans for specific ArUco markers to confirm it reached the correct section (e.g., Engineering vs. Science).
* **Self-Healing:** If the robot gets lost or dragged to a new location manually, it can visually scan the room, identify a marker, and **teleport its internal map position** to match reality instantly.

### 4. 🗣️ Interactive Command Interface
A Python-based terminal GUI allows users to:
* Select specific library sections (Engineering, History, Cafe, etc.).
* Interrupt tasks mid-mission to change destinations.
* Automatically return to the entrance after a period of inactivity ("Welcome Mode").

---

## 🛠️ Technology Stack

* **Framework:** ROS 2 Humble Hawksbill
* **Simulation:** Gazebo Classic
* **Robot:** TurtleBot3 Waffle Pi
* **AI/Vision:** Ultralytics YOLOv8, OpenCV
* **Mapping:** SLAM Toolbox (Serialized Pose Graphs)
* **Language:** Python 3

---

## 🚀 Installation & Usage

The source code and detailed technical instructions are located in the package directory.

👉 **[CLICK HERE FOR RUN INSTRUCTIONS](./src/libro_perception/README.md)**

To run this simulation, you will need the `libro_perception` package which contains:
* The custom World and Models.
* The Navigation Maps (`.yaml`/`.pgm`).
* The Visual Navigation Scripts.

---

*Istanbul Technical University (ITU) - Robotics Project 2026*
