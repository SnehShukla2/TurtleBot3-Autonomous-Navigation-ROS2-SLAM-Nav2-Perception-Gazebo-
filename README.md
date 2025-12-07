# TurtleBot3 ROS2 Autonomous Navigation

<img width="857" height="575" alt="Screenshot 2025-04-19 123756" src="https://github.com/user-attachments/assets/481c496a-76d6-4fd0-b421-4ab4e2e5f04f" />

<img width="826" height="455" alt="Screenshot 2025-06-06 210925" src="https://github.com/user-attachments/assets/a0d1faf7-55ac-43ac-b4db-49e7aa91fffb" />


<img width="776" height="431" alt="Screenshot 2025-06-06 212656" src="https://github.com/user-attachments/assets/4069d8af-f234-4107-a2cf-5886c38642dd" />


# TurtleBot3 ROS2 Autonomous Navigation

This repository contains a complete ROS2-based autonomy stack for a TurtleBot3-style mobile robot in Gazebo, including teleoperation, SLAM, global path planning, and object detection. The project ties together a custom TurtleBot3 URDF, a custom Gazebo world, `slam_toolbox` for online mapping, an A* planner for path generation, and RViz2 for rich visualization of maps, poses, and paths.

The focus is on reproducible bring-up of an autonomous navigation pipeline:
- Launch a TurtleBot3 in a custom Gazebo environment and drive it with keyboard teleop.
- Build a 2D occupancy grid map in real time using `slam_toolbox` with simulated time.
- Plan and visualize collision-free paths using an A* planner over the generated map.
- Run a separate ROS2 workspace for YOLO-based object detection on camera data.

This repo excludes the YOLOv3 weights file (src/yolov3.weights, >200MB).
See README for instructions to download it from the official source before running object detection.
---
## How to Run

---

### 1. Build TurtleBot3 Gazebo workspace
```
cd ~/Sneh_project_adjustments/src/turtlebot3_ws
rm -rf build install log
colcon build --cmake-clean-cache
source install/setup.bash
```

---

### 2. Launch Gazebo with custom world

```
ros2 launch turtlebot3_gazebo turtlebot3_my_world.launch.py
```
---
### 3. Teleop control (New Terminal)

```
source ~/Sneh_project_adjustments/src/turtlebot3_ws/install/setup.bash
ros2 run turtlebot3_teleop teleop_keyboard
```
---
### 4. Object detection
In a New Terminal:

```
cd ~/Sneh_project_backup_copy/src/ros2_ws
rm -rf build install log
colcon build --packages-select object_detection --symlink-install
source install/setup.bash
ros2 launch object_detection object_detection.launch.py
```
---
### 5. SLAM with slam_toolbox

Start simulation (robot state publisher)
Then, In a New Terminal:
```
ros2 launch turtlebot3_bringup turtlebot3_state_publisher.launch.py use_sim_time:=True
```

Start SLAM (In a New Terminal)

```
ros2 launch slam_toolbox online_async_launch.py use_sim_time:=True
```


Inspect map topic (In a new Terminal)
```
ros2 topic echo /map
```


Visualize in RViz2 (In a New Terminal)
```
rviz2
```
---
### 6. A* path planning
Keep SLAM and Robot state publisher on then Run A* planner (In a New Terminal):

```
~/Sneh_project_done/install/astarplanner/bin/astar_planner
```

To visualize the planned path, open RViz2 and add the path planning topics/markers.
---
### 7. TF frames PDF

In a New Terminal Run:
```
ros2 run tf2_tools view_frames
```

---
## Downloading YOLOv3 Weights (Required for Object Detection)

GitHub does not allow files larger than 100 MB to be stored in the repo, so the YOLOv3 weights file (`src/yolov3.weights`, ~200+ MB) must be downloaded manually before running the object detection node.

From your home directory:

```
cd ~/Sneh_project_adjustments/src
```
Download official YOLOv3 COCO weights from the original YOLO website
```
wget https://pjreddie.com/media/files/yolov3.weights
```

This will create the file:
~/Sneh_project_adjustments/src/yolov3.weights


The repository already includes the matching configuration and class labels:

- `src/yolov3.cfg`
- `src/coco.names`

With all three files in place (`yolov3.weights`, `yolov3.cfg`, `coco.names`), you can now follow the **Object detection** steps in the section above.
