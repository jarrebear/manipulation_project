

# Perception-Driven Pick & Place (ROS 2 / MoveIt 2)

A ROS 2 manipulation project in which a robot arm **detects a block with a depth camera and picks it up and places it**, working in both **Gazebo simulation** and on **real hardware**. A Python perception node turns raw point clouds into object poses, and C++ MoveIt 2 nodes use those poses to plan and execute the grasp.

Built as the manipulation project of the ROS Masterclass by The Construct.

## Demos

### Object Detection

https://github.com/user-attachments/assets/6a800246-0fd1-48b4-9c27-7647109cbacb

### Real Robot Pick and Place

https://github.com/user-attachments/assets/3073a931-b63c-4616-9655-1871483f8c74

## At a Glance

| | |
|---|---|
| **What it does** | Detects a table surface and objects on it from a point cloud, then picks an object and places it using MoveIt 2 |
| **Languages / stack** | C++ (MoveIt 2 `MoveGroupInterface`), Python (rclpy, python-pcl, NumPy), ROS 2, RViz2, CMake / colcon |
| **Robot** | UR3e 6DOF arm with a gripper (MoveIt planning groups `ur_manipulator` and `gripper`) |
| **Sensor** | RGB-D depth camera (wrist-mounted in simulation, `/camera/depth/color/points` on the real robot) |
| **Environments** | Gazebo simulation and real hardware, each with its own configuration |
| **Packages** | `object_detection`, `moveit2_scripts`, `custom_msgs`, `my_moveit_config` (sim), `real_moveit_config` (real) |

## What to Expect From This Repo

- **A full perception-to-action pipeline,** not just motion planning: point cloud → filtering → plane segmentation → clustering → object pose → grasp.
- **Sim-to-real work.** The same pipeline runs in simulation and on a physical robot, with separate MoveIt configs, tuned filter bands, grasp offsets and gripper control for each.
- **Motion planning with MoveIt 2.** Joint-space and pose-goal planning, named poses, Cartesian approach/retreat paths and independent control of arm and gripper groups.
- **Clean ROS 2 structure.** Custom message interfaces, launch files for every mode, and perception and manipulation as separate, decoupled packages.

## Skills Demonstrated

| Area | Where it shows up |
|---|---|
| Point cloud processing (PCL) | Pass-through filtering, RANSAC plane segmentation, Euclidean clustering in `object_detection.py` |
| 3D geometry & TF2 | Transforming clouds from the camera frame to `base_link` with quaternion → rotation matrix maths; static camera transform |
| Motion planning (MoveIt 2) | `pick_and_place*.cpp`: pose targets, named targets, Cartesian paths, joint-value targets |
| Manipulation logic | Approach, grasp, retreat, rotate and release sequence driven by detected object pose |
| Perception ↔ planning integration | The pick nodes subscribe to `/object_detected` and compute the grasp pose from the object's centroid and size |
| Custom interfaces | `DetectedObjects.msg`, `DetectedSurfaces.msg` |
| Sim-to-real adaptation | Separate `*_real` nodes, launch files, MoveIt configs, thresholds and gripper commands |
| Visualisation | RViz marker arrays for detected surfaces (green) and objects (red) |
| Concurrency | Separate executor threads for the MoveIt interface and the detection subscriber; mutex-protected shared data |
| Build & tooling | Mixed C++ / Python multi-package colcon workspace, launch files in Python |

## Repository Structure

```
manipulation_project/
├── custom_msgs/
│   └── msg/
│       ├── DetectedObjects.msg
│       └── DetectedSurfaces.msg
├── object_detection/
│   ├── object_detection/
│   │   ├── object_detection.py
│   │   ├── object_detection_real.py
│   │   ├── static_transform_publisher.py
│   │   └── test_pointcloud.py
│   ├── launch/
│   │   ├── object_detection.launch.py
│   │   └── object_detection_real.launch.py
│   └── rviz/
│       ├── rviz_config.rviz
│       └── rviz_config_real.rviz
├── moveit2_scripts/
│   ├── src/
│   │   ├── pick_and_place.cpp
│   │   ├── pick_and_place_perception.cpp
│   │   ├── pick_and_place_real.cpp
│   │   └── pick_and_place_perception_real.cpp
│   └── launch/
│       ├── pick_and_place.launch.py
│       ├── pick_and_place_perception.launch.py
│       ├── pick_and_place_real.launch.py
│       └── pick_and_place_perception_real.launch.py
├── my_moveit_config/        # MoveIt config for the simulated robot
└── real_moveit_config/      # MoveIt config for the real robot
```

## What Each Package and File Does

### `custom_msgs`: message interfaces

| File | Purpose |
|---|---|
| `msg/DetectedObjects.msg` | One detected object: `object_id`, `position` (centroid), `height`, `width`, `thickness`. |
| `msg/DetectedSurfaces.msg` | One detected surface: `surface_id`, `position`, `height`, `width`. |

### `object_detection`: perception (Python)

| File | What it does | Key techniques |
|---|---|---|
| `object_detection.py` | **Simulation perception node.** Subscribes to the wrist camera point cloud, transforms it into `base_link` using TF, filters it by distance and height, then runs two pipelines: (1) RANSAC plane fit + clustering to find the **surface**, and (2) clustering of the points sitting above it to find **objects**. For each cluster it computes the centroid and bounding-box size, then publishes both RViz markers and `DetectedObjects` / `DetectedSurfaces` messages. | PCL, RANSAC (0.01 m threshold), Euclidean clustering (2 cm tolerance, 100 to 80,000 points), TF2, NumPy |
| `object_detection_real.py` | **Real-robot perception node.** Same pipeline with the real camera topic (`/camera/depth/color/points`) and re-tuned filter bands for the real setup (tighter reach of 0.40 m and different height windows for surface and object points). | Sim-to-real parameter tuning |
| `static_transform_publisher.py` | Publishes the fixed `base_link` → camera optical frame transform used in simulation. | TF2 static broadcaster |
| `test_pointcloud.py` | Debug utility that reads the real camera cloud and inspects it in the `base_link` frame, used while calibrating the real setup. | Debugging, `sensor_msgs_py` |
| `launch/object_detection.launch.py` | Starts the static transform, detection node and RViz2 for simulation (`use_sim_time: true`). | Launch files |
| `launch/object_detection_real.launch.py` | Starts the real detection node and RViz2 (`use_sim_time: false`). | Launch files |
| `rviz/*.rviz` | RViz layouts showing the cloud and detection markers for sim and real. | |

### `moveit2_scripts`: manipulation (C++)

| File (executable) | What it does |
|---|---|
| `pick_and_place.cpp` (`pick_and_place`) | **Baseline, fixed-pose pick and place in simulation.** Moves home, goes above a known block pose, opens the gripper, descends along a Cartesian path, closes the gripper gradually, retreats, rotates the base 180°, releases and returns home. |
| `pick_and_place_perception.cpp` (`pick_and_place_perception`) | **Perception-driven version (simulation).** Waits for `/object_detected`, then builds the grasp pose from the detected object's centroid and dimensions (gripper pointing down) and runs the same pick, rotate and place sequence. |
| `pick_and_place_real.cpp` (`pick_and_place_real`) | **Fixed-pose version for the real robot.** Same flow with real-world poses, a different approach distance (5.8 cm) and a single-step gripper close. |
| `pick_and_place_perception_real.cpp` (`pick_and_place_perception_real`) | **Perception-driven version for the real robot.** Uses the detected object position plus small empirically tuned offsets to compensate for real-world calibration error. |
| `launch/*.launch.py` | One launch file per executable. Each loads the matching MoveIt config (`my_moveit_config` or `real_moveit_config`) and sets `use_sim_time` correctly for sim vs real. |

### MoveIt configuration

| Package | Purpose |
|---|---|
| `my_moveit_config` | MoveIt 2 config for the simulated robot: SRDF (planning groups `ur_manipulator` and `gripper`, named poses `home` and `gripper_open`), kinematics, joint limits and controllers. |
| `real_moveit_config` | The same for the real robot, plus a 3D sensor configuration. |

## How the Pipeline Works

```
Depth camera ──► object_detection ──► /object_detected (DetectedObjects)
(PointCloud2)     │  1. TF: camera frame → base_link        │
                  │  2. Pass-through filter (reach, height) ▼
                  │  3. RANSAC plane → surface      pick_and_place_perception
                  │  4. Euclidean clustering →        1. Wait for a detection
                  │     objects (centroid + size)     2. Home → pre-grasp above object
                  ▼                                   3. Open gripper, Cartesian descent
            RViz markers                              4. Close gripper, Cartesian retreat
          (surface: green,                            5. Rotate base 180°, release
           objects: red)                              6. Return home
```

## Getting Started

### Requirements
- Ubuntu with ROS 2 (Jazzy) and MoveIt 2
- `python-pcl` (PCL Python bindings) for the detection nodes
- A simulated or real robot arm exposing the `ur_manipulator` and `gripper` MoveIt groups and a depth camera publishing a `PointCloud2`
- The simulation or hardware bringup itself (robot, controllers, camera) is provided by your course environment and is **not** included in this repository

### Build
```bash
cd ~/ros2_ws/src
git clone https://github.com/jarrebear/manipulation_project.git
cd ~/ros2_ws
colcon build --packages-select custom_msgs object_detection moveit2_scripts my_moveit_config real_moveit_config
source install/setup.bash
```
### Run in simulation
With the simulation running:
```bash
# Terminal 1: perception (static transform + detection + RViz)
ros2 launch object_detection object_detection.launch.py

# Terminal 2: MoveIt 2 move_group
ros2 launch my_moveit_config move_group.launch.py

# Terminal 3: perception-driven pick and place
ros2 launch moveit2_scripts pick_and_place_perception.launch.py
```
For the fixed-pose baseline, run `ros2 launch moveit2_scripts pick_and_place.launch.py`.

### Run on the real robot
With the robot and camera driver running:
```bash
ros2 launch object_detection object_detection_real.launch.py
ros2 launch real_moveit_config move_group.launch.py
ros2 launch moveit2_scripts pick_and_place_perception_real.launch.py
```

## Design Notes

- **Why separate perception and manipulation packages?** The detection node only publishes poses, so it can be tested in RViz on its own and swapped for a different detector without touching the planning code.
- **Why separate `*_real` nodes?** The real setup differs in camera topic, mounting, calibration, filter ranges, approach distance and gripper behaviour. Keeping them separate made it safe to tune the real robot without breaking simulation.
- **Why Cartesian paths for approach and retreat?** A straight vertical line keeps the gripper aligned over the block, which joint-space planning does not guarantee.
- **Why a gradual gripper close in simulation?** Closing in small steps with short pauses gave the physics engine time to register the grasp reliably.

## Known Limitations & Possible Improvements

- Detection is **geometry only** (no object classification) and the pick node uses the **most recent single detection**, so it assumes one object on the table.
- The cloud transform loops over points in Python, which is slow. Vectorising it with NumPy or using `tf2_sensor_msgs` would raise the processing rate significantly.
- Grasp offsets and filter thresholds are hand-tuned constants. Moving them to ROS 2 parameters would make the system easier to re-calibrate.
- Add a grasp-success check (for example gripper position or force feedback) and retry logic.
- Add collision objects for the table and detected block to the MoveIt planning scene.

## License

Released under the [MIT License](LICENSE).

## Author

**jarrebear**: [github.com/jarrebear](https://github.com/jarrebear)
