## Repository for Package Delivery Bot (Team14)

```

├── auto\_delivery\_pkg
│   ├── auto\_delivery\_pkg  
│   │   ├── **init**.py
│   │   ├── oak\_perception\_node.py      \# NEW: Hardware Master (YOLO + RGB Stream)
│   │   ├── apriltag\_node.py            \# Software-only Front Tag Detection
│   │   ├── webcam\_apriltag.py          \# Rear Tag Detection (USB Webcam)
│   │   ├── mission\_controller.py       \# Main State Machine (Ackermann Logic)
│   │   ├── parking\_controller.py
│   │   ├── servo\_controller.py
│   │   └── box\_detection\_no\_rclpy.py   \# Utility/Debug
│   ├── config
│   │   ├── node\_config.yaml            \# Config for node activation
│   │   └── visualization.yaml          \# NEW: Config for GUI/Headless mode
│   ├── launch
│   │   └── mission.launch.py           \# Handles full system launch
│   ├── models
│   │   └── yolov8\_n.blob               \# Trained YOLOv8 model
│   ├── package.xml
│   ├── resource
│   │   └── auto\_delivery\_pkg
│   ├── setup.cfg
│   └── setup.py
├── requirements.txt                    \# Dependencies (depthai, pupil-apriltags)

````

### Setup:

#### 1. Install the package

**Option 1**: Clone directly onto `/home/projects/ros2_ws/src/` of the ucsdrobocar docker image. 
- This is easiest if you want just test, but will be more annoying when dealing with github permissions when pushing changes

**Option 2**: Clone locally (anywhere), then mount the local project onto the container.
- This "mounts" the project itself from the local file system to the docker image so that you can access/edit the project within robocar but commit/push changes on your local fs.
- If you are using the `robocar_docker` shell command, add this line:
```bash
--volume="{path to this project on local}:/home/projects/ros2_ws/src/delivery_package" \
-e PYTHONUNBUFFERED=1 \
````

after the other `--volume()` flag.

#### 2\. Modify UCSDRobocarhub (Integration)

Add the following to `src/ucsd_robocar_hub2/ucsd_robocar_nav2_pkg/config/node_pkg_locations_ucsd.yaml`

```yaml
delivery_mission: ['auto_delivery_pkg', 'mission.launch.py']
```

Add package as a config option in `src/ucsd_robocar_hub2/ucsd_robocar_nav2_pkg/config/node_config.yaml`

```yaml
delivery_mission: 1  # Delivery Mission (Add this new line)
```

#### 3\. Install dependencies

  - `depthai>=2.28.0.0` (Required for OAK-D Yolo)
  - `pupil-apriltags` (Required for CPU-based Tag Detection)

<!-- end list -->

```bash
pip install -r requirements.txt
```

### Usage:

#### Configuration

You can toggle visualizations (GUI windows) for headless operation in `config/visualization.yaml`:

```yaml
oak_perception_node:
  ros__parameters:
    show_vis: false # Set to true for debugging
```

#### Run ONLY this package (Standalone Test):

```bash
source_ros2
build_ros2
# Launches Mission Controller + Perception Nodes + Controllers
ros2 launch auto_delivery_pkg mission.launch.py
```

#### Run integration with UCSDRobocar:

```bash
source_ros2
build_ros2
ros2 launch ucsd_robocar_nav2_pkg all_nodes.launch.py
```

> **Note:** Make sure to enable `delivery_mission` in the `node_config.yaml` of the hub package.

```