## Repository for Package Delivery Bot (Team14)

### Setup:

#### 1. Install the package

**Option 1**: Clone directly onto `/home/projects/ros2_ws/src/` of the ucsdrobocar docker image. 
- This is easiest if you want just test, but will be more annoying when dealing with github permissions when pushing changes

**Option2**: Clone locally (anywhere), then mount the local project onto the container.
- This "mounts" the project itself from the local file system to the docker image so that you can access/edit the project within robocar but commit/push changes on your local fs.
- If you are using the `robocar_docker` shell command, add this line:
```
--volume="{path to this project on local}:/home/projects/ros2_ws/src/delivery_package" \
```
after the other `--volume()` flag. 

#### 2. Modify UCSDRobocarhub

Add the following to `src/ucsd_robocar_hub2/ucsd_robocar_nav2_pkg/config/node_pkg_locations_ucsd.yaml`
```
delivery_system: ['auto_delivery_pkg', 'mission.launch.py']
```

Add package as a config option in `src/ucsd_robocar_hub2/ucsd_robocar_nav2_pkg/config/node_config.yaml`
```
delivery_mission: 1  # Delivery Mission (Add this new line)
```

### Usage:

Run ONLY this package:
```bash
source_ros2
build_ros2
ros2 run auto_delivery_pkg mission_controller
```

Run integration with UCSDRobocar
```
source_ros2
build_ros2
ros2 launch ucsd_robocar_nav2_pkg all_nodes.launch.py
```
> Make sure to enable `delivery_mission` in the `node_config.yaml`
