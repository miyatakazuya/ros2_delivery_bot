## Repository for Package Delivery Bot (Team14)

### Setup:
**Option 1**: Clone directly onto `/home/projects/ros2_ws/src/` of the ucsdrobocar docker image. 
- This is easiest if you want just test, but will be more annoying when dealing with github permissions when pushing changes

**Option2**: Clone locally (anywhere), then mount the local project onto the container.
- This "mounts" the project itself from the local file system to the docker image so that you can access/edit the project within robocar but commit/push changes on your local fs.
- If you are using the `robocar_docker` shell command, add this line:
```
--volume="{path to this project on local}:/home/projects/ros2_ws/src/delivery_package" \
```
after the other `--volume()` flag. 

### Usage:

I only wrote the controller draft so far:
```bash
source_ros2
build_ros2
ros2 run auto_delivery_pkg mission_controller
```
