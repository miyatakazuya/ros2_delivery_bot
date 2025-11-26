## Summary: 
ok so we need to test if a node we write can get the car moving. 

## Background:
Before starting, make sure you understand roughly what each package does in the UCSDRobocar Framework. (Section 2 on the google doc)

- The component that is most important in this case is the `vesc_twist_node` in the actuator package. TLDR there is a topic called `cmd_vel` that the node is subscribed to. By publishing throttle/steering commands to this topic, we can control the car.
- So essentially what we need to do is test if we can actually get the car to throttle and steer before we can program the backup sequence and whatnot. 

> [Read this README ](https://gitlab.com/ucsd_robocar2/ucsd_robocar_actuator2_pkg) on the actuator package to understand the specifics

## Setup
Ok before u do anything, get comfortable with the docker/ros2 stuff. In essense, I organized our custom ros2 package so that it 'mounts' to our robocar container. In other words, it copies itself to a directory within the container `smth/ros2_ws/src/delivery_smth_iforgotname`

So in order to work on the project, start the existing container and then open a new terminal session. 

Helpful commands:
```
docker ps -a        # Lists all containers including stopped ones
docker start <container name>
docker exec -it <container name> bash
```
> I think the one we are using right now is called delivery_robot or something like that.

Once we are in the container, always run `source_ros2`, which sets us up in the ros2 environment.

Then, follow the section called "2. Modify UCSDRobocarhub" in the `README.md` in this folder.

> **What is this doing?**: To put it briefly, the overall ros2 project is comprised of 1. UCSDRobocar and 2. Our RO2 Package. UCSDRobocar makes things easy because it already has the support to launch/control parts of the car as we desire (ex. VESC with cmd_vel). What we are doing here is adding our custom ROS2 package as an extension of the UCSDRobocar project, so that we can run both at the same time and reap the benefits of UCSDRobocar automatically setting up yucky vesc/sesnsor related stuff while we focus on the implementation of other project. 

Ok so once we have followed the steps on the README, the custom package should be able to run via UCSDRobocar. 

Let's test this by:
- Set all the nodes to 0 (off) except `delivery_mission` in `node_config.yaml`
- Set one node to be on in `node_config.yaml` on the the custom ros2 package (i realize i shoulda named it different cuz its confusing)

### BUILD BUILD
-> run `build_ros` EVERYTIME you make a change on any of the files. TS annoying but it is how it works.

To launch:
```
ros2 launch ucsd_robocar_nav2_pkg all_nodes.launch.py
```
We should see our node running. This is how we run our package through UCSDRobocar.

## Actually coding
ok idrc if u vibe code but good experience if u dont.

For this task, we should try and modify `mission_controller.py` so that the car moves for a short amount of time, then stops. and then moves again in a cycle.

Again this is done through publishing to the `cmd_vel` topic. 
Good luck :)

> If anyone gets lost, I found a [good example](https://github.com/UCSD-ECEMAE-148/final-project-repository-su25-team4/blob/main/final_project_batmobile/final_project_batmobile/cmd_arbiter.py#L31) from a past team that does the same thing (publishing to cmd_vel)