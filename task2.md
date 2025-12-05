## Summary: 
Add activation/deactivation for nodes

## Background:

If you havent setup our package yet, do so in the previous task

## Actually coding

For certain nodes, we want to be able to enable/disable them at will such that we can control what programs are running at a given time, saving us compute resources.
The 'root' or master node is `mission_controller.py`, which is our FSM (Finite State Machine) which controls our entire mission sequence. 
- For example, in the transition from `SEARCH` to `PARK`, we would want to disable the YOLO program so that the camera resources can be saved.
- The suggested way for this to be done is to create a new ROS2 topic, in which our master node is the sole publisher and other nodes subscribe to.
- This topic will be how the nodes listen to any commands from the controller on whether or not they should enable/disable at a given time.

Therefore we need to implement:
- some `wait()` or `idle()` function in which the node listens to the topic for an 'Activation' Command
- when it does get a command to activate, it should run the main control loop (ex. for yolo `init_depthai()` then `timer_callback()`).
- This will allow us to toggle nodes at will.



> If anyone gets lost, I found a [good example](https://github.com/UCSD-ECEMAE-148/final-project-repository-su25-team4/blob/main/final_project_batmobile/final_project_batmobile/cmd_arbiter.py#L31) from a past team that does the same thing (publishing to cmd_vel)