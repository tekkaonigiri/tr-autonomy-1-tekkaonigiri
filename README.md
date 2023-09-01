# TR-CV-1
First Training Module for TR CV Recruits

## Task Overview

In this module you will be given regularly measured position and velocity of a target to track. However, these measurements will only be published every 1.5 seconds, so it is your job to estimate the intermediate positions.

The below video shows what the tracker output should look like. The target is represented by `//` and the estimate is represented by `\\`. When they coincide, they are represented by `╳╳`

https://github.com/Triton-Robotics-Training/TR-CV-1/assets/33632547/c09eebcf-4f47-490b-9f65-17ddb58e281f

## Getting Started

Don't use conda. If it is activated in your shell use conda deactivate until there is no environment specified at the left. Then `pip install numpy`

Make sure you [have colcon installed](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Colcon-Tutorial.html#:~:text=sudo%20apt%20install%20python3%2Dcolcon%2Dcommon%2Dextensions).

*Note:*	

Next you have to build the packages. Source the setup file from your ros installation (typically in `/opt/ros/humble/setup.bash`) in the shell you are building in.
Then at the root of this git repo, run `colcon build`. Finally, run `source install/setup.bash`. Those last 2 commands are run every time you want to rebuild your solution.

Finally, you can run the *spinnything* node which makes the target visualization, publishes the tracking data, and listens for the predicted position.
```bash
ros2 run spinnything spinnything
```

The output should look like the following. Notice the predicted position is not moving, 

https://github.com/Triton-Robotics-Training/TR-CV-1/assets/33632547/c80cfce8-6c66-4b9f-a2fd-f210406cc211

## Architecture

The point of this assignment is to get used to writing ROS2 publishers and subscribers in a non-trivial example.

When running spinnything, there is a spinnything node:
```bash
~/Documents/TR-CV-1$ ros2 node list
/spinnything
```

We can also see what topics there are:
```bash
~/Documents/TR-CV-1$ ros2 topic list
/measuredpos
/measuredvel
/parameter_events
/predictedpos
/rosout
```
The topics published by spinnything are `/measuredpos` and `/measuredvel`. We can check how often they are published:
```bash
~/Documents/TR-CV-1$ ros2 topic hz /measuredpos
average rate: 0.663
	min: 1.501s max: 1.517s std dev: 0.00821s window: 2
```
It is up to you to publish `/predictedpos` much faster than this:
```
~/Documents/TR-CV-1$ ros2 topic hz /predictedpos
average rate: 2000.087
	min: 0.000s max: 0.001s std dev: 0.00003s window: 2002
```

To do this, you write a node in the `your_solution` package

```mermaid
graph TD;
sol("yoursolution")
tng("spinnything")
sol == /predictedpos ==> tng;
tng == /measuredpos ==> sol;
tng == /measuredvel ==> sol;
```

The messages are all of type `ArrayMsg = std_msgs::msg::Float64MultiArray`, the 0th entry is the x coordinate, the 1st entry is the y coordinate.

In order to predict the location of the target: use the following algorithm:

# $\vec{x_p} = \vec{x} + \Delta t \vec{v}$

Where $x_p$ is the predicted position vector, $x$ and $v$ are previously measured position and velocity, and $\Delta t$ is the change in time since the measurement was received by your node.

## What you need to do:

### Part 1 (Optional)

Create a node (the code is set up for you in `spin_slow_update.cpp` and `spin_slow_update.h`) that takes the measured position and immediately republishes it to the predicted postion. The result should look like this:

https://github.com/Triton-Robotics-Training/TR-CV-1/assets/33632547/2b949c8f-c465-4124-879e-83cc3d86424f

### Part 2 (Required)

Create a node (the code is set up for you in `spin_sol.cpp` and `spin_sol.h`) that predicts the position of the target (using the algorithm above) and publishes it more frequently than the measurements. It should use a rclcpp wall timer callback to do this. The final product should look like this:

https://github.com/Triton-Robotics-Training/TR-CV-1/assets/33632547/c09eebcf-4f47-490b-9f65-17ddb58e281f

### Submission Directions

Commit your completed code for Part 2 (and optionally Part 1) to this github repo, and submit it if that's possible (IDK how GH classroom works)
