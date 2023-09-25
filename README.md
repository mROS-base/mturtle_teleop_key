# mturtle_teleop_key

turtle_teleop_key for mturtlesim (mROS 2 dedicated version of turtlesim)

original code is from [ros/tutorials](https://github.com/ros/ros_tutorials/blob/humble/turtlesim/tutorials/teleop_turtle_key.cpp)

## Build

```
$ mkdir -p your-workspace/src
$ cd your-workspace
$ git clone https://github.com/mROS-base/mturtle_teleop_key.git src/mturtle_teleop_key
$ source /opt/ros/humble/setup.bash
$ colcon build
$ source install/local_setup.bash
```

## Run

```
$ ros2 run mturtle_teleop_key mturtle_teleop_key
Reading from keyboard
---------------------------
O|L|.|,|M|J|U|I keys to move around.
'K' to stop the turtle.
W|X to increase/decrease maximum speeds by 10%.
E|C to increase/decrease linear speed by 10%.
R|V to increase/decrease angular speed by 10%.
'Q' to quit.
```

          i
          ^
    u <-  |  -> o
        \ | /
         +-+
       +-+ +-+
       |     |
       +-----+
        / | \
    m <-  |  -> .
          v
          ,

- j: rotate left
- l: rotate right
- k: stop

## Topics

| Topic | Type | Direcetion | Description |
| --- | --- | --- | --- |
| `cmd_vel` | `geometry_msgs/Twist` | pub | Velocity command. |

## Parameters

| Parameter | Type | Description |
| --- | --- | --- |
| `scale_angular` | double | Scaling factor for angular speed (default 0.25). |
| `scale_linear` | double | Scaling factor for linear speed (default 0.25). |
