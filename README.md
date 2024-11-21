# ROS2 Studies

My personal ROS2 studies, from pubsub until the end!

Using [ROS2 Humble](https://docs.ros.org/en/humble/index.html), I'm just learning how to use it by following its tutorials and implementing some small projects.

## How to run

Using pubsub as example:

To check for missing dependencies (at root)  
`rosdep install -i --from-path src --rosdistro humble -y`

To build all packages (at root)  
`colcon build`

To source the newly built packages (at root)  
`source install/setup.bash`

To run the publisher and subscriber packages, for example  
`ros2 run cpp_pubsub talker`  
`ros2 run cpp_pubsub listener`

## What is implemented so far

### Subscriber/Publisher

The basic tutorial project for implementing [ROS2 topics](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Topics/Understanding-ROS2-Topics.html). It just creates a talker and a listener that talk to each other. It uses a future project - the tutorial interfaces.

### Service/Client

The basic tutorial project for implementing [ROS2 services](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Services/Understanding-ROS2-Services.html). As simple as the last, also using tutorial interfaces.

### Interfaces

Another basic tutorial that explains how to implement [custom interfaces in ROS2](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Single-Package-Define-And-Use-Interface.html) to be used with other packages, be it packages that use topics, services or even actions.

### Double Turtle

A small project with the aim to make a turtlesim go in circles and another in squares, but instead of having it so simple, I chose to use actions with topics in the square turtle.  
First, to use it at all, you need to run the turtlesim multisim to have the turtles up in two terminals. The command is  
`ros2 launch turtlesim multisim.launch.py`  
and then you run the package like the other packages.

The first turtlesim is only talked to with topics in the `turtlesim_twist_pub.cpp` code file, receiving an endless publish stream of "go ahead with a slight angle" through `cmd_vel`.

The second turtlesim takes on two communications in the `turtlesim_rotate_action_cli` code file. First, it gets an action goal sent to it through `rotate_absolute` to spin in place and then it receives a `cmd_vel` like the other turtlesim to simply go ahead for some time. I was able to get these two communications to work one after the other by cancelling the wall timer in the beginning of the `send_goal()` method that begins the action, to then call the `timer_callback()` method (the one that calls the topic) manually at the end of the action's result callback, to then recreate the wall timer at the end of it, which calls the action again. I could have simply called two well timed topics in `cmd_vel`, but I took this opportunity to learn actions too.
