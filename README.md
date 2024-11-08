# ROS2 Studies

My personal ROS2 studies, from pubsub until the end!

Using pubsub as example:

To check for missing dependencies

`rosdep install -i --from-path src --rosdistro humble -y`

To build all packages

`colcon build`

To source the newly built packages

`source install/setup.bash`

To run the publisher and subscriber (specific to the pubsub now)

`ros2 run cpp_pubsub talker`

`ros2 run cpp_pubsub listener`
