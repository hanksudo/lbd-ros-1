# ROS 1 noetic

- Python
- C++
- Rust

## Usage

```bash
cd ~/catkin_ws
catkin_make
roscore
```

## add_two_ints

```bash
# C++

rosrun my_robot_tutorials add_two_ints_server
rosrun my_robot_tutorials add_two_ints_client

# Python

rosrun my_robot_tutorials add_two_ints_server.py
rosrun my_robot_tutorials add_two_ints_client.py

# Rust

cd src/my_robot_tutorials/add_two_ints_rust/
cargo run --bin add_two_ints_client
cargo run --bin add_two_ints_server

```

## Subscriber / Publisher

```bash
# C++
rosrun my_robot_tutorials cplusplus_publisher_node
rosrun my_robot_tutorials cplusplus_subscriber_node

# Python
rosrun my_robot_tutorials first_publisher.py
rosrun my_robot_tutorials first_subscriber.py
```

## Number counter

```bash
# Python
rosrun my_robot_tutorials number_counter.py
rosrun my_robot_tutorials number_publisher.py

# Rust
cd src/my_robot_tutorials/number_tools_rust
cargo run --bin number_counter
cargo run --bin number_publisher

# check result
rostopic echo /number_count

# reset number
rosservice call /reset_counter "data: true"
```

## Custom message

```bash
# Need to source again if anything add to catkin package
source devel/setup.bash

# Python
rosrun rosrun my_robot_tutorials hw_status_publisher.py

rosmsg show my_robot_msgs/HardwareStatus
rossrv show my_robot_msgs/ComputerDiskArea
```

## Led panel (custom srv)

```bash
# Python
rosrun rosrun my_robot_tutorials led_panel.py

rosservice call /set_led "led_number: 2
state: 1"

rosservice call /set_led "led_number: 1
state: 1"
```