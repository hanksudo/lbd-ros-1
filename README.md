# ROS 1 noetic

- Python
- C++
- Rust

## Usage

```bash
cd ~/catkin_ws
catkin_make
roscore

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
