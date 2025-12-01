// src/lib.rs

// Make the module public so binaries can access it
pub mod msg {
    rosrust::rosmsg_include!(
        my_robot_msgs/SetLed
    );
}