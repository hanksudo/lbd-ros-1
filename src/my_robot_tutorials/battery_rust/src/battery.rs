use rosrust;
use std::time::Duration;

use battery_rust::msg::my_robot_msgs::{SetLed, SetLedReq};

fn main() {
    // Initialize logger and node
    env_logger::Builder::from_default_env()
        .filter_level(log::LevelFilter::Info)
        .init();
    
    rosrust::init("battery");

    while rosrust::is_ok() {
        rosrust::sleep(Duration::from_secs(7).into());
        rosrust::ros_info!("the battery is empty !");
        set_led(true); 

        rosrust::sleep(Duration::from_secs(3).into());
        rosrust::ros_info!("the battery is now full");
        set_led(false);
    }
}

/// Helper function to call the service
fn set_led(is_empty: bool) {
    let service_name = "/set_led";

    if let Err(_err) = rosrust::wait_for_service(service_name, Some(Duration::from_secs(2))) {
        rosrust::ros_err!("Service {} not found/ready", service_name);
        return;
    }

    // 2. Create the client
    let client = rosrust::client::<SetLed>(service_name).unwrap();

    // 3. Map logic: Empty = 1 (On), Full = 0 (Off)
    let state_val = if is_empty { 1 } else { 0 };

    let request = SetLedReq {
        led_number: 1,
        state: state_val,
    };

    // 4. Send the request
    match client.req(&request) {
        Ok(res) => {
            rosrust::ros_info!("Set led success flag : {}", res.unwrap().success);
        }
        Err(err) => {
            rosrust::ros_err!("Service call failed: {}", err);
        }
    }
}