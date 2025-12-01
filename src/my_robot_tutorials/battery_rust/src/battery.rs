use rosrust;
use std::time::Duration;

use battery_rust::msg::my_robot_msgs::{SetLed, SetLedReq};

#[derive(Debug, Clone, Copy)]
enum BatteryState {
    Empty,
    Full,
}

impl BatteryState {
    fn led_state(self) -> i64 {
        match self {
            BatteryState::Empty => 1, // LED on
            BatteryState::Full => 0,  // LED off
        }
    }

    fn message(self) -> &'static str {
        match self {
            BatteryState::Empty => "the battery is empty !",
            BatteryState::Full => "the battery is now full",
        }
    }
}

fn main() {
    env_logger::Builder::from_default_env()
        .filter_level(log::LevelFilter::Info)
        .init();
    
    rosrust::init("battery");

    run_battery_simulation();
}

fn run_battery_simulation() {
    while rosrust::is_ok() {
        rosrust::sleep(Duration::from_secs(7).into());
        update_battery_state(BatteryState::Empty); 

        rosrust::sleep(Duration::from_secs(3).into());
        update_battery_state(BatteryState::Full); 
    }
}

fn update_battery_state(state: BatteryState) {
    rosrust::ros_info!("{}", state.message());
    set_led(state);
}

fn set_led(battery_state: BatteryState) {
    let service_name: &str = "/set_led";

    if let Err(_err) = rosrust::wait_for_service(service_name, Some(Duration::from_secs(10))) {
        rosrust::ros_err!("Service {} not found/ready", service_name);
        return;
    }

    let client = rosrust::client::<SetLed>(service_name).unwrap();

    let request = SetLedReq {
        led_number: 1,
        state: battery_state.led_state(),
    };

    match client.req(&request) {
        Ok(res) => {
            rosrust::ros_info!("Set LED success : {}", res.unwrap().success);
        }
        Err(err) => {
            rosrust::ros_err!("Service call failed: {}", err);
        }
    }
}