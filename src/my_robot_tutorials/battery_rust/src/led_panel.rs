use rosrust;
use std::sync::{Arc, Mutex};

const NUM_LEDS: usize = 3;

type LedStates = Arc<Mutex<Vec<i64>>>;

use battery_rust::msg::my_robot_msgs::{SetLed, SetLedReq, SetLedRes};

fn main() {
    env_logger::Builder::from_default_env()
        .filter_level(log::LevelFilter::Info)
        .init();

    rosrust::init("led_panel");

    // shared LED state (3 LEDs, initial off)
    let led_states = Arc::new(Mutex::new(vec![0; NUM_LEDS]));

    let _service = start_led_service(led_states.clone());
    run_status_loop(led_states);
}

fn start_led_service(led_states: LedStates) -> rosrust::Service {
    rosrust::service::<SetLed, _>(
        "/set_led",
        move |req| {
            Ok(handle_set_led_request(&led_states, req))
        },
    )
    .unwrap()
}

fn handle_set_led_request(led_states: &LedStates, req: SetLedReq) -> SetLedRes {
    let led_number = req.led_number;
    let state = req.state;

    if !is_valid_request(led_number, state) {
        return SetLedRes { success: false };
    }

    let mut leds = led_states.lock().unwrap();
    let index = (led_number - 1) as usize;

    leds[index] = state;
    rosrust::ros_info!("Set LED {} -> {}", led_number, state);

    SetLedRes { success: true }
}

fn is_valid_request(led_number: i64, state: i64) -> bool {
    led_number >= 1
        && led_number <= NUM_LEDS as i64
        && (state == 0 || state == 1)
}


fn run_status_loop(led_states: LedStates) {
    let rate = rosrust::rate(10.0);

    while rosrust::is_ok() {
        let leds = led_states.lock().unwrap();
        rosrust::ros_info!("{:?}", *leds);
        drop(leds); // Explicitly release lock before sleeping
        rate.sleep();
    }
}