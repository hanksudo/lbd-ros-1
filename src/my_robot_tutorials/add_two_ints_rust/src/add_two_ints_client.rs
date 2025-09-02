mod common;
use log::info;
use rosrust;
use std::time;

fn main() {
    common::setup_logger();
    rosrust::init("add_two_ints_client");

    rosrust::wait_for_service("/add_two_ints", Some(time::Duration::from_secs(10)))
        .expect("Service not available");

    let client = rosrust::client::<rosrust_msg::rospy_tutorials::AddTwoInts>("/add_two_ints")
        .expect("Failed to create client");

    match client.req(&rosrust_msg::rospy_tutorials::AddTwoIntsReq { a: 5, b: 6 }) {
        Ok(Ok(response)) => info!("Returned sum is {}", response.sum),
        Ok(Err(e)) => {
            info!("Service returned an error: {:?}", e);
        }
        Err(e) => info!("Failed to call service: {:?}", e),
    }
}
