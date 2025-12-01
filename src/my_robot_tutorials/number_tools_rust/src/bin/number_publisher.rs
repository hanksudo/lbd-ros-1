use rosrust;
use rosrust_msg::std_msgs::Int64;
use log::{error, info};
use std::process;

fn main() {
    env_logger::Builder::from_default_env()
        .filter_level(log::LevelFilter::Info)
        .init();

    let node_name = format!("number_publisher_{}", process::id());
    rosrust::init(&node_name);

    let publisher = rosrust::publish::<Int64>("/number", 10).expect("Failed to create publisher");
    let rate = rosrust::rate(1.0);

    info!("number_publisher started");

    while rosrust::is_ok() {
        let mut msg = Int64::default();
        msg.data = 2;
        let published_value = msg.data;

        if let Err(e) = publisher.send(msg) {
            error!("Failed to publish: {:?}", e);
        } else {
            info!("Published: {}", published_value);
        }
        rate.sleep();
    }
}
