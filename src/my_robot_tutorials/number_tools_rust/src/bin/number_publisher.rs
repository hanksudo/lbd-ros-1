use rosrust;
use rosrust_msg::std_msgs::Int64;
use log::{error, info};

fn main() {
    env_logger::init();

    rosrust::init("number_publisher_rust");

    let publisher = rosrust::publish::<Int64>("/number", 10).expect("Failed to create publisher");
    let rate = rosrust::rate(1.0);

    let mut value: i64 = 0;
    info!("number_publisher_rust started");

    while rosrust::is_ok() {
        let mut msg = Int64::default();
        msg.data = value;

        if let Err(e) = publisher.send(msg) {
            error!("Failed to publish: {:?}", e);
        } else {
            info!("Published: {}", value);
        }

        value = value.wrapping_add(1);
        rate.sleep();
    }
}
