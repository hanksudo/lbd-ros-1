use rosrust;
use rosrust_msg::std_msgs::Int64;
use log::{error, info};
use std::process;

fn main() {
    env_logger::Builder::from_default_env()
        .filter_level(log::LevelFilter::Info)
        .init();

    let node_name = format!("number_publisher_params_{}", process::id());
    rosrust::init(&node_name);

    let publisher = rosrust::publish::<Int64>("/number", 10).expect("Failed to create publisher");
    let rate = rosrust::rate(1.0);

    info!("number_publisher started");

    // Read `number_to_publish` param (default to 2)
    let number_to_publish: i64 = match rosrust::param("~number_to_publish") {
        Some(param) => match param.get() {
            Ok(v) => v,
            Err(e) => {
                error!("Failed to parase param `number_to_publish`: {:?}, using default 2", e);
                2
            }
        },
        None => {
            info!("Param `number_to_publish` not set, using default 2");
            2
        }
    };
    info!("number_to_publish = {}", number_to_publish);

    if let Err(e) = rosrust::param("/another_param").unwrap().set(&"Hello".to_string()) {
        error!("Failed to set param /another_param`: {:?}", e);
    } else {
        info!("Set param `/another_param` to 'Hello'");
    }

    if let Err(e) = rosrust::param("~another_private_param").unwrap().set(&"Hello".to_string()) {
        error!("Failed to set param ~another_private_param`: {:?}", e);
    } else {
        info!("Set param `~another_private_param` to 'Hello'");
    }

    while rosrust::is_ok() {
        let mut msg = Int64::default();
        msg.data = number_to_publish;
        let published_value = msg.data;

        if let Err(e) = publisher.send(msg) {
            error!("Failed to publish: {:?}", e);
        } else {
            info!("Published: {}", published_value);
        }
        rate.sleep();
    }
}
