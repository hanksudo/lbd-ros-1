use rosrust;
use rosrust_msg::std_msgs::Int64;
use std::sync::Arc;
use std::sync::atomic::{AtomicUsize, Ordering};
use log::info;

fn main() {
    env_logger::init();

    rosrust::init("number_counter");

    let counter = Arc::new(AtomicUsize::new(0));
    let c = counter.clone();

    let _sub = rosrust::subscribe("/number", 10, move |msg: Int64| {
        let n = c.fetch_add(1, Ordering::SeqCst) + 1;
        info!("Received #{}: {}", n, msg.data);
    })
    .expect("Failed to create subscriber");

    info!("number_counter ready");
    rosrust::spin();
}
