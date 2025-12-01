use rosrust;
use rosrust_msg::std_msgs::Int64;
use std::sync::Arc;
use std::sync::atomic::{AtomicI64, Ordering};
use log::{error, info};

fn main() {
    env_logger::Builder::from_default_env()
        .filter_level(log::LevelFilter::Info)
        .init();

    rosrust::init("number_counter");

    // Publisher that will publish the sum
    let publisher = rosrust::publish::<Int64>("/number_count", 10)
        .expect("Failed to create publisher");

    // Atomic running sum (i64) shared with the subscriber callback.
    let counter = Arc::new(AtomicI64::new(0));
    let c = counter.clone();
    let p = publisher.clone();

    let _sub = rosrust::subscribe("/number", 10, move |msg: Int64| {
        // Add imcoming value to sum atomically and publish the new sum.
        let prev = c.fetch_add(msg.data, Ordering::SeqCst);
        let sum = prev.wrapping_add(msg.data);

        let mut out = Int64::default();
        out.data = sum;

        if let Err(e) = p.send(out) {
            error!("Failed to publish sum: {:?}", e);
        } else {
            info!("Received: {} -> published sum: {}", msg.data, sum);
        }
    })
    .expect("Failed to create subscriber");

    info!("number_counter ready");
    rosrust::spin();
}
