use rosrust;
use rosrust_msg::std_msgs::Int64;
use rosrust_msg::std_srvs::SetBool;
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

    let _subscriber = rosrust::subscribe("/number", 10, move |msg: Int64| {
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
    .unwrap();

    let _service = rosrust::service::<SetBool, _>(
        "reset_counter",
        move |req| {
            if req.data {
                counter.store(0, Ordering::SeqCst);
                info!("Counter reset to 0");
                Ok(rosrust_msg::std_srvs::SetBoolRes {
                    success: true,
                    message: "Counter reset successfully".to_string(),
                })
            } else {
                Ok(rosrust_msg::std_srvs::SetBoolRes {
                    success: false,
                    message: "Reset request was false".to_string(),
                })
            }
        },
    )
    .unwrap();

    info!("number_counter ready");
    rosrust::spin();
}
