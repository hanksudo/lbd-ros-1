use env_logger::Builder;
use rosrust;
use std::io::Write;

fn main() {
    setup_logger();

    rosrust::init("add_two_ints_server");
    let _service = rosrust::service::<rosrust_msg::rospy_tutorials::AddTwoInts, _>(
        "add_two_ints",
        move |req| {
            let sum = req.a + req.b;
            rosrust::ros_info!("Received request: {} + {} = {}", req.a, req.b, sum);

            Ok(rosrust_msg::rospy_tutorials::AddTwoIntsRes { sum })
        },
    )
    .expect("Failed to create service");

    rosrust::ros_info!("Add Two Ints server ready");

    rosrust::spin();
}

fn setup_logger() {
    Builder::new()
        .format(|buf, record| {
            let timestamp = std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .unwrap()
                .as_secs_f64();
            writeln!(buf, "[INFO] [{:.9}]: {}", timestamp, record.args())
        })
        .filter_level(log::LevelFilter::Info)
        .init();
}
