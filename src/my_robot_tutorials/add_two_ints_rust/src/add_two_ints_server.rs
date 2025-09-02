mod common;
use rosrust;

fn main() {
    common::setup_logger();

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
