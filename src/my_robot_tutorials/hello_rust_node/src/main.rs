use rosrust;
use env_logger::Builder;
use log::info;
use std::io::Write;

fn main() {
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
    rosrust::init("my_first_rust_node");

    rosrust::ros_info!("Node has been started");

    let rate = rosrust::rate(10.0);

    while rosrust::is_ok() {
        info!("Hello from Rust node");
        rate.sleep();
    }
}
