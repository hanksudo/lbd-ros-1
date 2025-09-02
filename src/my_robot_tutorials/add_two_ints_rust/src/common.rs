use env_logger::Builder;
use std::io::Write;

pub fn setup_logger() {
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
