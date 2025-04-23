use clap::Parser;
use kos::daemon::kos_runtime;
use kos_kbot::KbotPlatform;

/// CLI Args (For Enabling Services)
#[derive(Parser, Debug)]
#[command(name = "kbot", version, about = "Run KBot platform services")]
struct Args {
    /// Enable IMU service
    #[arg(long)]
    imu: bool,

    /// Enable actuator + powerboard services
    #[arg(long)]
    actuators: bool,
}

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    // // Create file appender
    // let file = File::create("actuator.log")?;
    // let file_writer = std::sync::Mutex::new(file);

    // // Create stdout layer
    // let stdout_layer = tracing_subscriber::fmt::layer()
    //     .with_writer(std::io::stdout);

    // // Create file layer
    // let file_layer = tracing_subscriber::fmt::layer()
    //     .with_writer(file_writer.with_max_level(tracing::Level::DEBUG))
    //     .with_filter(tracing_subscriber::filter::filter_fn(|metadata| {
    //         // Only log messages from the robstride crate
    //         metadata.target().starts_with("robstride")
    //     }));

    // // Initialize subscriber with both layers
    // tracing_subscriber::registry()
    //     .with(stdout_layer)
    //     .with(file_layer)
    //     .init();

    let args = Args::parse();

    // Default to both (actuators + imu) if no flags passed
    let enable_imu = args.imu || (!args.imu && !args.actuators);
    let enable_actuators = args.actuators || (!args.imu && !args.actuators);

    let mut platform = KbotPlatform::new();
    platform.set_feature_flags(enable_imu, enable_actuators);

    kos_runtime(Box::new(platform)).await.map_err(|e| {
        eprintln!("Runtime error: {}", e);
        e
    })?;

    Ok(())
}
