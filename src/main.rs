use kos::daemon::kos_runtime;
use kos_kbot::KbotPlatform;

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

    let platform = Box::new(KbotPlatform::new());
    kos_runtime(platform).await.map_err(|e| {
        eprintln!("Runtime error: {}", e);
        e
    })?;
    Ok(())
}
