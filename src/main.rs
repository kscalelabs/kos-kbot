use kos::daemon::kos_runtime;
use kos_kbot::KbotPlatform;
// use tracing_subscriber::prelude::*;
// use tracing_subscriber::filter::EnvFilter;

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    // let console_layer = console_subscriber::spawn();

    // let stdout_layer = tracing_subscriber::fmt::layer()
    //     .with_writer(std::io::stdout)
    //     .with_filter(
    //         EnvFilter::from_default_env()
    //             .add_directive("h2=error".parse().unwrap())
    //             .add_directive("grpc=error".parse().unwrap())
    //             .add_directive("rumqttc=error".parse().unwrap())
    //             .add_directive("kos::telemetry=error".parse().unwrap())
    //             .add_directive("polling=error".parse().unwrap())
    //             .add_directive("async_io=error".parse().unwrap())
    //             .add_directive("krec=error".parse().unwrap()),
    //     );


//     tracing_subscriber::registry()
//     .with(console_layer)
//     .with(stdout_layer)
// //  .with(..potential additional layer..)
//     .init();

    let platform = Box::new(KbotPlatform::new());
    kos_runtime(platform).await.map_err(|e| {
        eprintln!("Runtime error: {}", e);
        e
    })?;
    Ok(())
}
