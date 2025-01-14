use kos::daemon::kos_runtime;
use kos_kbot::KbotPlatform;

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    let platform = Box::new(KbotPlatform::new());
    kos_runtime(platform).await.map_err(|e| {
        eprintln!("Runtime error: {}", e);
        e
    })?;
    Ok(())
}
