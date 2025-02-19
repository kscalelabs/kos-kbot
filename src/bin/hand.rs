use inspirehand::Hand;

#[tokio::main]
async fn main() -> Result<()> {
    tracing_subscriber::fmt::init();

    let hand = Hand::new("/dev/ttyUSB0", 115200)?;

    hand.start().await?;

    hand.set_finger_position(0, 100).await?;
    hand.set_finger_position(1, 200).await?;

    let pos = hand.get_finger_position(0).await?;
    println!("Finger 0 position: {}", pos);

    tokio::time::sleep(Duration::from_secs(5)).await; // Run for a few seconds

    Ok(())
}
