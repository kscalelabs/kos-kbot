use eyre::{eyre, Result};
use serialport::SerialPort;
use std::sync::Arc;
use tokio::sync::Mutex;
use tokio::time::{sleep, Duration};
use tracing::{error, instrument, trace};

const REGDICT: [(u16, &'static str); 10] = [
    (1000, "ID"),
    (1001, "baudrate"),
    (1004, "clearErr"),
    (1009, "forceClb"),
    (1486, "angleSet"),
    (1498, "forceSet"),
    (1522, "speedSet"),
    (1546, "angleAct"),
    (1582, "forceAct"),
    (1606, "errCode"),
];

pub struct Hand {
    serial_port: Arc<Mutex<Box<dyn SerialPort + Send>>>,
    finger_positions: Arc<Mutex<Vec<i32>>>,
    hand_id: u8,
}

impl Hand {
    pub fn new(port: &str, baudrate: u32, hand_id: u8) -> Result<Self> {
        let serial_port = serialport::new(port, baudrate)
            .open()
            .map_err(|e| eyre!("Failed to open serial port: {}", e))?;

        Ok(Self {
            serial_port: Arc::new(Mutex::new(serial_port)),
            finger_positions: Arc::new(Mutex::new(vec![-1; 6])), // Initialize all to -1 (no position set)
            hand_id,
        })
    }

    #[instrument(skip(self))]
    pub async fn set_finger_position(&self, finger: usize, position: i32) -> Result<()> {
        if finger >= 6 {
            return Err(eyre!("Invalid finger index. Must be between 0 and 5."));
        }

        let mut positions = self.finger_positions.lock().await;
        positions[finger] = position;

        trace!("Setting finger {} to position {}", finger, position);

        // Create an array with the current positions
        let mut values = [0; 6];
        for (i, &pos) in positions.iter().enumerate() {
            values[i] = pos;
        }

        // Use self.hand_id instead of hardcoded 1
        self.write_6(self.hand_id, "angleSet", &values).await
    }

    #[instrument(skip(self))]
    pub async fn get_finger_position(&self, finger: usize) -> Result<i32> {
        if finger >= 6 {
            return Err(eyre!("Invalid finger index. Must be between 0 and 5."));
        }

        let positions = self.finger_positions.lock().await;
        let position = positions[finger];

        trace!("Getting finger {} position: {}", finger, position);
        Ok(position)
    }

    async fn write_register(&self, id: u8, addr: u16, num: u8, values: &[u8]) -> Result<()> {
        let mut bytes = vec![0xEB, 0x90, id, num + 3, 0x12];
        bytes.push((addr & 0xFF) as u8);
        bytes.push(((addr >> 8) & 0xFF) as u8);
        bytes.extend_from_slice(values);

        // Calculate checksum with wrapping addition
        let checksum: u8 = bytes[2..].iter().fold(0u8, |acc, &x| acc.wrapping_add(x));
        bytes.push(checksum);

        let mut serial = self.serial_port.lock().await;
        serial.write_all(&bytes)?;
        sleep(Duration::from_millis(10)).await;

        // Clear response buffer
        let mut buf = [0u8; 128];
        serial.read(&mut buf)?;

        Ok(())
    }

    async fn read_register(&self, id: u8, addr: u16, num: u8) -> Result<Vec<u8>> {
        let mut bytes = vec![0xEB, 0x90, id, 0x04, 0x11];
        bytes.push((addr & 0xFF) as u8);
        bytes.push(((addr >> 8) & 0xFF) as u8);
        bytes.push(num);

        // Calculate checksum with wrapping addition
        let checksum: u8 = bytes[2..].iter().fold(0u8, |acc, &x| acc.wrapping_add(x));
        bytes.push(checksum);

        let mut serial = self.serial_port.lock().await;
        serial.write_all(&bytes)?;
        sleep(Duration::from_millis(10)).await;

        let mut buf = [0u8; 128];
        let n = serial.read(&mut buf)?;
        if n == 0 {
            return Ok(vec![]);
        }

        let data_len = (buf[3] & 0xFF) - 3;
        let mut values = Vec::with_capacity(data_len as usize);
        for i in 0..data_len {
            values.push(buf[7 + i as usize]);
        }

        Ok(values)
    }

    async fn write_6(&self, id: u8, reg: &str, values: &[i32; 6]) -> Result<()> {
        let addr = REGDICT
            .iter()
            .find(|(_, name)| *name == reg)
            .ok_or_else(|| eyre!("Invalid register name"))?
            .0;

        let mut val_reg = Vec::with_capacity(12);
        for &val in values {
            val_reg.push((val & 0xFF) as u8);
            val_reg.push(((val >> 8) & 0xFF) as u8);
        }

        self.write_register(id, addr, 12, &val_reg).await
    }

    async fn read_6(&self, id: u8, reg: &str) -> Result<Vec<i32>> {
        let addr = REGDICT
            .iter()
            .find(|(_, name)| *name == reg)
            .ok_or_else(|| eyre!("Invalid register name"))?
            .0;

        let values = self.read_register(id, addr, 12).await?;
        if values.len() < 12 {
            return Err(eyre!("No data received"));
        }

        let mut result = Vec::with_capacity(6);
        for i in 0..6 {
            let val = (values[2 * i] as i32) | ((values[2 * i + 1] as i32) << 8);
            result.push(val);
        }

        Ok(result)
    }

    async fn poll_finger_positions(&self) -> Result<()> {
        loop {
            // Use self.hand_id instead of hardcoded 1
            match self.read_6(self.hand_id, "angleAct").await {
                Ok(positions) => {
                    let mut stored_positions = self.finger_positions.lock().await;
                    for (i, &pos) in positions.iter().enumerate() {
                        stored_positions[i] = pos;
                    }
                }
                Err(e) => error!("Failed to read positions: {}", e),
            }
            sleep(Duration::from_secs_f32(0.05)).await; // Poll at 20Hz
        }
    }

    pub async fn start(self: Arc<Self>) -> Result<()> {
        let hand = self.clone();
        tokio::spawn(async move {
            if let Err(e) = hand.poll_finger_positions().await {
                error!("Error in polling finger positions: {}", e);
            }
        });
        Ok(())
    }
}