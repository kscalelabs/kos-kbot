pub mod bno055;
pub mod hexmove;
pub mod hiwonder;

#[cfg(not(any(feature = "imu_hiwonder", feature = "imu_hexmove", feature = "imu_bno055")))]
compile_error!(
    "An IMU feature flag must be enabled. Choose one of: imu_hiwonder, imu_hexmove, imu_bno055"
);

#[cfg(any(
    all(feature = "imu_hiwonder", feature = "imu_hexmove"),
    all(feature = "imu_hiwonder", feature = "imu_bno055"),
    all(feature = "imu_hexmove", feature = "imu_bno055"),
))]

compile_error!(
    "Only one IMU feature flag (imu_hiwonder, imu_hexmove, imu_bno055) can be enabled at a time."
);
