#!/usr/bin/env python3
import can
import time

def build_29bit_id(target_address, comm_type, reserved=0x00):
    """
    Utility function to build the 29-bit ID from:
      - target_address (lowest 8 bits)
      - comm_type (next 13 bits)
      - reserved (top 8 bits)
    """
    # bits 0..7   -> target_address
    # bits 8..20  -> comm_type
    # bits 21..28 -> reserved
    arbitration_id = (target_address & 0xFF) | ((comm_type & 0x1FFF) << 8) | ((reserved & 0xFF) << 21)
    return arbitration_id

def send_control_frame_enable_autoreport(bus):
    """
    Sends a 0x1001 Control Frame to enable auto-reporting on the power board.
    
    Byte layout (0..7):
        Byte0: 1/0 -> Fan
        Byte1: 1/0 -> Pre-charge
        Byte2: 1/0 -> Motor output
        Byte3: 1/0 -> Main control
        Byte4: 1/0 -> Restart
        Byte5: 1/0 -> Clear fault
        Byte6: 1/0 -> Auto-report data at 100ms intervals
        Byte7: 1/0 -> Reserved
    """
    target_address = 0xAA
    comm_type = 0x1001
    
    arb_id = build_29bit_id(target_address, comm_type, reserved=0x00)
    
    data = [0]*8
    
    # Set control bits
    data[0] = 0  # Fan off
    data[1] = 1  # Pre-charge on
    data[2] = 1  # Motor output on
    data[3] = 1  # Main control on
    data[4] = 0  # Restart off
    data[5] = 0  # Clear fault off
    data[6] = 1  # Auto-report on
    data[7] = 0  # Reserved
    
    msg = can.Message(
        arbitration_id=arb_id,
        data=data,
        is_extended_id=True
    )
    
    try:
        bus.send(msg)
        print("[INFO] Sent 0x1001 Control Frame: Enabled auto-reporting (Byte6=1).")
    except can.CanError:
        print("[ERROR] Failed to send the 0x1001 control frame!")

def parse_power_board_message(msg):
    """
    Parse incoming 29-bit CAN messages according to the power board protocol.
    """
    arbitration_id = msg.arbitration_id
    data = msg.data
    is_extended_id = msg.is_extended_id

    if not is_extended_id:
        print("[WARN] Received a standard 11-bit ID but expected 29-bit ID.")
        return

    # Extract fields from the 29-bit ID
    target_address = arbitration_id & 0xFF                 # bits [0..7]
    comm_type = (arbitration_id >> 8) & 0x1FFF             # bits [8..20]
    reserved = (arbitration_id >> 21) & 0xFF               # bits [21..28]

    # For debugging:
    # print(f"Received ID=0x{arbitration_id:X}, Target=0x{target_address:X}, CommType=0x{comm_type:X}, Reserved=0x{reserved:X}")

    if comm_type == 0x1003:
        parse_status_frame_1003(data)
    elif comm_type == 0x1004:
        parse_status_frame_1004(data)
    else:
        # We can handle other frames (0x1001, 0x1002) if needed, but here we focus on status frames.
        pass

def parse_status_frame_1003(data):
    """
    0x1003 Status Frame:
      Byte0-1: Battery Voltage (range 0~655.36 V)
      Byte2-3: Motor Voltage   (range 0~655.36 V)
      Byte4-5: Current         (range 0~655.36 A)
      Byte6-7: Fault Status (bitfield)
    """
    if len(data) < 8:
        print("[ERROR] 0x1003 frame data too short!")
        return

    battery_voltage_raw = (data[0] << 8) | (data[1])
    battery_voltage = battery_voltage_raw / 100.0

    motor_voltage_raw = (data[2] << 8) | (data[3])
    motor_voltage = motor_voltage_raw / 100.0

    current_raw = (data[4] << 8) | (data[5])
    current_value = current_raw / 100.0

    fault_raw = (data[6] << 8) | (data[7])
    # Extract bits:
    power_chip_oc = bool(fault_raw & (1 << 0))  # Over-current
    power_chip_ot = bool(fault_raw & (1 << 1))  # Over-temp
    power_chip_sc = bool(fault_raw & (1 << 2))  # Short-circuit
    sampling_oc   = bool(fault_raw & (1 << 3))
    vbus_ov       = bool(fault_raw & (1 << 4))
    vbus_uv       = bool(fault_raw & (1 << 5))
    vmbus_ov      = bool(fault_raw & (1 << 6))
    vmbus_uv      = bool(fault_raw & (1 << 7))

    print("=== 0x1003 Status Frame ===")
    print(f" Battery Voltage: {battery_voltage:.2f} V")
    print(f" Motor Voltage:   {motor_voltage:.2f} V")
    print(f" Current:         {current_value:.2f} A")
    print(f" Fault Status Raw: 0x{fault_raw:04X}")
    print(f"   Power Chip Over-Current: {power_chip_oc}")
    print(f"   Power Chip Over-Temp:    {power_chip_ot}")
    print(f"   Power Chip Short-Circuit:{power_chip_sc}")
    print(f"   Sampling Over-Current:   {sampling_oc}")
    print(f"   VBUS Over-Voltage:       {vbus_ov}")
    print(f"   VBUS Under-Voltage:      {vbus_uv}")
    print(f"   VMBUS Over-Voltage:      {vmbus_ov}")
    print(f"   VMBUS Under-Voltage:     {vmbus_uv}")
    print("==========================\n")

def parse_status_frame_1004(data):
    """
    0x1004 Status Frame:
      Byte0-1: Left Leg Power   (0~655.36 W)
      Byte2-3: Right Leg Power  (0~655.36 W)
      Byte4-5: Left Arm Power   (0~655.36 W)
      Byte6-7: Right Arm Power  (0~655.36 W)
    """
    if len(data) < 8:
        print("[ERROR] 0x1004 frame data too short!")
        return

    left_leg_raw = (data[0] << 8) | (data[1])
    left_leg_power = left_leg_raw / 100.0

    right_leg_raw = (data[2] << 8) | (data[3])
    right_leg_power = right_leg_raw / 100.0

    left_arm_raw = (data[4] << 8) | (data[5])
    left_arm_power = left_arm_raw / 100.0

    right_arm_raw = (data[6] << 8) | (data[7])
    right_arm_power = right_arm_raw / 100.0

    print("=== 0x1004 Status Frame ===")
    print(f" Left Leg Power:  {left_leg_power:.2f} W")
    print(f" Right Leg Power: {right_leg_power:.2f} W")
    print(f" Left Arm Power:  {left_arm_power:.2f} W")
    print(f" Right Arm Power: {right_arm_power:.2f} W")
    print("==========================\n")

def main():
    # 1) Connect to the CAN bus on 'can0' at 1 Mbps
    #    Make sure your system is configured for that bitrate (ip link, etc.).
    bus = can.interface.Bus(bustype='socketcan', channel='can0', bitrate=1000000)
    
    # 2) Send one 0x1001 control frame to enable auto-reporting (Byte6=1)
    print("[INFO] Sending 0x1001 to enable auto-reporting...")
    send_control_frame_enable_autoreport(bus)
    
    # Optional: Wait a moment for the board to process
    time.sleep(0.5)
    
    # 3) Continuously listen for frames on the bus.
    print("[INFO] Listening for 0x1003 / 0x1004 status frames. Press Ctrl+C to exit.\n")
    
    try:
        while True:
            msg = bus.recv(timeout=1.0)
            if msg is None:
                # no message received in this second
                continue
            
            # If we received a message, parse it.
            parse_power_board_message(msg)
            
    except KeyboardInterrupt:
        print("\n[INFO] Exiting...")
    
if __name__ == "__main__":
    main()
