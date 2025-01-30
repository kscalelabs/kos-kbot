#!/usr/bin/env python3
import can
import time

def setup_can(channel='can0'):
    return can.interface.Bus(
        channel=channel,
        interface='socketcan',
        bitrate=1000000
    )

def create_arbitration_id():
    # Combine 0x1001 (message type) with 0xAA (board ID)
    return (0x1001) | (0x430 << 16)  # Results in 0x4301001

def try_can_interface(channel):
    print(f"\nTrying {channel}...")
    try:
        bus = setup_can(channel)

        print(f"Arbitration ID: {create_arbitration_id():X}")
        
        # Create control message
        msg = can.Message(
            arbitration_id=create_arbitration_id(),
            data=[0, 0, 0, 0, 0, 0, 1, 0],  # Enable auto-reporting
            is_extended_id=True,
            dlc=8
        )
        
        print(f"Sending message on {channel}...")
        bus.send(msg)
        
        # Wait for response
        print(f"Waiting for response on {channel}...")
        response = bus.recv(timeout=1.0)
        
        if response:
            print(f"Received message on {channel}:")
            print(f"ID: {response.arbitration_id:X}")
            print(f"Data: {' '.join([f'{b:02X}' for b in response.data])}")
        else:
            print(f"No response received on {channel}")
            
    except Exception as e:
        print(f"Error on {channel}: {e}")
    finally:
        if 'bus' in locals():
            bus.shutdown()

def main():
    # Try each CAN interface
    for i in range(5):  # will try can0 through can4
        try_can_interface(f'can{i}')
        time.sleep(0.5)  # Small delay between attempts

if __name__ == "__main__":
    main()
