import sys
import time

import serial


def send_hex_string(ser, hex_string):
    """Send hex string like 'FF FF 01 02 01 FB'"""
    bytes_to_send = bytes.fromhex(hex_string.replace(" ", ""))
    print(f"Sending: {hex_string}")
    ser.write(bytes_to_send)
    ser.flush()


def read_response(ser, timeout=0.5):
    """Read response with timeout"""
    start_time = time.time()
    response = b""
    while (time.time() - start_time) < timeout:
        if ser.in_waiting:
            response += ser.read(ser.in_waiting)
            time.sleep(0.01)  # Small delay for more data
        else:
            if response:  # If we got data and no more coming
                break
            time.sleep(0.01)
    return response


def main():
    port = "/dev/ttyAMA0"
    baud = 115200

    print(f"Opening {port} at {baud} baud...")
    try:
        ser = serial.Serial(port, baud, timeout=1)
        time.sleep(2)  # Wait for ESP32 to stabilize
        print("Connected!\n")

        print("=== Test 1: Ping Servo ID 0 ===")

        # ping_packet = "FF FF 00 02 01 7F FC"
        ping_packet = "FF"
        send_hex_string(ser, ping_packet)

        response = read_response(ser, timeout=1.0)
        if response:
            print(f"Response: {response.hex(' ').upper()}")
        else:
            print("No response received!")
        ser.close()
        print("\nTest complete!")
    except serial.SerialException as e:
        print(f"Error: {e}")
        sys.exit(1)


if __name__ == "__main__":
    main()
