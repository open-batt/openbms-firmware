#!/usr/bin/env python3

import sys
import serial
import time

# ---------------------------------------------------------------
# Constants
# ---------------------------------------------------------------
APP_FLASH_START     = 0x0800A000
BAUD_RATE           = 230400
TIMEOUT_PING        = 2.0    # seconds — per ping attempt
TIMEOUT_ERASE       = 10.0   # seconds — erase takes ~3s for 108 pages
TIMEOUT_PROGRAM     = 2.0    # seconds — per line
TIMEOUT_CRC         = 5.0    # seconds
TIMEOUT_FLAG        = 2.0    # seconds
TIMEOUT_BOOT_WAIT   = 10.0   # seconds — max wait for bootloader after firmware resets
MAX_RETRIES         = 2
PING_RETRIES        = 3
BOOT_RETRY_DELAY    = 0.2    # seconds — wait between pings after firmware reset

# ---------------------------------------------------------------
# CRC-32/MPEG-2 — matches STM32 HAL CRC peripheral configuration:
#   DEFAULT_POLYNOMIAL  = 0x04C11DB7
#   DEFAULT_INIT_VALUE  = 0xFFFFFFFF
#   INPUT_INVERSION     = NONE
#   OUTPUT_INVERSION    = DISABLE
#   INPUT_FORMAT        = BYTES
# ---------------------------------------------------------------
def crc32_mpeg2(data: bytes) -> int:
    crc = 0xFFFFFFFF
    for byte in data:
        crc ^= byte << 24
        for _ in range(8):
            if crc & 0x80000000:
                crc = (crc << 1) ^ 0x04C11DB7
            else:
                crc <<= 1
            crc &= 0xFFFFFFFF
    return crc

# ---------------------------------------------------------------
# Parse Intel HEX file
# Returns binary image starting at APP_FLASH_START and its size
# ---------------------------------------------------------------
def parse_hex_file(filename: str):
    data_map  = {}
    ext_addr  = 0

    with open(filename, 'r') as f:
        lines = [line.strip() for line in f.readlines()]

    for line in lines:
        if not line or line[0] != ':':
            continue

        byte_count  = int(line[1:3],   16)
        address     = int(line[3:7],   16)
        record_type = int(line[7:9],   16)
        data        = bytes.fromhex(line[9:9 + byte_count * 2])

        if record_type == 0x00:  # Data
            full_address = (ext_addr << 16) | address
            for i, byte in enumerate(data):
                data_map[full_address + i] = byte

        elif record_type == 0x04:  # Extended linear address
            ext_addr = int(line[9:13], 16)

        elif record_type == 0x01:  # EOF
            break

    if not data_map:
        return None, 0

    min_addr = min(data_map.keys())
    max_addr = max(data_map.keys())
    size     = max_addr - min_addr + 1

    # Build contiguous binary image — gaps filled with 0xFF (erased flash)
    binary = bytearray([0xFF] * size)
    for addr, byte in data_map.items():
        binary[addr - min_addr] = byte

    return bytes(binary), size

# ---------------------------------------------------------------
# Send command and wait for response
# ---------------------------------------------------------------
def send_command(ser: serial.Serial, command: str, timeout: float) -> str:
    ser.reset_input_buffer()
    ser.write(command.encode())

    start    = time.time()
    response = ''

    while time.time() - start < timeout:
        if ser.in_waiting:
            char = ser.read(1).decode('ascii', errors='ignore')
            response += char
            if '\n' in response:
                return response.strip()
        time.sleep(0.001)

    return None

# ---------------------------------------------------------------
# Ping bootloader
#
# Response "0" — bootloader active, ready to proceed
# Response "1" — main firmware running, it will remove flag
#                and reset into bootloader — wait and retry
# No response   — nothing running, error
# ---------------------------------------------------------------
def ping_bootloader(ser: serial.Serial) -> bool:
    print("Pinging device...")

    # First ping — check what's running
    for attempt in range(PING_RETRIES):
        response = send_command(ser, "B\n", timeout=TIMEOUT_PING)

        if response == "0":
            # Bootloader already active
            print("Bootloader active\n")
            return True

        elif response == "1":
            # Main firmware is running — it will remove flag and reset
            print("Main firmware detected — waiting for reset into bootloader...")

            # Wait for STM32 to reset and bootloader to start
            start = time.time()
            while time.time() - start < TIMEOUT_BOOT_WAIT:
                time.sleep(BOOT_RETRY_DELAY)

                response = send_command(ser, "B\n", timeout=TIMEOUT_PING)

                if response == "0":
                    print("Bootloader active\n")
                    return True

                # Print dots to show progress
                print(".", end='', flush=True)

            print(f"\nERR: bootloader did not respond within {TIMEOUT_BOOT_WAIT}s after firmware reset")
            return False
            
        elif response == "2":
            print("Bootloader starting failed...")

        else:
            if attempt < PING_RETRIES - 1:
                print(f"  No response, retrying ({attempt + 2}/{PING_RETRIES})...")

    return False

# ---------------------------------------------------------------
# Main
# ---------------------------------------------------------------
def main():
    if len(sys.argv) != 3:
        print("Usage: python flash.py <COM_PORT> <HEX_FILE>")
        print("Example: python flash.py COM3 firmware.hex")
        sys.exit(1)

    port     = sys.argv[1]
    hex_file = sys.argv[2]

    # -------------------------------------------------------
    # Parse hex file
    # -------------------------------------------------------
    print(f"Parsing {hex_file}...")

    try:
        binary_data, size = parse_hex_file(hex_file)
    except FileNotFoundError:
        print(f"ERR: file not found: {hex_file}")
        sys.exit(1)
    except Exception as e:
        print(f"ERR: failed to parse hex file: {e}")
        sys.exit(1)

    if binary_data is None:
        print("ERR: hex file contains no data")
        sys.exit(1)

    # Read raw lines for programming
    with open(hex_file, 'r') as f:
        hex_lines = [line.strip() for line in f.readlines() if line.strip()]

    total_lines  = len(hex_lines)
    expected_crc = crc32_mpeg2(binary_data)

    print(f"Firmware size : {size} bytes")
    print(f"Expected CRC  : 0x{expected_crc:08X}")
    print(f"Total lines   : {total_lines}\n")

    # -------------------------------------------------------
    # Open serial port
    # -------------------------------------------------------
    try:
        ser = serial.Serial(port, BAUD_RATE, timeout=1)
        time.sleep(0.5)
        ser.reset_input_buffer()
    except serial.SerialException as e:
        print(f"ERR: failed to open {port}: {e}")
        sys.exit(1)

    print(f"Connected to {port} at {BAUD_RATE} baud\n")

    try:
        # -------------------------------------------------------
        # Step 0 — Ping bootloader
        # -------------------------------------------------------
        if not ping_bootloader(ser):
            print("ERR: bootloader not responding")
            print("     Make sure device is powered and connected")
            sys.exit(1)

        # -------------------------------------------------------
        # Step 1 — Erase
        # -------------------------------------------------------
        print("Erasing flash...")
        response = send_command(ser, "E\n", timeout=TIMEOUT_ERASE)

        if response is None:
            print("ERR: erase timeout — no response from device")
            sys.exit(1)

        if response != "0":
            print(f"ERR: erase failed, response: {response}")
            sys.exit(1)

        print("Erase OK\n")

        # -------------------------------------------------------
        # Step 2 — Program line by line
        # -------------------------------------------------------
        print("Programming firmware...")

        for idx, line in enumerate(hex_lines):
            if not line:
                continue

            command = f"P,{line}\n"
            success = False

            for attempt in range(MAX_RETRIES):
                response = send_command(ser, command, timeout=TIMEOUT_PROGRAM)

                # 0 = OK, 1 = EOF — both are acceptable
                if response in ("0", "1"):
                    success = True
                    break
                else:
                    if attempt < MAX_RETRIES - 1:
                        print(f"\n  Retrying line {idx + 1} (attempt {attempt + 2}/{MAX_RETRIES})...")
                    else:
                        print(f"\nERR: programming failed at line {idx + 1}")
                        print(f"     Line    : {line}")
                        print(f"     Response: {response}")
                        sys.exit(1)

            # Progress
            percent = (idx + 1) / total_lines * 100
            print(f"\r  {percent:5.1f}%  ({idx + 1}/{total_lines} lines)", end='', flush=True)

        print("\nProgramming OK\n")

        # -------------------------------------------------------
        # Step 3 — CRC verification
        # -------------------------------------------------------
        print("Verifying CRC...")
        crc_command = f"C,{size},{expected_crc}\n"
        response    = send_command(ser, crc_command, timeout=TIMEOUT_CRC)

        if response is None:
            print("ERR: CRC timeout — no response from device")
            sys.exit(1)

        if response != "0":
            print(f"ERR: CRC mismatch, response: {response}")
            sys.exit(1)

        print("CRC OK\n")

        # -------------------------------------------------------
        # Step 4 — Set application flag
        # -------------------------------------------------------
        print("Setting application flag...")
        response = send_command(ser, "A\n", timeout=TIMEOUT_FLAG)

        if response is None:
            print("ERR: set flag timeout — no response from device")
            sys.exit(1)

        if response != "0":
            print(f"ERR: set flag failed, response: {response}")
            sys.exit(1)

        print("Flag OK\n")

        # -------------------------------------------------------
        # Step 5 — Reset
        # -------------------------------------------------------
        print("Resetting device...")
        ser.write("R\n".encode())
        time.sleep(0.1)
        print("Done! Device is resetting.\n")

    except KeyboardInterrupt:
        print("\nAborted by user")
        sys.exit(1)

    finally:
        ser.close()

if __name__ == "__main__":
    main()
