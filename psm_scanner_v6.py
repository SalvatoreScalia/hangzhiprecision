# -*- coding: utf-8 -*-

"""
PSM RAW SCANNER
================
Scans ALL pages and ALL ARY registers (0..63) using AskDat (single-ARY)
and prints the RAW response frame in HEX exactly as received.

Usage:
  python psm_raw_scanner.py

Then enter your COM port (example: COM4).

This script does NOT attempt to decode the data. It only dumps raw frames
so you can send them to me and we will reverse engineer page layout, offsets
and actual energy register positions for your specific firmware.
"""

import serial
import time
import sys

HOST_ADDR   = 0x01
DEVICE_ADDR = 0xC1
READ_TIMEOUT = 0.3

CMD_ASKDAT = 0x82
CMD_ANSDAT = 0x42
CMD_RSP    = 0xC0


# -------------------------------------------------------------
# Support: XOR checksum
# -------------------------------------------------------------
def xor_checksum(data: bytes) -> int:
    x = 0
    for b in data:
        x ^= b
    return x & 0xFF


# -------------------------------------------------------------
# Build AskDat for single ARY
# -------------------------------------------------------------
def build_askdat_single(page: int, ary: int, device_addr: int) -> bytes:
    groups = [0]*8
    grp = ary // 8
    bit = ary % 8
    if 0 <= grp < 8:
        groups[grp] = (1 << bit)
    databody = bytes([page] + groups)
    flen = 5 + len(databody) + 1
    base  = bytes([0x81, device_addr, HOST_ADDR, flen, CMD_ASKDAT]) + databody
    chk   = xor_checksum(base)
    return base + bytes([chk])


# -------------------------------------------------------------
# Read raw frame (NO validation)
# -------------------------------------------------------------
def read_raw_frame(ser: serial.Serial):
    """
    Returns (raw_frame_bytes or None, error_flag)
    error_flag=True when checksum mismatch or short frame
    """
    header = ser.read(4)
    if len(header) < 4:
        return None, True

    flen = header[3]
    body = ser.read(flen - 4)
    if len(body) != flen - 4:
        return header + body, True

    frame = header + body
    # Check XOR
    if xor_checksum(frame[:-1]) != frame[-1]:
        return frame, True

    return frame, False


# -------------------------------------------------------------
# Utility: hex formatting
# -------------------------------------------------------------
def hex_dump(b: bytes) -> str:
    return " ".join(f"{x:02X}" for x in b)


# -------------------------------------------------------------
# MAIN
# -------------------------------------------------------------
def main():
    print("=== PSM RAW SCANNER ===")
    com = input("Enter COM port (example: COM4): ").strip()

    # Typical baudrates for PSM. Try until one replies.
    BAUDRATES = [38400, 115200, 19200, 9600, 4800, 2400, 1200]

    detected_baud = None
    test_page = 0x01
    test_ary  = 0x02  # DC Voltage is usually here

    print("\nDetecting baudrate...")
    for br in BAUDRATES:
        try:
            with serial.Serial(com, br, timeout=READ_TIMEOUT) as ser:
                req = build_askdat_single(test_page, test_ary, DEVICE_ADDR)
                ser.reset_input_buffer()
                ser.write(req)
                frame, err = read_raw_frame(ser)
                if frame and frame[4] in (CMD_ANSDAT, CMD_RSP):
                    detected_baud = br
                    print(f"Baudrate OK: {br}")
                    break
        except:
            pass

    if not detected_baud:
        print("ERROR: No baudrate detected.")
        sys.exit(1)

    br = detected_baud

    print("\nOpening port...")
    with serial.Serial(com, br, timeout=READ_TIMEOUT) as ser:
        print("\n=== STARTING FULL RAW SCAN ===")
        print("Scanning pages 0x00..0x03, ARY 0..63")
        print("------------------------------------------------\n")

        # SCAN pages 0x00, 0x01, 0x02, 0x03
        for page in range(0x00, 0x04):

            print(f"\n######## PAGE 0x{page:02X} ########")

            for ary in range(64):
                req = build_askdat_single(page, ary, DEVICE_ADDR)
                ser.reset_input_buffer()
                ser.write(req)

                frame, err = read_raw_frame(ser)

                if frame is None:
                    print(f"[Page 0x{page:02X}, Ary {ary:02d}] NO RESPONSE")
                    continue

                if err:
                    print(f"[Page 0x{page:02X}, Ary {ary:02d}] RAW (checksum ERROR): {hex_dump(frame)}")
                else:
                    print(f"[Page 0x{page:02X}, Ary {ary:02d}] RAW: {hex_dump(frame)}")

                # safety delay for serial stability
                time.sleep(0.02)

    print("\n=== RAW SCAN COMPLETED ===")


if __name__ == "__main__":
    main()