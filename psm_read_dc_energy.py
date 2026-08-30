# Antonio Scalia – PSM HZP v3.9 Energy Test
# v_7_d20260830_en

import serial
import struct
import time
import sys
import datetime
import csv
from typing import Dict, Tuple, Optional, List
import subprocess

"""
psm_read_dc_energy.py

    HZP v3.9 protocol PSM2000-Z
    - Energy page fix (Page02)
    - Energy index fix (49..54)
    - Product model reading via AskAry
    - CSV using semicolon separator
    - Software offsets so Wh and Ah counters start from ZERO
    - Optional parallel test script launcher
    - NEW: Batch AskDat per page (V/I/P together, Energy/Ah together) to minimize delay

Reference: HZP Communication Protocol v3.9 (pages / indices / timing, AskDat masks)  [Protocol]
(DC V/I/P on Page 0x01, Ary02/03/07; Energy/Ah on Page 0x02, Ary49..54. Replies ≤10ms, one page per frame)
"""
# =========================== USER CONFIGURATION ===============================

ENERGY_ERROR_TEST = False

METER_CONSTANT_K_IPKWH = 100          # DUT meter constant (imp/kWh)
VIRTUAL_PULSES_TARGET = 819000        # 1 virtual pulse → 10 Wh when k=100

POLLING_LOG_S = 0.1                   # You can switch to 0.2 for denser logging
TEST_TIMEOUT_S = 600.0

CSV_FILE = "psm_log.csv"

# Parallel test script (optional)
RUN_PARALLEL_SCRIPT = "./TDK_command_v3.py"
LAUNCH_PARALLEL = False   # Set False if you do not want to launch external script

# ======================= PROTOCOL CONSTANTS (HZP v3.9) ========================

HOST_ADDR = 0x01
DEFAULT_DEVICE_ADDR = 0xC1
READ_TIMEOUT = 0.3

CMD_ASKDAT = 0x82
CMD_ANSDAT = 0x42
CMD_RSP    = 0xC0
CMD_ASKARY = 0x84
CMD_ANSARY = 0x44
CMD_WRTARY = 0x85   # WrtAry

PAGE_SYS  = 0x00
PAGE_MEAS = 0x01     # DC V/I/P page
PAGE_ENER = 0x02     # Energy/Ah page

# SYS page arrays
ARY_BAUDRATE     = 8
ARY_PRODUCT_TYPE = 4

# MEAS page arrays (FLOAT)
ARY_DC_V = 2
ARY_DC_I = 3
ARY_DC_P = 7

# ENER page arrays (DOUBLE)
ARY_EN_WH_POS = 49
ARY_EN_WH_NEG = 50
ARY_EN_WH_TOT = 51
ARY_EN_AH_POS = 52
ARY_EN_AH_NEG = 53
ARY_EN_AH_TOT = 54

BAUDRATES = [38400, 115200, 19200, 9600, 4800, 2400, 1200]

# =============================== UTILITIES ====================================

def xor_checksum(data: bytes) -> int:
    c = 0
    for b in data:
        c ^= b
    return c & 0xFF


def build_askdat(page: int, ary: int, device_addr: int) -> bytes:
    # Single-Ary AskDat (kept for compatibility)
    groups = [0]*8
    grp = ary // 8
    bit = ary % 8
    groups[grp] = (1 << bit)
    databody = bytes([page] + groups)
    flen = 5 + len(databody) + 1
    base = bytes([0x81, device_addr, HOST_ADDR, flen, CMD_ASKDAT]) + databody
    return base + bytes([xor_checksum(base)])


def build_askary(page: int, ary: int, start0: int, start1: int, device_addr: int) -> bytes:
    databody = bytes([page, ary, start0, start1])
    flen = 5 + len(databody) + 1
    base = bytes([0x81, device_addr, HOST_ADDR, flen, CMD_ASKARY]) + databody
    return base + bytes([xor_checksum(base)])


def read_frame(ser: serial.Serial) -> Optional[bytes]:
    header = ser.read(4)
    if len(header) < 4: return None
    _, _, _, flen = header
    body = ser.read(flen - 4)
    if len(body) != flen - 4: return None
    frame = header + body
    if xor_checksum(frame[:-1]) != frame[-1]: return None
    return frame


def decode_rsp(frame: bytes) -> str:
    if len(frame) >= 8 and frame[4] == CMD_RSP:
        rsp = (frame[6] << 8) | frame[7]
        status = "OK" if (rsp & 0x8000) == 0 else "ERROR"
        return f"Rsp{status} (0x{rsp:04X})"
    return ""


def decode_float(b: bytes) -> float:
    return struct.unpack("<f", b)[0]


def decode_double(b: bytes) -> float:
    return struct.unpack("<d", b)[0]


# ============================= AnsDat Parser ==================================

def parse_ansdat_single(frame: bytes, target_ary: int, dtype: str) -> Optional[float]:
    # Single-Ary parser (kept for compatibility)
    if len(frame) < 15 or frame[4] != CMD_ANSDAT: return None

    groups = frame[6:14]
    size = 4 if dtype == "float" else 8
    data_end = len(frame) - 1

    grp_idx = target_ary // 8
    bit = target_ary % 8

    if (groups[grp_idx] & (1 << bit)) == 0:
        return None

    # Alternate pointer path sometimes used by vendor stacks
    alt_ptr = 6 + grp_idx + 1
    if alt_ptr + size <= data_end:
        chunk = frame[alt_ptr:alt_ptr+size]
        try:
            return decode_float(chunk) if size == 4 else decode_double(chunk)
        except:
            pass

    ptr = 14
    if ptr + size <= data_end:
        chunk = frame[ptr:ptr+size]
        try:
            return decode_float(chunk) if size == 4 else decode_double(chunk)
        except:
            pass

    return None


# ===================== Batch AskDat (per page) ================================

def build_askdat_multi(page: int, ary_list: List[int], device_addr: int) -> bytes:
    """
    Build one AskDat (0x82) with multiple Ary within the same page by setting
    the Grp0..Grp7 bitmasks as specified by HZP v3.9.  (One page per frame.)  [Protocol]
    
    """
    groups = [0]*8
    for ary in ary_list:
        grp = ary // 8
        bit = ary % 8
        if 0 <= grp < 8:
            groups[grp] |= (1 << bit)
    databody = bytes([page] + groups)
    flen = 5 + len(databody) + 1
    base = bytes([0x81, device_addr, HOST_ADDR, flen, CMD_ASKDAT]) + databody
    return base + bytes([xor_checksum(base)])


def parse_ansdat_multi(frame: bytes, requested_aries: List[Tuple[int, str]]) -> Dict[int, float]:
    """
    Correct parsing for HZP AnsDat (0x42) with interleaved groups:
    81 RxID TxID Flen 42 Page  [Grp0 (data...)] [Grp1 (data...)] ... [Grp7 (data...)]  ChkSum
    We read: Page (idx 5), then starting at idx 6:
      - read Grp0 mask byte
      - for each set bit in Grp0, read its data block (FLOAT=4 or DOUBLE=8)
      - then Grp1 mask byte, its data blocks, ... up to Grp7
    """
    if len(frame) < 15 or frame[4] != CMD_ANSDAT:
        return {}

    ptr = 5  # frame[5] = Page
    page = frame[ptr]
    ptr += 1  # now at Grp0

    out: Dict[int, float] = {}

    # We'll look up dtype per ary quickly
    dtype_map = {a: t for (a, t) in requested_aries}

    for grp_idx in range(8):
        if ptr >= len(frame) - 1:  # keep last byte for checksum
            break
        mask = frame[ptr]
        ptr += 1  # move past this group mask

        if mask == 0:
            continue

        # Bits 0..7 within this group
        for bit in range(8):
            if (mask >> bit) & 0x01:
                ary_idx = grp_idx * 8 + bit
                dtype = dtype_map.get(ary_idx)
                if dtype is None:
                    # Unexpected Ary (not requested): can't size it -> abort safely
                    return out
                size = 4 if dtype == "float" else 8
                if ptr + size > len(frame) - 1:  # avoid checksum
                    return out
                chunk = frame[ptr:ptr+size]
                ptr += size
                val = decode_float(chunk) if size == 4 else decode_double(chunk)
                out[ary_idx] = val

    return out


# =============================== READ FUNCTIONS ==============================

def detect_baud(serial_port: str, device_addr: int):
    req = build_askdat(PAGE_SYS, ARY_BAUDRATE, device_addr)
    for br in BAUDRATES:
        try:
            with serial.Serial(serial_port, br, timeout=READ_TIMEOUT) as ser:
                ser.reset_input_buffer()
                ser.write(req)
                frame = read_frame(ser)
                if frame and frame[4] == CMD_ANSDAT:
                    return br, frame[14]
                elif frame and frame[4] == CMD_RSP:
                    continue
        except:
            pass
    return None, None


def read_register(ser, page, ary, dtype, device_addr):
    req = build_askdat(page, ary, device_addr)
    ser.reset_input_buffer()
    ser.write(req)
    frame = read_frame(ser)
    if not frame: return None
    if frame[4] == CMD_RSP:
        print("Rsp:", decode_rsp(frame))
        return None
    return parse_ansdat_single(frame, ary, dtype)


def askary_ascii(ser, page, ary, nbytes, device_addr):
    nbytes = max(12, min(nbytes, 32))
    req = build_askary(page, ary, 0, nbytes-1, device_addr)
    for _ in range(2):
        ser.reset_input_buffer()
        ser.write(req)
        frame = read_frame(ser)
        if frame and frame[4] == CMD_ANSARY and frame[5] == page and frame[6] == ary:
            data = frame[10:-1]
            return data.decode("ascii", errors="ignore").rstrip("\x00").strip()
    return ""


def read_product_type(ser, device_addr):
    return askary_ascii(ser, PAGE_SYS, ARY_PRODUCT_TYPE, 12, device_addr)


# ======================= Grouped Reads per Page ===============================

def read_measure_group(ser: serial.Serial, device_addr: int) -> Dict[str, Optional[float]]:
    """
    Page 0x01 (MEAS): Ary02 (V), Ary03 (I), Ary07 (P) — all FLOAT (4 bytes). [Protocol]
    
    """
    ary_meas = [(ARY_DC_V, "float"), (ARY_DC_I, "float"), (ARY_DC_P, "float")]
    req = build_askdat_multi(PAGE_MEAS, [a for a, _ in ary_meas], device_addr)
    ser.reset_input_buffer()
    ser.write(req)
    frame = read_frame(ser)
    if not frame:
        return {}
    if frame[4] == CMD_RSP:
        print("Rsp:", decode_rsp(frame))
        return {}
    vals = parse_ansdat_multi(frame, ary_meas)
    return {
        "DC Voltage": vals.get(ARY_DC_V),
        "DC Current": vals.get(ARY_DC_I),
        "DC Power":   vals.get(ARY_DC_P),
    }


def read_energy_group(ser: serial.Serial, device_addr: int) -> Dict[str, Optional[float]]:
    """
    Page 0x02 (ENER): Ary49..54 — Energy/Ah, DOUBLE (8 bytes): Wh/Ah. [Protocol]
    
    """
    ary_energy = [
        (ARY_EN_WH_POS, "double"),
        (ARY_EN_WH_NEG, "double"),
        (ARY_EN_WH_TOT, "double"),
        (ARY_EN_AH_POS, "double"),
        (ARY_EN_AH_NEG, "double"),
        (ARY_EN_AH_TOT, "double"),
    ]
    req = build_askdat_multi(PAGE_ENER, [a for a, _ in ary_energy], device_addr)
    ser.reset_input_buffer()
    ser.write(req)
    frame = read_frame(ser)
    if not frame:
        return {}
    if frame[4] == CMD_RSP:
        print("Rsp:", decode_rsp(frame))
        return {}
    vals = parse_ansdat_multi(frame, ary_energy)
    return {
        "EnergyPositive": vals.get(ARY_EN_WH_POS),
        "EnergyNegative": vals.get(ARY_EN_WH_NEG),
        "EnergyTotal":    vals.get(ARY_EN_WH_TOT),
        "AhPositive":     vals.get(ARY_EN_AH_POS),
        "AhNegative":     vals.get(ARY_EN_AH_NEG),
        "AhTotal":        vals.get(ARY_EN_AH_TOT),
    }


# ============================ REGISTER MAP (kept) =============================

def model_register_map(model: str):
    # Kept for reference; no longer used for grouped reads inside the loop,
    # but still useful for initial structure and compatibility.
    return {
        "DC Voltage": (PAGE_MEAS, ARY_DC_V, "float"),
        "DC Current": (PAGE_MEAS, ARY_DC_I, "float"),
        "DC Power":   (PAGE_MEAS, ARY_DC_P, "float"),

        "EnergyPositive": (PAGE_ENER, ARY_EN_WH_POS, "double"),
        "EnergyNegative": (PAGE_ENER, ARY_EN_WH_NEG, "double"),
        "EnergyTotal":    (PAGE_ENER, ARY_EN_WH_TOT, "double"),

        "AhPositive":     (PAGE_ENER, ARY_EN_AH_POS, "double"),
        "AhNegative":     (PAGE_ENER, ARY_EN_AH_NEG, "double"),
        "AhTotal":        (PAGE_ENER, ARY_EN_AH_TOT, "double"),
    }


# ================================ CSV =========================================

def open_csv(path):
    newfile = False
    try:
        open(path, "r").close()
    except:
        newfile = True

    f = open(path, "a", newline="")
    writer = csv.writer(f, delimiter=';')

    if newfile:
        writer.writerow([
            "timestamp","epoch",
            "dc_voltage_V","dc_current_A","dc_power_W",
            "Wh_pos","Wh_neg","Wh_total",
            "Ah_pos","Ah_neg","Ah_total",
            "note"
        ])
    return writer, f


# ============================ PARALLEL PROCESS ===============================

def launch_parallel_test(script_path):
    try:
        process = subprocess.Popen(
            [sys.executable, script_path],
            creationflags=subprocess.CREATE_NEW_CONSOLE  # Windows-only; keep as in original
        )
        print(f"\nParallel script started: {script_path}\n")
        return process
    except Exception as e:
        print(f"ERROR launching script: {e}")
        return None


# =============================== ENERGY WINDOW ===============================

def target_wh_from_k(k, N):
    return 1000.0 * (N / k)


def energy_window_test(ser, device_addr, targetWh, poll_s, timeout_s, writer, regmap, offsets):

    E0_pos, E0_neg, E0_total, AH0_pos, AH0_neg, AH0_total = offsets
    t0 = time.time()

    # Initial total energy (not used later; kept as in original)
    E_init = read_register(ser, PAGE_ENER, ARY_EN_WH_TOT, "double", device_addr) or 0.0

    while True:
        time.sleep(poll_s)

        # Grouped reads: one frame for MEAS page, one for ENER page (minimizes delay). [Protocol]
        # 
        meas = read_measure_group(ser, device_addr)
        ener = read_energy_group(ser, device_addr)
        snapshot_raw = {**(meas or {}), **(ener or {})}

        # APPLY SOFTWARE OFFSETS so energy starts at zero
        snapshot = {
            "DC Voltage": snapshot_raw.get("DC Voltage"),
            "DC Current": snapshot_raw.get("DC Current"),
            "DC Power":   snapshot_raw.get("DC Power"),

            "EnergyPositive": (snapshot_raw.get("EnergyPositive") or 0.0) - E0_pos,
            "EnergyNegative": (snapshot_raw.get("EnergyNegative") or 0.0) - E0_neg,
            "EnergyTotal":    (snapshot_raw.get("EnergyTotal")    or 0.0) - E0_total,

            "AhPositive": (snapshot_raw.get("AhPositive") or 0.0) - AH0_pos,
            "AhNegative": (snapshot_raw.get("AhNegative") or 0.0) - AH0_neg,
            "AhTotal":    (snapshot_raw.get("AhTotal")    or 0.0) - AH0_total
        }

        dWh = snapshot["EnergyTotal"]

        writer.writerow([
            #datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
            datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3],  # Centesimi di secondo
            int(time.time()),
            snapshot["DC Voltage"], snapshot["DC Current"], snapshot["DC Power"],
            snapshot["EnergyPositive"], snapshot["EnergyNegative"], snapshot["EnergyTotal"],
            snapshot["AhPositive"], snapshot["AhNegative"], snapshot["AhTotal"],
            f"poll dWh={dWh:.6f}"
        ])

        if dWh >= targetWh:
            return {"dWh": dWh, "elapsed": time.time()-t0}

        if time.time() - t0 > timeout_s:
            return {"dWh": dWh, "elapsed": time.time()-t0}


# ================================= MAIN =====================================

def main():

    print("=== PSM HZP v3.9 — Energy Test ===")
    serial_port = input("Enter COM port (e.g., COM4): ").strip()

    # Detect baud
    print("\nDetecting baudrate...")
    br, code = detect_baud(serial_port, DEFAULT_DEVICE_ADDR)
    if not br:
        print("ERROR: Could not detect baudrate.")
        sys.exit(1)
    print(f"Baudrate OK: {br} (code 0x{code:02X})")

    # CSV
    writer, fcsv = open_csv(CSV_FILE)

    # Identify product
    with serial.Serial(serial_port, br, timeout=READ_TIMEOUT) as ser:
        model = read_product_type(ser, DEFAULT_DEVICE_ADDR) or "Unknown"
    print(f"Product model: {model}")

    regmap = model_register_map(model)

    # Launch parallel script
    if LAUNCH_PARALLEL:
        launch_parallel_test(RUN_PARALLEL_SCRIPT)

    # Initial read (grouped per page for temporal consistency)
    print("\nReading initial values...")
    with serial.Serial(serial_port, br, timeout=READ_TIMEOUT) as ser:
        meas0 = read_measure_group(ser, DEFAULT_DEVICE_ADDR)
        ener0 = read_energy_group(ser, DEFAULT_DEVICE_ADDR)
        initial_raw = {**(meas0 or {}), **(ener0 or {})}

    # STORE OFFSETS so counters start at 0
    E0_pos   = initial_raw.get("EnergyPositive") or 0.0
    E0_neg   = initial_raw.get("EnergyNegative") or 0.0
    E0_total = initial_raw.get("EnergyTotal")    or 0.0
    AH0_pos  = initial_raw.get("AhPositive")     or 0.0
    AH0_neg  = initial_raw.get("AhNegative")     or 0.0
    AH0_total= initial_raw.get("AhTotal")        or 0.0

    offsets = (E0_pos, E0_neg, E0_total, AH0_pos, AH0_neg, AH0_total)

    # Write INITIAL row with zeros
    writer.writerow([
        datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
        int(time.time()),
        initial_raw.get("DC Voltage"),
        initial_raw.get("DC Current"),
        initial_raw.get("DC Power"),
        0, 0, 0,    # Wh zeros
        0, 0, 0,    # Ah zeros
        "INITIAL"
    ])
    fcsv.flush()

    # Energy window test
    targetWh = target_wh_from_k(METER_CONSTANT_K_IPKWH, VIRTUAL_PULSES_TARGET)
    print(f"\nEnergy target: {targetWh:.6f} Wh")

    with serial.Serial(serial_port, br, timeout=READ_TIMEOUT) as ser:
        result = energy_window_test(
            ser, DEFAULT_DEVICE_ADDR,
            targetWh,
            POLLING_LOG_S,
            TEST_TIMEOUT_S,
            writer,
            regmap,
            offsets
        )

    print(f"\nFINISHED: ΔWh = {result['dWh']:.6f} in {result['elapsed']:.2f}s")

    fcsv.flush()
    fcsv.close()
    print("\n=== COMPLETED ===")


if __name__ == "__main__":
    main()
