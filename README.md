# hangzhiprecision
# PSM HZP v3.9 — DC Energy Test

Lightweight Python tool to read DC measurements and energy counters from PSM2000-Z devices using the HZP v3.9 protocol, and optionally coordinate a parallel test script. The tool performs grouped AskDat requests per page (V/I/P together, Energy/Ah together) to minimize latency and writes time-stamped snapshots to a semicolon-separated CSV.

## Key features
- Detects device baudrate automatically (trial of common baudrates).
- Grouped reads per page (faster and temporally consistent snapshots).
- Energy and Ah counters are offset in software so logged values can start from zero.
- Optional launcher for a parallel test script (e.g., to drive a load).
- CSV output with epoch and sub-second timestamps.

## Files
- `psm_read_dc_energy.py` — main script implementing the test and logging.

## Requirements
- Python 3.8+ (recommended)
- Dependencies:
  - `pyserial`
  - `pymodbus` (only required if you use the Modbus/DUT features from other scripts)

Install dependencies with pip:

```bash
pip install pyserial pymodbus
```

## Usage
Run the script and provide the COM port when prompted:

```bash
python psm_read_dc_energy.py
```

The script will:

1. Detect the PSM baudrate.
2. Open (or create) `psm_log.csv` and write a header (if needed).
3. Read initial values and store offsets so energy/Ah start at zero in the log.
4. Run the energy window test, polling at the `POLLING_LOG_S` interval and appending rows to the CSV until the energy target or timeout is reached.

## Configuration (top of file)
- `POLLING_LOG_S` — polling interval in seconds (default `0.1`).
- `TEST_TIMEOUT_S` — maximum test duration in seconds.
- `CSV_FILE` — output CSV filename.
- `METER_CONSTANT_K_IPKWH`, `VIRTUAL_PULSES_TARGET` — used to compute energy target when testing meter pulses.
- `LAUNCH_PARALLEL` and `RUN_PARALLEL_SCRIPT` — control launching an external script in a new console.

## CSV format
Semicolon-separated columns:

- `timestamp` (human readable with sub-second precision)
- `epoch` (integer seconds)
- `dc_voltage_V`, `dc_current_A`, `dc_power_W`
- `Wh_pos`, `Wh_neg`, `Wh_total`
- `Ah_pos`, `Ah_neg`, `Ah_total`
- `note`

## Notes
- The script implements HZP v3.9 AskDat/AnsDat parsing and assumes replies arrive within `READ_TIMEOUT` (default 0.3 s). Adjust timeouts if your device responds slower.
- The grouped AskDat approach reduces per-sample delay; the effective sampling period is `POLLING_LOG_S` plus the time needed to perform the two grouped reads.

## License
Choose a license for your repository (e.g., MIT) or add one as needed.

## Short description (for GitHub repository listing)
Python script that logs DC voltage/current/power and energy/Ah from PSM2000-Z devices using the HZP v3.9 protocol; writes time-stamped snapshots to a semicolon CSV and supports configurable polling and optional parallel test control.
