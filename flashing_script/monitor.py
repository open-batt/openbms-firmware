#!/usr/bin/env python3

import sys
import serial
import time
import struct
import msvcrt
import csv
from datetime import datetime

# ---------------------------------------------------------------
# Protocol constants
# ---------------------------------------------------------------
BAUD_RATE        = 230400
TIMEOUT_RESPONSE = 0.03    # 30ms — tight enough for 20Hz cycle

CMD_READ = 0x02

# ---------------------------------------------------------------
# Registers to monitor  — Peripheral_Data_t / FuelGauge_Data_t
# ---------------------------------------------------------------
REG_CELL_VOLTAGE        = 0x00   # float[7]  — mV
REG_CELL_VOLTAGE_FILT   = 0x01   # float[7]  — mV
REG_PACK_VOLTAGE        = 0x02   # float     — mV
REG_PACK_VOLTAGE_FILT   = 0x03   # float     — mV
REG_PACK_CURRENT        = 0x04   # float     — mA
REG_PACK_CURRENT_FILT   = 0x05   # float     — mA
REG_TEMPERATURE         = 0x06   # float     — °C
REG_FET_STATUS          = 0x09   # uint16    — flags
REG_UPTIME              = 0x51   # uint64    — ms  (Control_Data_t)
REG_LEARNING_STATUS     = 0xAC   # uint16    — flags (FuelGauge_Data_t)

POLL_INTERVAL_S = 0.05           # 20 Hz
DISPLAY_EVERY   = 4              # update screen every 4th read = 5 Hz

# ---------------------------------------------------------------
# ANSI codes
# ---------------------------------------------------------------
HOME   = "\033[H"
EOL    = "\033[K"
BOLD   = "\033[1m"
RESET  = "\033[0m"
CYAN   = "\033[96m"
GREEN  = "\033[92m"
YELLOW = "\033[93m"
RED    = "\033[91m"
DIM    = "\033[2m"


# ---------------------------------------------------------------
# Protocol helpers
# ---------------------------------------------------------------
def checksum(data):
    return sum(data) % 256

def build_read_command(address):
    packet = bytes([CMD_READ, address, 0x00])
    return packet + bytes([checksum(packet)])

def read_register(ser, address):
    ser.reset_input_buffer()
    ser.write(build_read_command(address))

    header = ser.read(3)
    if len(header) < 3:
        return None

    payload_length = header[2]
    rest = ser.read(payload_length + 1)
    if len(rest) < payload_length + 1:
        return None

    raw = header + rest

    if checksum(raw[:-1]) != raw[-1]:
        return None
    if raw[1] != address:
        return None

    return raw[3:3 + payload_length]


def read_float_array(ser, address, count):
    data = read_register(ser, address)
    if data is None or len(data) < count * 4:
        return None
    return [struct.unpack_from('<f', data, i * 4)[0] for i in range(count)]

def read_float(ser, address):
    result = read_float_array(ser, address, 1)
    return result[0] if result else None

def read_uint16(ser, address):
    data = read_register(ser, address)
    if data is None or len(data) < 2:
        return None
    return struct.unpack_from('<H', data)[0]

def read_uint64(ser, address):
    data = read_register(ser, address)
    if data is None or len(data) < 8:
        return None
    return struct.unpack_from('<Q', data)[0]


# ---------------------------------------------------------------
# Display helpers
# ---------------------------------------------------------------
def fmt_f(val, dec=2):
    return f"{val:.{dec}f}" if val is not None else "ERR"

def fmt_h(val):
    return f"0x{val:04X}" if val is not None else "ERR"

def fmt_csv(val):
    return "" if val is None else str(val)

def bar(val, lo, hi, width=24):
    if val is None:
        return DIM + "[" + "?" * width + "]" + RESET
    ratio  = max(0.0, min(1.0, (val - lo) / (hi - lo)))
    filled = int(ratio * width)
    return DIM + "[" + RESET + GREEN + "█" * filled + DIM + "░" * (width - filled) + "]" + RESET

def cell_color(v):
    if v is None:             return RED
    if v < 2800 or v > 4200: return RED
    if v < 3000 or v > 4150: return YELLOW
    return GREEN

def current_color(v):
    if v is None:      return RED
    if abs(v) > 18000: return RED
    if abs(v) > 12000: return YELLOW
    return GREEN

def temp_color(t):
    if t is None: return RED
    if t >= 55:   return RED
    if t >= 45:   return YELLOW
    return GREEN

def ln(text=''):
    print(text + EOL)


# ---------------------------------------------------------------
# CSV logging
# ---------------------------------------------------------------
CSV_HEADER = [
    "uptime_ms",
    "cell_v1",  "cell_v2",  "cell_v3",  "cell_v4",  "cell_v5",  "cell_v6",  "cell_v7",
    "cell_vf1", "cell_vf2", "cell_vf3", "cell_vf4", "cell_vf5", "cell_vf6", "cell_vf7",
    "pack_voltage_mv",
    "pack_voltage_filtered_mv",
    "current_ma",
    "current_filtered_ma",
    "temperature_c",
    "fet_status",
    "learning_status",
]

def make_csv_row(uptime, cell_v, cell_v_filt, pack_v, pack_vf,
                 current, curr_filt, temp_c, fet_status, learning_status):
    row = [fmt_csv(uptime)]
    row += [fmt_csv(cell_v[i]      if cell_v      else None) for i in range(7)]
    row += [fmt_csv(cell_v_filt[i] if cell_v_filt else None) for i in range(7)]
    row += [fmt_csv(pack_v), fmt_csv(pack_vf)]
    row += [fmt_csv(current), fmt_csv(curr_filt), fmt_csv(temp_c)]
    row += [fmt_csv(fet_status), fmt_csv(learning_status)]
    return row


# ---------------------------------------------------------------
# Main
# ---------------------------------------------------------------
def main():
    if len(sys.argv) != 2:
        print("Usage: python monitor.py <COM_PORT>")
        print("Example: python monitor.py COM3")
        sys.exit(1)

    port = sys.argv[1]

    try:
        ser = serial.Serial(port, BAUD_RATE, timeout=TIMEOUT_RESPONSE)
        time.sleep(0.3)
        ser.reset_input_buffer()
    except serial.SerialException as e:
        print(f"ERR: failed to open {port}: {e}")
        sys.exit(1)

    log_filename = f"openbms_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
    log_file     = open(log_filename, 'w', newline='')
    log_writer   = csv.writer(log_file)
    log_writer.writerow(CSV_HEADER)

    print("\033[2J", end='', flush=True)
    print(f"Logging to: {log_filename}")
    time.sleep(0.5)

    iteration    = 0
    display_iter = 0
    errors       = 0
    last         = {}

    while True:

        if msvcrt.kbhit():
            if msvcrt.getwch().lower() == 'q':
                print(RESET + "\nStopped.")
                break

        t_start = time.time()

        # -------------------------------------------------------
        # Read all registers — 20 Hz
        # -------------------------------------------------------
        uptime          = read_uint64     (ser, REG_UPTIME                    )
        cell_v          = read_float_array(ser, REG_CELL_VOLTAGE,      7      )
        cell_v_filt     = read_float_array(ser, REG_CELL_VOLTAGE_FILT, 7      )
        pack_v          = read_float      (ser, REG_PACK_VOLTAGE               )
        pack_vf         = read_float      (ser, REG_PACK_VOLTAGE_FILT          )
        current         = read_float      (ser, REG_PACK_CURRENT               )
        curr_filt       = read_float      (ser, REG_PACK_CURRENT_FILT          )
        temp_c          = read_float      (ser, REG_TEMPERATURE                )
        fet_status      = read_uint16     (ser, REG_FET_STATUS                 )
        learning_status = read_uint16     (ser, REG_LEARNING_STATUS            )

        elapsed_ms = (time.time() - t_start) * 1000

        if any(v is None for v in [uptime, cell_v, cell_v_filt, pack_v, pack_vf,
                                    current, curr_filt, temp_c, fet_status, learning_status]):
            errors += 1

        # -------------------------------------------------------
        # Log to CSV — every read (20 Hz)
        # -------------------------------------------------------
        log_writer.writerow(make_csv_row(uptime, cell_v, cell_v_filt, pack_v, pack_vf,
                                         current, curr_filt, temp_c, fet_status, learning_status))
        log_file.flush()

        last = dict(uptime=uptime, cell_v=cell_v, cell_v_filt=cell_v_filt,
                    pack_v=pack_v, pack_vf=pack_vf,
                    current=current, curr_filt=curr_filt, temp_c=temp_c,
                    fet_status=fet_status, learning_status=learning_status,
                    elapsed_ms=elapsed_ms)

        # -------------------------------------------------------
        # Refresh display — every 4th read (5 Hz)
        # -------------------------------------------------------
        if display_iter % DISPLAY_EVERY == 0:

            v    = last['cell_v']
            vf   = last['cell_v_filt']
            pv   = last['pack_v']
            pvf  = last['pack_vf']
            cur  = last['current']
            curf = last['curr_filt']
            tc   = last['temp_c']
            up   = last['uptime']
            fet  = last['fet_status']
            ls   = last['learning_status']
            ela  = last['elapsed_ms']

            print(HOME, end='')

            ln(BOLD + CYAN + "=" * 72 + RESET)
            ln(f"  OpenBMS Monitor  —  {port}  —  20Hz read / 5Hz display  —  errors: {errors}")
            ln(BOLD + CYAN + "=" * 72 + RESET)

            # Uptime
            ln()
            if up is not None:
                h  = up // 3_600_000
                m  = (up % 3_600_000) // 60_000
                s  = (up % 60_000) // 1000
                ms = up % 1000
                ln(BOLD + f"  Uptime: {h:02d}:{m:02d}:{s:02d}.{ms:03d}  ({up} ms)" + RESET)
            else:
                ln(BOLD + "  Uptime: ERR" + RESET)

            # Cell voltages
            ln()
            ln(BOLD + "  Cell Voltages (mV)" + RESET)
            ln(DIM + f"  {'Cell':<6} {'Raw':>10}   {'Filtered':>10}   Bar (2500 — 4250 mV)" + RESET)
            ln(DIM + f"  {'-'*6}   {'-'*10}   {'-'*10}   {'-'*26}" + RESET)
            for i in range(7):
                vi  = v[i]  if v  else None
                vfi = vf[i] if vf else None
                ln(f"  {cell_color(vfi)}Cell {i+1}  {fmt_f(vi):>10}   {fmt_f(vfi):>10}   {bar(vfi, 2500, 4250)}{RESET}")

            # Pack voltage
            ln()
            ln(BOLD + "  Pack Voltage (mV)" + RESET)
            ln(f"  Raw:      {GREEN}{fmt_f(pv):>10} mV{RESET}   {bar(pv,  19600, 29400)}")
            ln(f"  Filtered: {GREEN}{fmt_f(pvf):>10} mV{RESET}   {bar(pvf, 19600, 29400)}")

            # Current
            ln()
            ln(BOLD + "  Current (mA)" + RESET)
            ln(f"  Raw:      {current_color(cur)}{fmt_f(cur):>10} mA{RESET}   {bar(cur,  -20000, 20000)}")
            ln(f"  Filtered: {current_color(curf)}{fmt_f(curf):>10} mA{RESET}   {bar(curf, -20000, 20000)}")

            # Temperature
            ln()
            ln(BOLD + "  Temperature" + RESET)
            ln(f"  Package:  {temp_color(tc)}{fmt_f(tc):>10} C{RESET}    {bar(tc, -20, 80)}")

            # Status flags
            ln()
            ln(BOLD + "  Status" + RESET)
            ln(f"  FET Status:       {CYAN}{fmt_h(fet)}{RESET}")
            ln(f"  Learning Status:  {CYAN}{fmt_h(ls)}{RESET}")

            # Footer
            ln()
            ln(DIM + f"  Read: {ela:.1f} ms  |  Log: {log_filename}  |  Press 'q' to quit" + RESET)

        iteration    += 1
        display_iter += 1

        sleep_s = POLL_INTERVAL_S - (time.time() - t_start)
        if sleep_s > 0:
            time.sleep(sleep_s)

    log_file.close()
    ser.close()


if __name__ == "__main__":
    main()
