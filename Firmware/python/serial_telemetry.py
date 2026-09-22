import serial
import time
import csv
import threading
import os

PORT = '/dev/ttyACM0'  # Change to your port
BAUD = 115200

TELEM_PREFIX = 'TELEM,'

# Global state
recording = False
show_prints = False
csv_header_written = False
recording_lock = threading.Lock()
print_lock = threading.Lock()
running = True


def parse_telem(line):
    """
    Parses: TELEM,<seq>,<f0>,<f1>,...
    Returns (seq, values_list) or None on failure.
    Format: seq, x[SIG_COUNT], z[Z_COUNT], d_bot[PHASES_COUNT], d_top[PHASES_COUNT]
    """
    if not line.startswith(TELEM_PREFIX):
        return None
    try:
        parts = line.split(',')
        seq = int(parts[1])
        values = [float(p) for p in parts[2:]]
        return seq, values
    except (ValueError, IndexError):
        return None


def send_command(ser, cmd):
    """Send a command to the ESP32 via serial."""
    try:
        ser.write((cmd + '\n').encode('utf-8'))
        print(f"[SENT] {cmd}")
    except Exception as e:
        print(f"[ERROR] Could not send command: {e}")


def show_help():
    """Display help menu."""
    print("\n" + "=" * 50)
    print("  COMMAND MENU")
    print("=" * 50)
    print("  H                    - Show this help")
    print("  R                    - Start recording to CSV")
    print("  P                    - Pause recording")
    print("  Hide                 - Stop showing ESP32 prints")
    print("  Show                 - Show ESP32 prints")
    print("  D <duty>             - Set all 3 phases to <duty> (0-1000 permille)")
    print("  D <u> <v> <w>        - Set U, V, W duty cycles individually")
    print("  M open               - Change mode to open loop")
    print("  M cascade            - Change mode to cascade (PI) control")
    print("  M idapbc             - Change mode to IDA-PBC control")
    print("  V <float>            - Set Vout reference")
    print("  I <float>            - Set IL reference")
    print("  Q                    - Quit program")
    print("=" * 50 + "\n")


def user_input_thread(ser_container):
    """Thread to handle user keyboard input."""
    global recording, show_prints, running

    show_help()

    while running:
        try:
            user_input = input().strip()

            if not user_input:
                continue

            cmd_upper = user_input.upper()

            # Help
            if cmd_upper == 'H':
                show_help()
                continue

            # Quit
            if cmd_upper == 'Q':
                print("Shutting down...")
                running = False
                break

            # Recording control
            if cmd_upper == 'R':
                with recording_lock:
                    recording = True
                print("[RECORDING STARTED]")
                continue

            if cmd_upper == 'P':
                with recording_lock:
                    recording = False
                print("[RECORDING PAUSED]")
                continue

            # Print visibility
            if cmd_upper == 'HIDE':
                with print_lock:
                    show_prints = False
                print("[PRINTS HIDDEN]")
                continue

            if cmd_upper == 'SHOW':
                with print_lock:
                    show_prints = True
                print("[PRINTS VISIBLE]")
                continue

            ser = ser_container['ser']

            # Mode commands — forward to ESP32 (open|cascade|idapbc)
            if cmd_upper.startswith('M '):
                mode_arg = user_input[2:].strip().lower()
                if not ser or not ser.is_open:
                    print("[ERROR] No device connected. Command ignored.")
                    continue
                if mode_arg in ('open', 'cascade', 'idapbc'):
                    send_command(ser, f'M {mode_arg}')
                else:
                    print(f"[ERROR] Unknown mode '{mode_arg}'. Use: open, cascade, idapbc")
                continue

            # Duty command — forward raw to ESP32 (D <float> or D <f0> <f1> <f2>)
            if cmd_upper.startswith('D '):
                if not ser or not ser.is_open:
                    print("[ERROR] No device connected. Command ignored.")
                    continue
                send_command(ser, user_input)
                continue

            # Vout reference
            if cmd_upper.startswith('V '):
                if not ser or not ser.is_open:
                    print("[ERROR] No device connected. Command ignored.")
                    continue
                send_command(ser, user_input)
                continue

            # IL reference
            if cmd_upper.startswith('I '):
                if not ser or not ser.is_open:
                    print("[ERROR] No device connected. Command ignored.")
                    continue
                send_command(ser, user_input)
                continue

            print(f"[ERROR] Unknown command '{user_input}'. Type 'H' for help.")

        except EOFError:
            running = False
            break
        except Exception as e:
            print(f"[INPUT ERROR] {e}")


def main():
    global recording, show_prints, running, csv_header_written

    print("=" * 50)
    csv_filename = input("Enter CSV filename (default: telemetry_log.csv): ").strip()
    if not csv_filename:
        csv_filename = 'telemetry_log.csv'
    if not csv_filename.endswith('.csv'):
        csv_filename += '.csv'
    print(f"Logging to: Tests/{csv_filename}")
    print("=" * 50 + "\n")

    ser_container = {'ser': None}

    input_thread = threading.Thread(target=user_input_thread, args=(ser_container,), daemon=True)
    input_thread.start()

    tests_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'Tests')
    os.makedirs(tests_dir, exist_ok=True)
    csv_filepath = os.path.join(tests_dir, csv_filename)
    f = open(csv_filepath, mode='a', newline='')

    writer = csv.writer(f)
    # Header is written dynamically on the first received frame (column count depends on firmware constants)
    csv_header_written = os.path.isfile(csv_filepath) and os.path.getsize(csv_filepath) > 0

    t0 = time.perf_counter()
    print(f"Waiting for device on {PORT}...")

    while running:
        # ── Connection ────────────────────────────────────────────────────────
        if ser_container['ser'] is None or not ser_container['ser'].is_open:
            try:
                ser_container['ser'] = serial.Serial(PORT, BAUD, timeout=0.1)
                print(f"\n[CONNECTED] Device found on {PORT}")
            except (serial.SerialException, FileNotFoundError):
                time.sleep(1)
                continue

        # ── Read ──────────────────────────────────────────────────────────────
        try:
            ser = ser_container['ser']
            line_bytes = ser.readline()
            if not line_bytes:
                continue

            line = line_bytes.decode('utf-8', errors='replace').strip()
            if not line:
                continue

            # Telemetry frame
            telem = parse_telem(line)
            if telem is not None:
                seq, values = telem
                n = len(values)

                # Write CSV header on first frame
                if not csv_header_written:
                    header = ['t_rel_s', 'seq'] + [f'v{i}' for i in range(n)]
                    writer.writerow(header)
                    f.flush()
                    csv_header_written = True

                with print_lock:
                    if show_prints:
                        vals_str = ' '.join(f'{v:.3f}' for v in values)
                        print(f"[TELEM] seq={seq} [{vals_str}]")

                with recording_lock:
                    if recording:
                        t_rel = time.perf_counter() - t0
                        writer.writerow([f"{t_rel:.6f}", seq] + [f"{v:.6f}" for v in values])
                        f.flush()
                continue

            # Command acknowledgment
            if 'CMD_OK' in line or 'CMD_ERR' in line:
                print(f"[ESP32] {line}")
                continue

            # Other ESP32 log output
            with print_lock:
                if show_prints:
                    print(f"[ESP32] {line}")

        except serial.SerialException:
            print("\n[DISCONNECTED] Device lost. Waiting for reconnect...")
            if ser_container['ser']:
                ser_container['ser'].close()
            ser_container['ser'] = None
            time.sleep(1)

    # ── Cleanup ───────────────────────────────────────────────────────────────
    if ser_container['ser'] and ser_container['ser'].is_open:
        ser_container['ser'].close()
    f.close()
    print(f"\nData saved to Tests/{csv_filename}")
    print("Program exited cleanly.")


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        running = False
        print("\nStopped by user.")