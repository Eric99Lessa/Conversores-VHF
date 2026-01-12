import serial
import time
import csv
import re
import threading
import sys
import os

PORT = '/dev/ttyACM0'  # Change to your port
BAUD = 115200

# Regex patterns for parsing telemetry (CSV format from updated firmware)
telem_re = re.compile(r'^TELEM,(\d+),([0-9.\-]+),([0-9.\-]+),([0-9.\-]+),([0-9.\-]+),([0-9.\-]+),([0-9.\-]+),(\d+),(\d+),(\d+)$')
cmd_ok_re = re.compile(r'CMD_OK')
cmd_err_re = re.compile(r'CMD_ERR')

# Global state
recording = False
show_prints = False  # Start with prints hidden
recording_lock = threading.Lock()
print_lock = threading.Lock()
running = True

def send_command(ser, cmd):
    """Send a command to the ESP32 via serial"""
    try:
        ser.write((cmd + '\n').encode('utf-8'))
        print(f"[SENT] {cmd}")
    except Exception as e:
        print(f"[ERROR] Could not send command: {e}")

def show_help():
    """Display help menu"""
    print("\n" + "="*50)
    print("  COMMAND MENU")
    print("="*50)
    print("  H                    - Show this help")
    print("  R                    - Start recording to CSV")
    print("  P                    - Pause recording")
    print("  Hide                 - Stop showing ESP32 prints")
    print("  Show                 - Show ESP32 prints")
    print("  D <duty>             - Set all 3 phases to <duty> (0-1000)")
    print("  D <u> <v> <w>        - Set U, V, W duty cycles individually")
    print("  M serial             - Change mode to Serial duty control")
    print("  M PI                 - Change mode to PI control")
    print("  M IDAPBC             - Change mode to IDA-PBC control")
    print("  Q                    - Quit program")
    print("="*50 + "\n")

def user_input_thread(ser_container):
    """Thread to handle user keyboard input"""
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
            
            # Mode commands - send to ESP32
            if cmd_upper.startswith('M '):
                mode_arg = user_input[2:].strip().upper()
                
                ser = ser_container['ser']
                if not ser or not ser.is_open:
                    print("[ERROR] No device connected. Command ignored.")
                    continue
                
                if mode_arg == 'SERIAL':
                    send_command(ser, 'M serial')
                elif mode_arg == 'PI':
                    send_command(ser, 'M PI')
                elif mode_arg == 'IDAPBC':
                    send_command(ser, 'M IDAPBC')
                else:
                    print(f"[ERROR] Unknown mode '{mode_arg}'. Use: serial, PI, or IDAPBC")
                continue
            
            # Duty commands - forward to ESP32
            if cmd_upper.startswith('D '):
                ser = ser_container['ser']
                if ser and ser.is_open:
                    send_command(ser, user_input)
                else:
                    print("[ERROR] No device connected. Command ignored.")
                continue
            
            # Unknown command
            print(f"[ERROR] Unknown command '{user_input}'. Type 'H' for help.")
        
        except EOFError:
            running = False
            break
        except Exception as e:
            print(f"[INPUT ERROR] {e}")

def main():
    global recording, show_prints, running
    
    # Ask for filename
    print("="*50)
    csv_filename = input("Enter CSV filename (default: telemetry_log.csv): ").strip()
    if not csv_filename:
        csv_filename = 'telemetry_log.csv'
    
    if not csv_filename.endswith('.csv'):
        csv_filename += '.csv'
    
    print(f"Logging to: {csv_filename}")
    print("="*50 + "\n")
    
    # Container to share the serial object between threads
    ser_container = {'ser': None}
    
    # Start user input thread
    input_thread = threading.Thread(target=user_input_thread, args=(ser_container,), daemon=True)
    input_thread.start()
    
    # Open CSV and write header
    file_exists = os.path.isfile(csv_filename)
    f = open(csv_filename, mode='a', newline='')
    writer = csv.writer(f)
    if not file_exists:
        writer.writerow(['t_rel_s', 'seq', 'x0', 'x1', 'x2', 'x3', 'x4', 'x5', 'duty_u', 'duty_v', 'duty_w'])

    t0 = time.perf_counter()

    print(f"Waiting for device on {PORT}...")

    while running:
        # 1. Connection Logic
        if ser_container['ser'] is None or not ser_container['ser'].is_open:
            try:
                ser_container['ser'] = serial.Serial(PORT, BAUD, timeout=0.1)
                print(f"\n[CONNECTED] Device found on {PORT}")
            except (serial.SerialException, FileNotFoundError):
                time.sleep(1)
                continue

        # 2. Read Logic
        try:
            ser = ser_container['ser']
            line_bytes = ser.readline()
            if not line_bytes:
                continue
            
            line = line_bytes.decode('utf-8', errors='replace').strip()
            if not line:
                continue
            
            # Check if it's telemetry data (CSV format)
            m_telem = telem_re.match(line)
            if m_telem:
                seq = int(m_telem.group(1))
                x0, x1, x2, x3, x4, x5 = [float(m_telem.group(i)) for i in range(2, 8)]
                duty_u, duty_v, duty_w = [int(m_telem.group(i)) for i in range(8, 11)]
                
                # Show if prints are enabled
                with print_lock:
                    if show_prints:
                        print(f"[TELEM] seq={seq} x=[{x0:.3f} {x1:.3f} {x2:.3f} {x3:.3f} {x4:.3f} {x5:.3f}] duty=[{duty_u} {duty_v} {duty_w}]")
                
                # Write to CSV if recording
                with recording_lock:
                    if recording:
                        t_rel = time.perf_counter() - t0
                        writer.writerow([
                            f"{t_rel:.6f}",
                            seq,
                            f"{x0:.6f}", f"{x1:.6f}", f"{x2:.6f}",
                            f"{x3:.6f}", f"{x4:.6f}", f"{x5:.6f}",
                            duty_u, duty_v, duty_w
                        ])
                        f.flush()
                continue
            
            # Check for command acknowledgment
            if cmd_ok_re.search(line):
                print(f"[ESP32] {line}")
                continue
            
            if cmd_err_re.search(line):
                print(f"[ESP32] {line}")
                continue
            
            # Show other ESP32 logs if prints are enabled
            with print_lock:
                if show_prints:
                    print(f"[ESP32] {line}")

        except serial.SerialException:
            print("\n[DISCONNECTED] Device lost. Waiting for reconnect...")
            if ser_container['ser']:
                ser_container['ser'].close()
            ser_container['ser'] = None
            time.sleep(1)

    # Cleanup on exit
    if ser_container['ser'] and ser_container['ser'].is_open:
        ser_container['ser'].close()
    f.close()
    print(f"\nData saved to {csv_filename}")
    print("Program exited cleanly.")

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        running = False
        print("\nStopped by user.")