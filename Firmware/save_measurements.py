import serial
import serial.tools.list_ports
import re
import threading
import time
import csv

def find_serial_port():
    ports = serial.tools.list_ports.comports()
    print("Available serial ports:")
    for i, port in enumerate(ports):
        print(f"{i+1}. {port.device} - {port.description}")
    if not ports:
        print("No serial ports found!")
        return None
    for port in ports:
        if any(keyword in port.description.lower() for keyword in ['usb', 'arduino', 'esp', 'cp210', 'ch340']):
            print(f"Auto-selected: {port.device}")
            return port.device
    print(f"Using first available port: {ports[0].device}")
    return ports[0].device

port = find_serial_port()
if port is None:
    exit("No serial port available")

try:
    ser = serial.Serial(port, 115200, timeout=1)
    print(f"Connected to {port} successfully!")
except serial.SerialException as e:
    print(f"Error connecting to {port}: {e}")
    exit()

# Data storage: list of [duty, l1, l2, l3, vin, vdc]
records = []
running = True

def read_serial_data():
    while running:
        try:
            line = ser.readline().decode('utf-8').strip()
            if line:
                print(f"Received: '{line}'")
                # Adjust regex to your format
                duty = re.search(r'Duty Cycle:\s*(\d+)', line)
                l1 = re.search(r'Corrente L1:\s*([\d.]+)', line)
                l2 = re.search(r'Corrente L2:\s*([\d.]+)', line)
                l3 = re.search(r'Corrente L3:\s*([\d.]+)', line)
                vin = re.search(r'Tensao Entrada[:\s]*([\d.]+)', line)
                vdc = re.search(r'DC Link[:\s]*([\d.]+)', line)
                
                # Check if at least one field was found
                if any([duty, l1, l2, l3, vin, vdc]):
                    record = [
                        int(duty.group(1)) if duty else '',
                        float(l1.group(1)) if l1 else '',
                        float(l2.group(1)) if l2 else '',
                        float(l3.group(1)) if l3 else '',
                        float(vin.group(1)) if vin else '',
                        float(vdc.group(1)) if vdc else ''
                    ]
                    records.append(record)
        except Exception as e:
            print(f"Serial read error: {e}")

def send_serial_data(message):
    try:
        if ser.is_open:
            ser.write(f"{message}\n".encode('utf-8'))
            print(f"Sent: '{message}'")
        else:
            print("Serial port is not open!")
    except Exception as e:
        print(f"Error sending data: {e}")

def user_interface():
    global running
    print("\n=== Serial Command Interface ===")
    print("Type commands to send via serial (or 'quit' to exit)")
    print("Type 'S' to save data to CSV")
    print("=====================================\n")
    while running:
        try:
            command = input("Serial> ").strip()
            if command.lower() == 'quit':
                running = False
                break
            elif command.lower() == 's':
                save_to_csv()
            elif command:
                send_serial_data(command)
        except (KeyboardInterrupt, EOFError):
            running = False
            break
    print("Command interface closed.")

def save_to_csv():
    if not records:
        print("No data to save.")
        return
    filename = time.strftime("serial_log_%Y%m%d_%H%M%S.csv")
    with open(filename, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['Duty', 'L1', 'L2', 'L3', 'Vin', 'Vdc'])
        writer.writerows(records)
    print(f"Saved {len(records)} records to {filename}")

# Start serial reading in background
serial_thread = threading.Thread(target=read_serial_data, daemon=True)
serial_thread.start()

# Start user interface in main thread
user_interface()

ser.close()
print("Serial port closed.")