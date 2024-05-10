import pygame
import serial
import time
import threading

# Global variables
joystick = None
joystick_initialized = False
last_analog_data = None
last_Analogdata_time = None
last_data = None

def initialize_joystick():
    pygame.init()
    num_joysticks = pygame.joystick.get_count()
    if num_joysticks == 0:
        print("No joystick found.")
        return None
    elif num_joysticks > 1:
        print("Multiple joysticks found. Please connect only one joystick.")
        return None
    else:
        joystick = pygame.joystick.Joystick(0)
        joystick.init()
        print("Joystick found:", joystick.get_name())
        return joystick

def initialize_serial(port, baud_rate):
    try:
        ser = serial.Serial(port, baud_rate)
        time.sleep(2)
        return ser
    except serial.SerialException:
        return None

def read_joystick(joystick):
    pygame.event.get()  # Update the joystick events

    # Read analog joystick data
    analog_data = [int(round(joystick.get_axis(i), 2) * 100) for i in range(6)]
    analog_data[2] = int((analog_data[2] + 100) * 0.5)
    analog_data[5] = int((analog_data[5] + 100) * 0.5)
    for i in range(6):
        if abs(analog_data[i]) < 8:
            analog_data[i] = 0

    # Read digital joystick data
    digital_data = [joystick.get_button(i) for i in range(12)]

    # Read D-pad datasss
    rl_Dpad, ud_Dpad = joystick.get_hat(0)

    return analog_data, digital_data, [rl_Dpad, ud_Dpad]

def calculate_checksum(data):
    checksum = sum(int(abs(x)) for x in data)
    return checksum

def send_data_to_arduino(ser, data):
    checksum = calculate_checksum(data)
    data_with_checksum = [checksum] + data
    data_str = ','.join(map(str, data_with_checksum)) + '\n'
    ser.write(data_str.encode())
    print("Sent data to Arduino:", data_with_checksum)

def receive_data_from_arduino(ser):
    received_data = ser.readline().strip().decode()
    print("Received data from Arduino:", received_data)

    return received_data
        

def joystick_thread():
    global joystick
    global joystick_initialized

    if not joystick:
        joystick = initialize_joystick()
        joystick_initialized = True

joystick_thread = threading.Thread(target=joystick_thread)
joystick_thread.start()

ser = initialize_serial('/dev/ttyUSB0', 115200)
if not ser:
    print("Zigbee port not found. Exiting...")
    exit()

while not joystick_initialized:
    time.sleep(1)

while True:
    if receive_data_from_arduino(ser):
        analog_data, digital_data, dpad_data = read_joystick(joystick)
        data_to_send = analog_data + digital_data + dpad_data

        send_data_to_arduino(ser, data_to_send)
        last_data = data_to_send
        last_analog_data = analog_data
