import pygame
import serial
import time

def initialize_serial(port, baud_rate):
    try:
        ser = serial.Serial(port, baud_rate)
        time.sleep(2)
        return ser
    except serial.SerialException:
        return None

def receive_data(ser):

    received_data = ser.readline().strip().decode()
    print("Received data :", received_data)
    ser.reset_input_buffer()

    return received_data
        
ser = initialize_serial('/dev/ttyUSB0', 230400)
if not ser:
    print("Zigbee port not found. Exiting...")
    exit()

while True:
    receive_data(ser)
    #time.sleep(0.05)
