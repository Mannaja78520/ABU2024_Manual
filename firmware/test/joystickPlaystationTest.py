import pygame
import numpy as np

pygame.init()
joystick = pygame.joystick.Joystick(0)
joystick.init()

def read_joystick():
    pygame.event.get()  # Update the joystick events

    # Analog0 = lx
    # Analog1 = ly
    # Analog2 = rx
    # Analog3 = ry
    # Analog4 = right_trigger
    # Analog5 = left_trigger

    # Digital0  = A
    # Digital1  = B
    # Digital2  = empty
    # Digital3  = X
    # Digital4  = Y
    # Digital5  = empty
    # Digital6  = left_Bumpper
    # Digital7  = right_Bumpper
    # Digital8  = left_Trigger
    # Digital9  = right_Trigger
    # Digital10 = screen
    # Digital11 = menu
    # Digital12 = fantech
    # Digital13 = leftStick_Button
    # Digital14 = rightStick_Button
    
    # Hat0 = right-left_Dpad
    # Hat1 = up-down_Dpad

    # Read analog joystick data
    analog_data = [int(round(joystick.get_axis(i), 2) * 100) for i in range(6)]
    analog_data[4] = int((analog_data[4] + 100) * 0.5)
    analog_data[5] = int((analog_data[5] + 100) * 0.5)

    # Read digital joystick data
    digital_data = [joystick.get_button(i) for i in [0, 1, 3, 4, 6, 7, 8, 9, 10, 11, 12, 13, 14]]

    # Read D-pad datasss
    rl_Dpad, ud_Dpad = joystick.get_hat(0)

    return analog_data, digital_data, [rl_Dpad, ud_Dpad]

while True:
    analogData, DigitalData, DpadData = read_joystick()
    data = analogData + DigitalData + DpadData
    print(data)
    # axis_lx, axis_ly, axis_rx, axis_ry, right_Trigger, left_Trigger = read_joystick()
    # print("Axis lx:", axis_lx, "Axis ly:", axis_ly, "Axis rx:", axis_rx, "Axis ry:", axis_ry, "left Trigger:", left_Trigger, "right Trigger:", right_Trigger)
