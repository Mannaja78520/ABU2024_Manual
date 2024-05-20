import pygame
import numpy as np

pygame.init()
joystick = pygame.joystick.Joystick(0)
joystick.init()

def read_joystick():
    pygame.event.get()  # Update the joystick events

    # Analog0 = lx
    # Analog1 = ly
    # Analog2 = left_trigger
    # Analog3 = rx
    # Analog4 = ry
    # Analog5 = right_trigger

    # Digital0  = A
    # Digital1  = B
    # Digital2  = X
    # Digital3  = Y
    # Digital4  = left_Bumpper
    # Digital5  = right_Bumpper
    # Digital6 = screen
    # Digital7 = menu
    # Digital8 = xbox
    # Digital9 = leftStick_Button
    # Digital10 = rightStick_Button
    # Digital11 = upload
    
    # Hat0 = right-left_Dpad
    # Hat1 = up-down_Dpad

    analog_data = [int(round(joystick.get_axis(i), 2) * 100) for i in range(6)]
    analog_data[2] = int((analog_data[2] + 100) * 0.5)
    analog_data[5] = int((analog_data[5] + 100) * 0.5)

    digital_data = [joystick.get_button(i) for i in range(12)]

    rl_Dpad, ud_Dpad = joystick.get_hat(0)

    # print("Analog data:", analog_data)
    # print("Digital data:", digital_data)
    # print(joystick.get_hat(0))

    return analog_data, digital_data, [rl_Dpad, ud_Dpad]

while True:
    analogData, DigitalData, DpadData = read_joystick()
    data = analogData + DigitalData + DpadData
    print(data)