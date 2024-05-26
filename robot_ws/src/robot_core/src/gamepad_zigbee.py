import serial
import time

class gamepad_Zigbee:
    lx = ly = rx = ry = 0
    right_trigger = left_trigger = 0
    
    A = B = X = Y = left_bumper = False
    right_bumper = screen = menu = logo = False
    left_stick_button = right_stick_button = upload = False
    dpad_right = dpad_left = dpad_up = dpad_down = False
    
    last_lx = last_ly = last_rx = last_ry = 0
    last_rt = last_lt = 0
    
    last_A = last_B = last_X = last_Y = last_lb = False
    last_rb = last_s = last_m = last_f = last_lsb = False
    last_rsb = last_u = False
    
    last_dr = last_dl = last_du = last_dd = False
    
    CurrentTime = time.time()
    
    have_data_from_controller = False
    
    received_data = ''
    port = ''
    baud_rate = 9600
    
    def Reset(self):
        # Joystick variables
        self.lx = self.ly = self.rx = self.ry = 0
        self.right_trigger = self.left_trigger = 0

        self.A = self.B = self.X = self.Y = self.left_bumper = False
        self.right_bumper = self.screen = self.menu = self.logo = False
        self.left_stick_button = self.right_stick_button = self.upload = False

        self.dpad_right = self.dpad_left = self.dpad_up = self.dpad_down = False
        
        # Last data variables
        self.last_lx = self.last_ly = self.last_rx = self.last_ry = 0
        self.last_rt = self.last_lt = 0

        self.last_A = self.last_B = self.last_X = self.last_Y = self.last_lb = False
        self.last_rb = self.last_s = self.last_m = self.last_f = self.last_lsb = False
        self.last_rsb = self.last_u = False

        self.last_dr = self.last_dl = self.last_du = self.last_dd = False
        self.CurrentTime = time.time()
        self.last_data_time = time.time()
        
        
    def __init__(self, port = '/dev/ttyUSB0', baud_rate = 9600):
        self.Reset()
        self.ser = self.initialize_serial(port, baud_rate)
        self.port = port
        self.baud_rate = baud_rate
        
    def initialize_serial(self, port, baud_rate):
        try:
            ser = serial.Serial(port, baud_rate)
            # time.sleep(2)
            return ser
        except serial.SerialException:
            return None
        
    def reset_variable(self):
        self.lx = 0
        self.ly = 0
        self.rx = 0
        self.ry = 0
        self.right_trigger = 0
        self.left_trigger = 0

        self.A = 0
        self.B = 0
        self.X = 0
        self.Y = 0
        self.left_bumper = 0
        self.right_bumper = 0
        self.screen = 0
        self.menu = 0
        self.logo = 0
        self.left_stick_button = 0
        self.right_stick_button = 0
        self.upload = 0

        self.dpad_right = False
        self.dpad_left = False
        self.dpad_up = False
        self.dpad_down = False
        
    def use_last_data(self):
        self.lx = self.last_lx
        self.ly = self.last_ly
        self.rx = self.last_rx
        self.ry = self.last_ry
        self.right_trigger = self.last_rt
        self.left_trigger = self.last_lt

        self.A = self.last_A
        self.B = self.last_B
        self.X = self.last_X
        self.Y = self.last_Y
        self.left_bumper = self.last_lb
        self.right_bumper = self.last_rb
        self.screen = self.last_s
        self.menu = self.last_m
        self.logo = self.last_f
        self.left_stick_button = self.last_lsb
        self.right_stick_button = self.last_rsb
        self.upload = self.last_u

        self.dpad_right = self.last_dr
        self.dpad_left = self.last_dl
        self.dpad_up = self.last_du
        self.dpad_down = self.last_dd
        
    def update_last_data(self):
        self.last_lx = self.lx
        self.last_ly = self.ly
        self.last_rx = self.rx
        self.last_ry = self.ry
        self.last_rt = self.right_trigger
        self.last_lt = self.left_trigger
        
        self.last_A = self.A
        self.last_B = self.B
        self.last_X = self.X
        self.last_Y = self.Y
        self.last_lb = self.left_bumper
        self.last_rb = self.right_bumper
        self.last_s = self.screen
        self.last_m = self.menu
        self.last_f = self.logo
        self.last_lsb = self.left_stick_button
        self.last_rsb = self.right_stick_button
        self.last_u = self.upload

        self.last_dr = self.dpad_right
        self.last_dl = self.dpad_left
        self.last_du = self.dpad_up
        self.last_dd = self.dpad_down
        
    def receive_data(self):
        self.reset_variable()
        self.CurrentTime = time.time()
        expected_data_length = 40
        checksum = -1
        try:
            if self.ser is None:
                print("Serial is not initialized. Attempting to initialize...")
                self.ser = self.initialize_serial(self.port, self.baud_rate)
                if self.ser is None:
                    print("Failed to initialize serial. Cannot receive data.")
                    return
            self.have_data_from_controller = self.ser.in_waiting
            if self.have_data_from_controller > 0:
                # reset_variables()
                self.last_data_time = self.CurrentTime
                received_bytes = self.ser.readline().strip()
                try:
                    self.received_data = received_bytes.decode('utf-8')
                except UnicodeDecodeError as e:
                    print("UnicodeDecodeError:", e)
                    # Handle the error gracefully, e.g., skip this iteration
                    return
                if expected_data_length <= len(self.received_data) <= 70:
                    tokens = self.received_data.split(",")
                    try:
                        index = 0
                        sum_of_data = 0
                        for token in tokens[:21]:
                            data = int(token) if token else 0
                            sum_of_data += abs(data)
                            if index == 0:
                                checksum = data
                                sum_of_data = 0
                            elif index == 1:
                                self.lx = data / 100.0
                            elif index == 2:
                                self.ly = data / 100.0
                            elif index == 3:
                                self.left_trigger = data / 100.0
                            elif     index == 4:
                                self.rx = data / 100.0
                            elif index == 5:
                                self.ry = data / 100.0
                            elif index == 6:
                                self.right_trigger = data / 100.0
                            elif index == 7:
                                self.A = data
                            elif index == 8:
                                self.B = data
                            elif index == 9:
                                self.X = data
                            elif index == 10:
                                self.Y = data
                            elif index == 11:
                                self.left_bumper = data
                            elif index == 12:
                                self.right_bumper = data
                            elif index == 13:
                                self.screen = data
                            elif index == 14:
                                self.menu = data
                            elif index == 15:
                                self.logo = data
                            elif index == 16:
                                self.left_stick_button = data
                            elif index == 17:
                                self.right_stick_button = data
                            elif index == 18:
                                self.upload = data
                            elif index == 19:
                                right_left_dpad = data
                            elif index == 20:
                                up_down_dpad = data
                            index += 1
                    except ValueError:
                        print("Error converting data to integer. Skipping...")
                        self.use_last_data()
                        
                    # Discard any remaining data in the buffer
                    self.ser.reset_input_buffer()

                    received_checksum = abs(sum_of_data - checksum)
                    if received_checksum == 0:
                        # Debugging: Print received data
                        # print("Data Match")
                        self.dpad_right = right_left_dpad == 1
                        self.dpad_left = right_left_dpad == -1
                        self.dpad_up = up_down_dpad == 1
                        self.dpad_down = up_down_dpad == -1
                        # compare_data(self.CurrentTime)
                        self.update_last_data()
                        return
                    print("Checksum mismatchnot ")
                    self.ser.readline().decode('utf-8').rstrip()
                    self.use_last_data()
                    return
                print("Incomplete data received ")
                self.ser.readline().decode('utf-8').rstrip()
                self.use_last_data()
                return
            if self.CurrentTime - self.last_data_time < 0.3:
                self.have_data_from_controller = True
                self.use_last_data()
                return
            self.have_data_from_controller = False
        except OSError as e:
            print("Serial communication error: " + str(e))
            # Retry logic
            self.have_data_from_controller = False
            self.ser.close()
            time.sleep(1)  # Wait for 1 second before retrying
            self.ser = self.initialize_serial(self.port, self.baud_rate)
            if self.ser is None:
                print("Failed to initialize serial. Cannot retry.")
                return
            self.receive_data()  # Retry communication