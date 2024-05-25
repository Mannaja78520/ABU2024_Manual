import subprocess
import time

def run_main_script():
    while True:
        try:
            subprocess.run(['python3', 'firmware/test/ZigbeeJoystickXbox.py'])  # Replace 'python' with 'python3' if necessary
        except Exception as e:
            print("Error occurred:", e)
            print("Restarting the main script in 5 seconds...")
            time.sleep(1)

if __name__ == "__main__":
    run_main_script()