from controller import Controller
import time

c = Controller(0, 0.001)

while True:
    print(c.Calculate(0.0005))
    print()
    print(c.Integral)
    print()
    # print(c.Dt)
    time.sleep(0.1)