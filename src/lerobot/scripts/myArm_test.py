from pymycobot import MyArmC
import time


myarm = MyArmC("COM6", 115200, debug=True)

time.sleep(2)
print("--")
print(myarm.get_robot_error_status())
print(myarm.get_servos_status())