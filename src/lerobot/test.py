from pymycobot import ElephantRobot,MyArmC,utils
from lerobot.robots.mycobot.elegripper import Gripper
import time
arm=ElephantRobot("10.194.13.177",5001)# 将ip更改成Pro630树莓派的实时ip
arm.start_client()# 启动机器人必要指令
time.sleep(1)
# while arm.state_check()==False:
#     arm.state_off()
#     time.sleep(2)
#     arm.power_on()
#     time.sleep(2)
#     arm.state_on()
#     time.sleep(2)
#     print(arm.state_check())

# arm.set_gripper_mode(0)
gripper = Gripper("com7", 115200, id=14)
time.sleep(0.3)
c=MyArmC("COM3", 1000000)
fact_angle = [0, 0, 0, 0, 0, 0]

def jointlimit(angles):
  max = [180.0, 90.0, 150.0, 80.0, 168.0, 175.0]
  min = [-180.0, -270, -150.0, -260.0, -168.0, -175.0]
  for i in range(6):
    if(angles[i] > max[i]):
       angles[i] = max[i]
    if(angles[i] < min[i]):
       angles[i] = min[i]

# try:
while 1:

    angle=c.get_joints_angle()
    if len(angle)==7:
        fact_angle[0] = angle[0]
        fact_angle[1] = angle[1] - 100
        fact_angle[2] = angle[2] + 100
        fact_angle[4] = -angle[3] - 90
        fact_angle[5] = angle[5] - 120
        if angle[4] < 90:
            fact_angle[3] = 90 - angle[4]
            fact_angle[3] = -90 - fact_angle[3]
        elif angle[4] > 90:
            fact_angle[3] = 90 - angle[4]
            fact_angle[3] = -90 + fact_angle[3]
        else:
            fact_angle[3] = -angle[4]
        for i in range(len(fact_angle)):
            fact_angle[i]=round(fact_angle[i],2)
        jointlimit(fact_angle)
        grip_value = int(-angle[6])
        if grip_value < 0:
            grip_value = 0
        if grip_value > 80:
            grip_value = 100
        fact_angle[4] = -90
        arm.write_angles(fact_angle,5999)
        gripper.set_gripper_value(grip_value, 100)
        time.sleep(0.2) #4hz  #50hz
    else:
        print("None")
# except:
#     arm.stop_client()
#     print("end")