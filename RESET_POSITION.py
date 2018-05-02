'''
Resetting robot position
 - Command arguements:
 - - 0      Immediate robot reset
 - - 1      Reset simulation on 127.0.0.1
 [other]    Reset robot in 5 seconds
'''
import abb
import time
import sys
from Robot_Config import robot_config

modeSelection = 2
cmdArgs = sys.argv
if len(cmdArgs)==2:
    modeSelection = int(cmdArgs[1])

    
    
if modeSelection == 1:
    R = abb.Robot("127.0.0.1")
    print("Robot resetting")
elif modeSelection == 0:
    R = abb.Robot("192.168.125.1")
    print("Robot resetting now")
elif modeSelection == 2:
    R = abb.Robot("192.168.125.1")
    print("Robot moving backwards and then resetting")
    currPos = R.get_cartesian()
    currPos[0][0] -= 100
    R.set_cartesian(currPos)
else:
    R = abb.Robot("192.168.125.1")
    print("---- WARNING ----")
    print(" Robot will move suddenly in 5 seconds")
    print(" Press CTRL+C to cancel move")
    print("-----------------")
    time.sleep(5)


#R.set_workobject(robot_config.get_wobj())
R.set_tool([[0,0,0],[1,0,0,0]])


R.reset_position(0)