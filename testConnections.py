'''
Test functions
'''

import sys
import time
from random import random

defaultMode = 1


def main(modeSelection):

    # Simulation or physical?
    if modeSelection == 1:
        ip = "127.0.0.1"
    else:
        ip = "192.168.125.1"
    
    
    R = test_initialisation(ip)

    test_extAx(R)
    test_buffer_functions(R)
    test_IO(R)
    
    R.reset_position(0)
    
    if modeSelection == 0:
        R.show_motions()
    
    
    
def test_initialisation(ip):
    print("Beginning initialisation")
    R = abb.Robot(ip)
    print(" - connection OK")
    
    R.set_tool([[0,0,50],[1,0,0,0]])
    print(" - tool set")
    
    R.set_workobject([[600,0,300],[1,0,0,0]])
    print(" - WObj set")
    
    R.set_speed([100,500,500,500])
    print(" - speed set")
    
    R.set_joints([0,0,0,0,0,0])
    print(" - joints reset")
    
    return R

    
    
def test_extAx(R):
    print("External axis:")
    R.set_external_axis([30, 60])
    print(" - Initial movement 1")
    time.sleep(5)
    
    extAx = R.get_external_axis()
    print(" - extAx reading:")
    print(extAx)
    
    R.set_external_axis([0,0])
    time.sleep(5)
    print(" - Returned position")
    
    extAx = R.get_external_axis()
    print(" - extAx reading:")
    print(extAx)
    
    
    
    
def test_buffer_functions(R):
    print("Testing buffer functions")

    qVal = [0,0,1,0]
    bufferXYZ = [[0,0,0],[50,0,0],[50,50,0],[50,50,50],[0,0,0]]
    bufferPose = [[xyz, qVal] for xyz in bufferXYZ]
    
    R.buffer_set(bufferPose)
    print(" - buffer set")
    
    R.buffer_save(1)
    print(" - buffer saved")
    
    bufferPose2 = bufferPose[::-1]
    R.buffer_set(bufferPose2)
    R.buffer_save(2)
    print(" - buffer reversed, loaded, and saved")
    
    R.buffer_load(1)
    print(" - buffer 1 loaded")
    
    R.buffer_execute(0)
    print(" - buffer 1 executed")
    
    R.buffer_load(2)
    R.buffer_execute(0)
    print(" - buffer 2 loaded and executed")
    
    R.buffer_load(1)
    R.buffer_offset([10,10,10])
    R.buffer_modify_speed(0.5)
    R.buffer_execute(1)
    print(" - buffer 1 loaded, speed modified, executed as print")
    
    
    

def test_IO(R):
    print("Testing I/O")
    R.reset_position(1)
    print(" - reset, enabled: DO10_1 = 1")
    time.sleep(5)
    R.reset_position(0)
    print(" - reset, disabled: DO10_1 = 0")
    time.sleep(5)
    
    R.set_dio(1)
    print(" - DIO 2 enabled")
    time.sleep(5)
    
    R.set_dio(0)
    print(" - DIO 2 disabled")
    time.sleep(5)
    
    for i in range(3):
        value = int(random()*255)
        R.set_go(value)
        print(" - GO value set to %d"%value)
        time.sleep(5)
    
    

    
    
if __name__ == "__main__":
    cmdArgs = sys.argv
    modeSelection = defaultMode
    if len(cmdArgs) == 2:
        modeSelection = int(cmdArgs[1])
    else:
        modeSelection = defaultMode
        
    # select testing (0), simulation (1), or robot (2)
    if modeSelection ==  0:
        print("Testing mode")
        import abb_testing as abb
    elif modeSelection == 1 or modeSelection== 2:
        print("Sim or Running mode")
        import abb
    else:
        raise Exception("Unknown input passed in")
    main(modeSelection)