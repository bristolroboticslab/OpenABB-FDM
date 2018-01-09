'''
David Pollard

FARSCOPE CDT
Bristol Robotics Laboratory

abb_testing.py
 - Simulates toolpath generated on ABB robot
 - Does not have full functionality.
 
Released under the MIT License


'''

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import warnings
import copy

warnings.filterwarnings("ignore",".*GUI is implemented.*")


class Robot:
    currPosition = [[0,0,0],[1,0,0,0]]
    currWObj = [[0,0,0],[1,0,0,0]]
    currTool = [[0,0,0],[1,0,0,0]]
    bufferPose = []
    currDIO = False
    travelList = [[]]
    speedVsFeedIdx = []
    currGO = -1
    GO_List = [currGO]
    savedBuffers = []
    
    OK_MSG = "b'-1' b'1'"
    
    plotTravelMotions = True
    # Animate printing - put negative for instant output
    animatePrinting =  0.05         
    
    def __init__(self, ip):
        print("Imaginary robot created")

    def reset_position(self, setEnable = 0):
        if setEnable == -1:
            print("Extrusion disabled")
        elif setEnable == 0:
            print("Position reset, extrusion disabled")
        elif setEnable == 1:
            print ("Position reset, extrusion enabled")
        elif setEnable == 2:
            print("Extrusion enabled")
        else:
            raise Exception("Unrecognised reset command input")
            
        
    def close(self):
        pass

    def set_cartesian(self, pose, fineMotion = False):
        self.currPosition = pose
        self.travelList[-1].append(pose)
        
    def set_joints(self, joints):
        pass

    def get_cartesian(self):
        return Robot.currPosition
        
    def get_joints(self):
        return None
        
    def get_external_axis(self):
        return None
        
    def set_tool(self, ToolObj = [[0,0,0],[1,0,0,0]]):
        pass         
        
    def set_workobject(self, WObj = [[0,0,0],[1,0,0,0]]):
        pass 
        
    def set_speed(self, speed=[100,50,50,50]):
        pass
        
    def rotate_z(self, value):
        pass
        
    def check_j6(self):
        pass
        
    def set_zone(self, zone_key = 'z1'):
        pass
        
    def check_position(self, pose):
        return True

        
    def buffer_set(self, poseList):
        if len(poseList[0]) == 2:           # sending full poses
            self.bufferPose = poseList
        elif len(poseList[0]) == 3:        # only sending positions
            self.bufferPose = [[poseList[i], self.qOrientation] for i in range(0, len(poseList))]
        else:
            raise Exception("Improper pose length")

    def buffer_set_orientation(self, value):
        self.qOrientation = value
        
    def buffer_execute(self, extrudeOn = False):
        if extrudeOn:
            self.set_cartesian(self.bufferPose[0])
            self.set_dio(1)
        for pose in self.bufferPose:
            self.travelList[-1].append(pose)
            self.currPosition = pose
        if extrudeOn:
            self.set_dio(0)

    def buffer_execute_circ(self):
        if len(self.bufferPose)!=2:
            raise Exception('Invalid Circle')
        for pose in self.bufferPose:
            self.travelList[-1].append(pose)
            self.currPosition = pose

    def buffer_save(self, value):
        if value > len(self.savedBuffers):
            self.savedBuffers.append(copy.deepcopy(self.bufferPose))
        else:
            self.savedBuffers[value-1] = copy.deepcopy(self.bufferPose)
        return self.OK_MSG
            
    def buffer_load(self, value):
        self.bufferPose = copy.deepcopy(self.savedBuffers[value-1])
        return self.OK_MSG
        
        
    def buffer_read_value(self, value):
        return(self.bufferPose[value-1])

    def buffer_offset(self, xyz):
        for i,[pos,q] in enumerate(self.bufferPose):
            pos = [pos[0]+xyz[0], pos[1]+xyz[1], pos[2]+xyz[2]]
            self.bufferPose[i][0] = pos
            
    def buffer_modify_speed(self, value):
        pass
        
        
        
    def set_external_axis(self, axis_values=[0,0]):
        if len(axis_values) != 2:
            raise Exception("Unsuitable external axis setting")
        
    def move_circular(self, poseCentre, poseEnd):
        self.currPosition = poseEnd
        # NB: Could create a circular path for better representation?
        self.travelList[-1].append(poseCentre)
        self.travelList[-1].append(poseEnd)
        self.currPosition = poseEnd
        
    def set_dio(self, value, id=0):
        if bool(value) is not self.currDIO:
            self.currDIO = bool(value)
            self.travelList.append([self.currDIO])
            self.travelList[-1].append(self.currPosition)
            
        if value == 0:
            self.GO_List.append(0) # Travel speed at index 0
        else:
            self.GO_List.append(self.currGO) # Other speed set before extrusion commanded
        
    def set_go(self, value):
        self.currGO = value   
        

    def update_speed_matrix(self, matrix):
        '''
        Warning... This isn't implemented on abb.py
        '''
        self.speedVsFeedIdx = matrix
        

        
    '''
    Final display function...
    '''
    def show_motions(self, animate = True):
        self.animatePrinting = animate
        fig = plt.figure()
        ax = fig.add_subplot(111, projection = '3d')

            
        # Plot extrusion lines
        for line in self.travelList:
            if len(line) > 0:
                xVec = [pos[0][0] for pos in line[1:]]
                yVec = [pos[0][1] for pos in line[1:]]
                zVec = [pos[0][2] for pos in line[1:]]

                if line[0]:
                    ax.plot(xVec, yVec, zVec, 'b')
                elif self.plotTravelMotions:
                    ax.plot(xVec, yVec, zVec, 'c')

                    
                if self.animatePrinting > 0:
                    fig.show()
                    plt.pause(self.animatePrinting)
            


        fig.show()


if __name__=='__main__':
    print("You shouldn't have clicked F5.")
