import copy

class robot_config:
    #
    # Base parameters
    #
    Tool = [[-44.51, 9.9, 113.95],[1,0,0,0]]
    FauxTool = [[-47.7913, 12.961, 103.954],[1,0,0,0]]
    
    # Initialise
    WObjData = []
    ExtAxData = []
    InitJointData = []

    
    # Flat plate
    WObjData.append([[563.1723, -92.69622, 290.004],[1,0,0,0]])
    ExtAxData.append([0,0])
    InitJointData.append([0,17,8,0,65,0])
    
    # 45 degrees
    WObjData.append([[555.8, -127.3, 383.6],[0.9235949, -0.38097291, 0.005997098517, 0.013148388823]])
    ExtAxData.append([45,0])
    InitJointData.append([16, 31, -8, -43, 84, 26])
    
    # 90 degrees
    WObjData.append([[564.949, -73.7138, 475.236],[0.70710678118654757, -0.70710678118654746, 0.0, 0.0]])
    ExtAxData.append([90,0])
    InitJointData.append([21, 29, -1, 90, -90, -70])
    
    
    # 135 degrees
    WObjData.append([[540.0918, -382.8466, 767.9519],[0.27059805007309851, -0.65328148243818829, -0.65328148243818829, -0.27059805007309845]])
    ExtAxData.append([45,0])
    InitJointData.append([-11.3, 30, -35, -130, 78, 5])
    
    # 180 degrees
    WObjData.append([[543.6473, -0.8785222, 925.7554],[0,0.707106781,0.707106781,0]])
    ExtAxData.append([90,0])
    InitJointData.append([18.8, 10, -5, -166, 66.9, 8])
    
    # Rotated toolplate
    WObjData.append([[633.5, 9.87, 380],[1,0,0,0]])
    ExtAxData.append([0,0])
    InitJointData.append([30,20,30,90,-80,60])
    
    #
    # Return functions
    #
    @classmethod
    def get_tool(robot_config, UseFauxNozzle = False):
        if UseFauxNozzle:
            print("Using nozzle model")
            return robot_config.FauxTool
        else:
            print("Using actual printhead")
            return robot_config.Tool
            
    @classmethod
    def get_wobj(robot_config,z_offset = 0, WObjNumber = 0):
        if WObjNumber < len(robot_config.WObjData):
            return robot_config.WObjData[WObjNumber]
        else:
            raise Exception("Unrecognised WObj Number")

        
    @classmethod
    def get_extax(robot_config,WObjNumber = 0):
        if WObjNumber < len(robot_config.ExtAxData):
            return robot_config.ExtAxData[WObjNumber]
        else:    
            raise Exception("Unrecognised external axis")
            
    @classmethod
    def get_initJoint(robot_config, WObjNumber = 0):
        if WObjNumber <len(robot_config.InitJointData):
            return robot_config.InitJointData[WObjNumber]
        else:
            raise Exception("Unrecognised initial joint position number")