'''
David Pollard

FARSCOPE CDT
Bristol Robotics Laboratory

abb_testing.py
 - Simulates toolpath generated on ABB robot
 - Does not have full functionality.
 
Released under the MIT License



Original Release (MIT License):

Michael Dawson-Haggerty

abb.py: contains classes and support functions which interact with an ABB Robot running our software stack (RAPID code module SERVER)


For functions which require targets (XYZ positions with quaternion orientation),
targets can be passed as [[XYZ], [Quats]] OR [XYZ, Quats]

'''

import socket
import json 
import time
import inspect
from threading import Thread
from collections import deque
import logging

log = logging.getLogger(__name__)
log.addHandler(logging.NullHandler())
    
class Robot:
    def __init__(self, 
                 ip          = '192.168.125.1', 
                 port_motion = 5000,
                 port_logger = 5001):

        self.delay   = .08

        self.connect_motion((ip, port_motion))
        #log_thread = Thread(target = self.get_net, 
        #                    args   = ((ip, port_logger))).start()
        
        self.set_units('millimeters', 'degrees')
        self.set_tool()
        self.set_workobject()
        self.set_speed()
        self.set_zone()
        
    def reset_position(self, value = 0):
        '''
        Moves robot back to zeroing position, resets all signals
         - Pass in:
                -1  Disable, stay still
                0   Disable, reset
                1   Enable, reset
                2   Enable, stay still    
        '''
        msg = '-1 ' + str(int(value)) + ' #'
        self.send(msg)

    def connect_motion(self, remote):        
        log.info('Attempting to connect to robot motion server at %s', str(remote))
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.settimeout(2.5)
        self.sock.connect(remote)
        self.sock.settimeout(None)
        log.info('Connected to robot motion server at %s', str(remote))

    def connect_logger(self, remote, maxlen=None):
        self.pose   = deque(maxlen=maxlen)
        self.joints = deque(maxlen=maxlen)
        
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.connect(remote)
        s.setblocking(1)
        try:
            while True:
                data = map(float, s.recv(4096).split())
                if   int(data[1]) == 0: 
                    self.pose.append([data[2:5], data[5:]])
                #elif int(data[1]) == 1: self.joints.append([a[2:5], a[5:]])
        finally:
            s.shutdown(socket.SHUT_RDWR)

    def set_units(self, linear, angular):
        units_l = {'millimeters': 1.0,
                   'meters'     : 1000.0,
                   'inches'     : 25.4}
        units_a = {'degrees' : 1.0,
                   'radians' : 57.2957795}
        self.scale_linear = units_l[linear]
        self.scale_angle  = units_a[angular]

    def set_cartesian(self, pose, fineMotion = False):
        '''
        Executes a move immediately from the current pose,
        to 'pose', with units of millimeters.
        '''
        if fineMotion == True:
            msg  = "01 1 " + self.format_pose(pose)
        else:
            msg  = "01 0 " + self.format_pose(pose)

        return self.send(msg)

    def set_joints(self, joints):
        '''
        Executes a move immediately, from current joint angles,
        to 'joints', in degrees. 
        '''
        if len(joints) != 6: return False
        msg = "02 "
        for joint in joints: msg += format(joint*self.scale_angle, "+08.2f") + " " 
        msg += "#" 
        return self.send(msg)

    def get_cartesian(self):
        '''
        Returns the current pose of the robot, in millimeters
        '''
        msg = "03 #"
        data = self.send(msg).split()
        r = [float(s) for s in data]
        return [r[2:5], r[5:9]]

    def get_joints(self):
        '''
        Returns the current angles of the robots joints, in degrees. 
        '''
        msg = "04 #"
        data = self.send(msg).split()
        return [float(s) / self.scale_angle for s in data[2:8]]

    def get_external_axis(self):
        '''
        If you have an external axis connected to your robot controller
        (such as a FlexLifter 600, google it), this returns the joint angles
        '''
        msg = "05 #"
        data = self.send(msg).split()
        return [float(s) for s in data[2:8]]
       
    def get_robotinfo(self):
        '''
        Returns a robot- unique string, with things such as the
        robot's model number. 
        Example output from and IRB 2400:
        ['24-53243', 'ROBOTWARE_5.12.1021.01', '2400/16 Type B']
        '''
        msg = "98 #"
        data = str(self.send(msg))[5:].split('*')
        log.debug('get_robotinfo result: %s', str(data))
        return data

    def set_tool(self, tool=[[0,0,0], [1,0,0,0]]):
        '''
        Sets the tool centerpoint (TCP) of the robot. 
        When you command a cartesian move, 
        it aligns the TCP frame with the requested frame.
        
        Offsets are from tool0, which is defined at the intersection of the
        tool flange center axis and the flange face.
        '''
        msg       = "06 " + self.format_pose(tool)    
        self.send(msg)
        self.tool = tool

    def load_json_tool(self, file_obj):
        if file_obj.__class__.__name__ == 'str':
            file_obj = open(filename, 'rb');
        tool = check_coordinates(json.load(file_obj))
        self.set_tool(tool)
        
    def get_tool(self): 
        log.debug('get_tool returning: %s', str(self.tool))
        return self.tool

    def set_workobject(self, work_obj=[[0,0,0],[1,0,0,0]]):
        '''
        The workobject is a local coordinate frame you can define on the robot,
        then subsequent cartesian moves will be in this coordinate frame. 
        '''
        msg = "07 " + self.format_pose(work_obj)   
        self.send(msg)

    def set_speed(self, speed=[100,50,50,50]):
        '''
        speed: [robot TCP linear speed (mm/s), TCP orientation speed (deg/s),
                external axis linear, external axis orientation]
        '''
        if len(speed) != 4: return False
        msg = "08 " 
        msg += format(speed[0], "+08.1f") + " " 
        msg += format(speed[1], "+08.2f") + " "  
        msg += format(speed[2], "+08.1f") + " " 
        msg += format(speed[3], "+08.2f") + " #"     
        self.send(msg)
        
    def rotate_z(self, value):
        '''
        Rotates current tool the provided angle (degrees)
        '''
        msg = "10 "
        msg += format(value, ".2f") + " #"
        self.send(msg)
        
    def check_j6(self):
        '''
        Checks joint 6 value, and corrects if windup has occured
        '''
        msg = "11 #"
        self.send(msg)

    def set_zone(self, 
                 zone_key     = 'z1', 
                 point_motion = False, 
                 manual_zone  = []):
        zone_dict = {'z0'  : [.3,.3,.03], 
                    'z1'  : [1,1,.1], 
                    'z5'  : [5,8,.8], 
                    'z10' : [10,15,1.5], 
                    'z15' : [15,23,2.3], 
                    'z20' : [20,30,3], 
                    'z30' : [30,45,4.5], 
                    'z50' : [50,75,7.5], 
                    'z100': [100,150,15], 
                    'z200': [200,300,30]}
        '''
        Sets the motion zone of the robot. This can also be thought of as
        the flyby zone, AKA if the robot is going from point A -> B -> C,
        how close do we have to pass by B to get to C
        
        zone_key: uses values from RAPID handbook (stored here in zone_dict)
        with keys 'z*', you should probably use these

        point_motion: go to point exactly, and stop briefly before moving on

        manual_zone = [pzone_tcp, pzone_ori, zone_ori]
        pzone_tcp: mm, radius from goal where robot tool centerpoint 
                   is not rigidly constrained
        pzone_ori: mm, radius from goal where robot tool orientation 
                   is not rigidly constrained
        zone_ori: degrees, zone size for the tool reorientation
        '''

        if point_motion: 
            zone = [0,0,0]
        elif len(manual_zone) == 3: 
            zone = manual_zone
        elif zone_key in zone_dict.keys(): 
            zone = zone_dict[zone_key]
        else: return False
        
        msg = "09 " 
        msg += str(int(point_motion)) + " "
        msg += format(zone[0], "+08.4f") + " " 
        msg += format(zone[1], "+08.4f") + " " 
        msg += format(zone[2], "+08.4f") + " #" 
        self.send(msg)
        
    def check_position(self, pose):
        msg = "40 " + self.format_pose(pose)
        retVal = self.send(msg)
        if retVal[3] == 49:
            return True
        else:
            return False

    def buffer_add(self, pose):
        '''
        Appends single pose to the remote buffer
        Move will execute at current speed (which you can change between buffer_add calls)
        '''
        if len(pose)==2:
            msg = "30 " + self.format_pose(pose) 
        elif len(pose)==3:
            msg = "30 " + self.format_pos(pose) 
        else:
            raise Exception("Unexpected pose length")
        self.send(msg)

    def buffer_set(self, pose_list):
        '''
        Adds every pose in pose_list to the remote buffer
        '''
        self.clear_buffer()
        for pose in pose_list: 
            self.buffer_add(pose)
        if self.buffer_len() == len(pose_list):
            log.debug('Successfully added %i poses to remote buffer', 
                      len(pose_list))
            return True
        else:
            log.warn('Failed to add poses to remote buffer!')
            self.clear_buffer()
            return False
            
    def buffer_set_orientation(self, orientation):
        msg = "29 " + self.format_orient(orientation)
        self.send(msg)

    def clear_buffer(self):
        msg = "31 #"
        data = self.send(msg)
        if self.buffer_len() != 0:
            log.warn('clear_buffer failed! buffer_len: %i', self.buffer_len())
            raise NameError('clear_buffer failed!')
        return data

    def buffer_len(self):
        '''
        Returns the length (number of poses stored) of the remote buffer
        '''
        msg = "32 #"
        data = self.send(msg).split()
        return int(float(data[2]))

    def buffer_execute(self, extrudeOn = False):
        '''
        Immediately execute linear moves to every pose in the remote buffer.
         - passing True in will set DO_2 to high for length of movement (also moves up slightly afterwards)
        '''
        if extrudeOn:
            msg = "33 1 #"
        else:
            msg = "33 #"
        return self.send(msg)
        
    def buffer_execute_circ(self):
        '''
        Execute buffer in circular motion - NB: Only two poses on here
        '''
        msg = "37 #"
        return self.send(msg)
        
    # BUFFER SAVE FUNCTIONS
    def buffer_save(self, bufferNum):
        '''
        Saves buffer for future use
        '''
        msg = "50 "
        msg += format(bufferNum,"d") + " #"
        return self.send(msg)
        
    def buffer_load(self, bufferNum):
        '''
        Loads saved buffer
        '''
        msg = "51 "
        msg += format(bufferNum,"d") + " #"
        return self.send(msg)
        
    def buffer_read_value(self, value):
        '''
        Reads specific position (value) from buffer
         - NB: Not zero-indexed, start from 1. Use -1 for final value
        '''
        msg = "52 "
        msg += format(value,"d") + " #"
        data = self.send(msg).split()
        r = [float(s) for s in data]
        return [r[2:5], r[5:9]]
        
    def buffer_offset(self, xyz):
        '''
        Offsets entire buffer by XYZ values provided
        '''
        msg = "53 "
        msg += format(xyz[0], "+08.4f") + " "
        msg += format(xyz[1], "+08.4f") + " "
        msg += format(xyz[2], "+08.4f") + " #"
        self.send(msg)
        
    def buffer_modify_speed(self, value = 1):
        '''
        Scales TCP speed of current buffer by value
        '''
        msg = "54 "
        msg += format(value, "+08.4f") + " #"
        self.send(msg)
        

    def set_external_axis(self, axis_values=[0,0]):
        if len(axis_values) != 2: return False
        msg = "34 "
        for axis in axis_values:
            msg += format(axis, "+08.2f") + " " 
        msg += "#"   
        return self.send(msg)

    def move_circular(self, pose_onarc, pose_end):
        '''
        Executes a movement in a circular path from current position, 
        through pose_onarc, to pose_end
        '''
        print("Warning... move_circular might not work. Use buffer_set, and buffer_execute_circ instead")
        msg_0 = "35 " + self.format_pose(pose_onarc)  
        msg_1 = "36 " + self.format_pose(pose_end)

        data = self.send(msg_0).split()
        if data[1] != '1': 
            log.warn('move_circular incorrect response, bailing!')
            return False
        return self.send(msg_1)

    def set_dio(self, value, id=0):
        '''
        A function to set a physical DIO line on the robot.
        For this to work you're going to need to edit the RAPID function
        and fill in the DIO you want this to switch. 
        '''
        msg = '97 ' + str(int(bool(value))) + ' #'
        self.send(msg)
        return 
        #return self.send(msg)
        
    def set_go(self, value):
        '''
        Function sets Group Output value to the one specified.
        Does not check if it is within limits
        '''
        msg = '96 ' + str(int(value)) + ' #'
        self.send(msg)
        return
        
    def send(self, message, wait_for_response=True):
        '''
        Send a formatted message to the robot socket.
        if wait_for_response, we wait for the response and return it
        '''
        caller = inspect.stack()[1][3]
        log.debug('%-14s sending: %s', caller, message)
        self.sock.send(str.encode(message))
        time.sleep(self.delay)
        if not wait_for_response: return
        data = self.sock.recv(4096)
        log.debug('%-14s recieved: %s', caller, data)
        return data
        
    '''
    format_pose - formats entire pose for format [[x,y,z],[q1,q2,q3,q4]]
    format_pos - formats position [x,y,z]
    format_orient - formats orientation [q1,q2,q3,q4]
    '''
    def format_pose(self, pose):
        pose = check_coordinates(pose)
        msg  = ''
        for cartesian in pose[0]:
            msg += format(cartesian * self.scale_linear,  "+08.1f") + " " 
        for quaternion in pose[1]:
            msg += format(quaternion, "+08.5f") + " " 
        msg += "#" 
        return msg    

    def format_pos(self, pose):
        msg = ''
        for cartesian in pose:
            msg += format(cartesian * self.scale_linear, "08.1f") + " "
        msg += "#"
        return msg
        
    def format_orient(self, pose):
        msg = ''
        for quaternion in pose:
            msg += format(quaternion, "08.5f") + " "
        msg += "#"
        return msg
        
    def close(self):
        self.send("99 #", False)
        self.sock.shutdown(socket.SHUT_RDWR)
        self.sock.close()
        log.info('Disconnected from ABB robot.')

    def __enter__(self):
        return self
        
    def __exit__(self, type, value, traceback):
        self.close()

def check_coordinates(coordinates):
    if ((len(coordinates) == 2) and
        (len(coordinates[0]) == 3) and 
        (len(coordinates[1]) == 4)): 
        return coordinates
    elif (len(coordinates) == 7):
        return [coordinates[0:3], coordinates[3:7]]
    log.warn('Recieved malformed coordinate: %s', str(coordinates))
    raise NameError('Malformed coordinate!')

if __name__ == '__main__':
    formatter = logging.Formatter("[%(asctime)s] %(levelname)-7s (%(filename)s:%(lineno)3s) %(message)s", "%Y-%m-%d %H:%M:%S")
    handler_stream = logging.StreamHandler()
    handler_stream.setFormatter(formatter)
    handler_stream.setLevel(logging.DEBUG)
    log = logging.getLogger('abb')
    log.setLevel(logging.DEBUG)
    log.addHandler(handler_stream)
    
