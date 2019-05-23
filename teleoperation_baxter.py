#!/usr/bin/env python

"""
This code is part of the research project
"DE VITO: A Dual-arm, High Degree-of-freedom, Lightweight, Inexpensive, Passive Upper-limb Exoskeleton for Robot Teleoperation"
This code was written, edited and documented by:
- Kawin Larppichet (Imperial College London, Robot Intelligence Lab)
- Fabian Falck (Imperial College London, Robot Intelligence Lab)
For correspondence on this project, please open an Issue on Github.
Further details can be found at http://www.imperial.ac.uk/robot-intelligence/robots/de_vito/.


General remarks on this script:
This python script will create ROS node "teleoperation" that subscribes to exo_info and controls the Baxter robot with the selected mode.
The 3 involved steps are:
   1. Read all exoskeleton data
   2. Calculate the mapped baxter robot joint angles from the given controlling mode
   3. Control the Baxter robot with all given information
"""


import argparse
import rospy
import baxter_interface
import baxter_external_devices
from baxter_interface import CHECK_VERSION
import numpy as np
import math 
import serial
import time
from exoskeleton.msg import exo_info
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Int32MultiArray
from geometry_msgs.msg import Twist
from baxter_pykdl import baxter_kinematics
from tf.transformations import euler_from_quaternion, quaternion_from_euler

pi=math.pi
max_lin=0.8
max_ang=3

des_baxter_state=np.zeros((1,14))
act_baxter_state=np.zeros((1,14))

# enter your desired mode here 0=joint mapping 1=Catesian mapping with end effectors pointing toward each others 2=Catesian mapping with end effectors pointing downward
mode=0

if mode==0:
        #elbow down posture
        r_last_command_baxter_angles=[ 0.2, 0.5, 3, 1.5 ,0, 1.2,pi]
        l_last_command_baxter_angles=[-0.2, 0.5, -3, 1.5 ,0, 1.2,-pi]
elif mode ==1:
        #elbow half postur
        r_last_command_baxter_angles=[  -0.5, -0.6, 1.5, 1.2, 0.5, 1.6,-pi/2]
        l_last_command_baxter_angles=[   0.5, -0.6, -1.5, 1.2 ,-0.5, 1.6,pi/2]
elif mode ==2:
        #elbow up posture
        r_last_command_baxter_angles=[ 0.3, -0.9, 0.8, 1.2 , -0.6, 1.2,0]
        l_last_command_baxter_angles=[-0.4, -0.9, -0.8, 1.2 , 0.6, 1.2,0]

# Set up the range of motion of each Exoskeleton joints   [minimum angle, maximum angle, sign]
rl_min_max=[[-30,75,1]
          ,[0,90,1]
          ,[-110,45,0]
          ,[-5,115,0]
          ,[-150,25,0]
          ,[-27,55,0]
          ,[-30,90,0]
          ,[-75,30,1]
          ,[0,90,1]
          ,[-45,110,0]
          ,[-5,115,0]
          ,[-150,25,0]
          ,[-40,27,0]
          ,[-90,20,0]]

# Set up the range of motion of each Baxter robot joints   [minimum angle, maximum angle]
baxter_min_max=[[-97,97]
               ,[-123,60]
               ,[-174,174]
               ,[-2.8,150]
               ,[-175,175]
               ,[-90,120]
               ,[-175,175]
               ,[-97,97]
               ,[-123,60]
               ,[-174,174]
               ,[-2.8,150]
               ,[-175,175]
               ,[-90,120]
               ,[-175,175]]


def set_j(limb, joint_name, angle):
    """
    Commands the joints of this limb to the specified positions by using pure mapped exoskeleton angles

    :param limb: (str:Limb) Interface class for a limb on the Baxter robot
    :param joint_name: (dict({str})) - joint_name
    :param angle: (list({float})) - angle command
    :return: -
    """
    joint_command = dict(zip(joint_name,angle))
    limb.set_joint_positions(joint_command)


def set_pose(limb,kin,jointname,end_point,rpy,elbow,rl,mode):
    """
    Commands the joints of this limb to the specified positions by using the end effector pose and elbow angle

    :param limb: (str:Limb) Interface class for a limb on the Baxter robot
    :param kin: (str:Limb) baxter kinematics class
    :param jointname: (dict({str})) - joint_name
    :param end_point: (list({float})) - end effector positions
    :param rpy: (list({float})) - orientations
    :param elbow: float - elbow joint angles
    :param rl: int - specifiy either left or right limb to be controlled ( 0 = right, 1 = left )
    :param mode: int - mapping algorithm mode 
    :return: -
    """
    global r_last_command_baxter_angles
    global l_last_command_baxter_angles

    if mode==1: #elbow lying in the horizontal axis and both end effectors are pointing toward each others

            if rl==0:
                    quat = quaternion_from_euler (rpy[0]+pi/2,-rpy[2]+pi/2,rpy[1],axes='rzyz') #right
            else:
                    quat = quaternion_from_euler (rpy[0]-pi/2,rpy[2]+pi/2,-rpy[1],axes='rzyz') #left
            current_pose=limb.endpoint_pose()
            position=current_pose['position']
            orientation=current_pose['orientation']

            #adding the ofset to map 2 different range of motions from exoskeleton and Baxter robot
            if rl==0:
                    x=0.16+end_point[0]
                    y=(end_point[1]+0.2)*1.25-0.2
                    z=0.55+end_point[2]
            else:
                    x=0.16+end_point[0]
                    y=(end_point[1]-0.2)*1.25+0.2
                    z=0.55+end_point[2]
            o_x=quat[0]
            o_y=quat[1]
            o_z=quat[2]
            o_w=quat[3]
            new_pose=[x,y,z]
            new_rot=[o_x,o_y,o_z,o_w]

            if rl==0:
                    angles=kin.inverse_kinematics(new_pose,new_rot,r_last_command_baxter_angles) #,r_last_command_baxter_angles
            else:
                    angles=kin.inverse_kinematics(new_pose,new_rot,l_last_command_baxter_angles)
            jacob=kin.jacobian()
            new_jacob=np.concatenate((jacob,[[0,0,1,0,0,0,0]]), axis=0)
            jinv=np.linalg.inv(new_jacob)

            # rotates the elbow of the Baxter to match the desired pose from exoskeleton
            if rl==0:
                    speed_in_null_space=np.matrix([0,0,0,0,0,0,(elbow-angles[2]-pi/2)*0.5]) #right
            else:
                    speed_in_null_space=np.matrix([0,0,0,0,0,0,(elbow-angles[2]+pi/2)*0.5]) #left

            #get angular speed for rotating each joints to match the desired pose
            angular_speed=np.matmul(jinv,speed_in_null_space.transpose())
            new_angles=np.squeeze([(a+b) for a, b in zip(angles,angular_speed)])

    elif mode==2: #elbow joints are pointing upward and the end effectors are pointing downward

            if rl==0:
                    quat = quaternion_from_euler (rpy[0]-pi,-rpy[1],-rpy[2]+pi,axes='rzyx')   #right
            else:
                    quat = quaternion_from_euler (rpy[0]-pi,-rpy[1],-rpy[2]-pi,axes='rzyx')   #left
            current_pose=limb.endpoint_pose()
            position=current_pose['position']
            orientation=current_pose['orientation']

            #adding the ofset to map 2 different range of motions from exoskeleton and Baxter robot
            x=0.24+end_point[0]
            y=end_point[1]
            z=0.5+end_point[2]

            o_x=quat[0]
            o_y=quat[1]
            o_z=quat[2]
            o_w=quat[3]

            new_pose=[x,y,z]
            new_rot=[o_x,o_y,o_z,o_w]

            if rl==0:
                    angles=kin.inverse_kinematics(new_pose,new_rot,r_last_command_baxter_angles) #,r_last_command_baxter_angles
            else:
                    angles=kin.inverse_kinematics(new_pose,new_rot,l_last_command_baxter_angles)

            jacob=kin.jacobian()
            new_jacob=np.concatenate((jacob,[[0,0,1,0,0,0,0]]), axis=0)
            jinv=np.linalg.inv(new_jacob)

            # rotates the elbow of the Baxter to match the desired pose from exoskeleton
            if rl==0:
                    speed_in_null_space=np.matrix([0,0,0,0,0,0,(elbow-angles[2]-pi*0.45)*0.8]) #right
            else:
                    speed_in_null_space=np.matrix([0,0,0,0,0,0,(elbow-angles[2]+pi*0.45)*0.8]) #left

            #get angular speed for rotating each joints to match the desired pose
            angular_speed=np.matmul(jinv,speed_in_null_space.transpose())
            new_angles=np.squeeze([(a+b) for a, b in zip(angles,angular_speed)])

    if new_angles is not None:
            if rl==0:
                    if all(i-j <= 1 and i-j >= -1  for i,j in zip(new_angles,r_last_command_baxter_angles)):
                            filtered_angles_r=[(a + 9*b)/10 for a, b in zip(new_angles,r_last_command_baxter_angles)]
                            set_j(limb,jointname,filtered_angles_r)
                            r_last_command_baxter_angles[:]=filtered_angles_r[:]
            else:
                    if all(i-j <= 1 and i-j >= -1  for i,j in zip(new_angles,l_last_command_baxter_angles)):
                            filtered_angles_l=[(a + 9*b)/10 for a, b in zip(new_angles,l_last_command_baxter_angles)]
                            set_j(limb,jointname,filtered_angles_l)
                            l_last_command_baxter_angles[:]=filtered_angles_l[:]


def move_catesian(limb_r,limb_l,kin_r,kin_l,jointname_r,jointname_l,dx,dy,dz):
    """
    Incrementally move the Baxter end effectors in X Y Z direction controlled by the joy stick of Nunchuk controller

    :param limb_r: (str:Limb) Interface class for a limb on the Baxter robot
    :param limb_l: (str:Limb) Interface class for a limb on the Baxter robot
    :param kin_r: (str:Limb) baxter kinematics class
    :param kin_l: (str:Limb) baxter kinematics class
    :param jointname_r: (dict({str})) - joint_name
    :param jointname_l: (dict({str})) - joint_name
    :param dx: float - displacement in x axis
    :param dy: float - displacement in y axis
    :param dz: float - displacement in z axis
    :return: -
    """
    global r_last_command_baxter_angles
    global l_last_command_baxter_angles
    ##Right

    #get current pose from Baxter
    current_pose=limb_r.endpoint_pose()
    position=current_pose['position']
    orientation=current_pose['orientation']
    x=position[0]
    y=position[1]
    z=position[2]
    o_x=orientation[0]
    o_y=orientation[1]
    o_z=orientation[2]
    o_w=orientation[3]

    #adding the values from joystick
    new_x=x+dx
    new_y=y+dy
    new_z=z+dz
    new_pose=[new_x,new_y,new_z]
    print(new_pose)
    new_rot=[o_x,o_y,o_z,o_w]

    #calculate the inverse kinematics from a new desired position
    angles_r=kin_r.inverse_kinematics(new_pose,new_rot,r_last_command_baxter_angles) #,r_last_command_baxter_angles

    ##Left

    #get current pose from Baxter
    current_pose=limb_l.endpoint_pose()
    position=current_pose['position']
    orientation=current_pose['orientation']
    x=position[0]
    y=position[1]
    z=position[2]
    o_x=orientation[0]
    o_y=orientation[1]
    o_z=orientation[2]
    o_w=orientation[3]

    #adding the values from joystick
    new_x=x+dx
    new_y=y-dy
    new_z=z+dz
    new_pose=[new_x,new_y,new_z]
    new_rot=[o_x,o_y,o_z,o_w]

    #calculate the inverse kinematics from a new desired position
    angles_l=kin_l.inverse_kinematics(new_pose,new_rot,l_last_command_baxter_angles)

    #Filtered out the new ikine solution to smooth the motion
    filtered_angles_r=[(a + 19*b)/20 for a, b in zip(angles_r.tolist(),r_last_command_baxter_angles)]
    filtered_angles_l=[(a + 19*b)/20 for a, b in zip(angles_l.tolist(),l_last_command_baxter_angles)]

    #set the new joint positions
    if filtered_angles_r is not None:
            set_j(limb_r,jointname_r,filtered_angles_r)
            r_last_command_baxter_angles[:]=filtered_angles_r[:]
    if filtered_angles_l is not None:
            set_j(limb_l,jointname_l,filtered_angles_l)
            l_last_command_baxter_angles[:]=filtered_angles_l[:]


def r_exo_baxter_map(angles):
    """
    Maps the joint positions of the right arm of exoskeleton to Baxter arm joint positions with a one to one mapping

    :param angles: (list({float})) - Exoskeleton angles
    :return: (list({float})) - Baxter angles
    """
    #one to one mapping
    mapped_angles=np.zeros((7))
    mapped_angles[0]= -angles[0]+pi/6
    mapped_angles[1]= -angles[1]+pi/2
    mapped_angles[2]= angles[2]+pi*5/6
    mapped_angles[3]= angles[3]
    mapped_angles[4]= angles[4]
    mapped_angles[5]= angles[5]+pi/2
    mapped_angles[6]= angles[6]-pi+pi/2

    return mapped_angles


def r_exo_baxter_map_ex(angles):
    """
    Maps the joint positions of the right arm of exoskeleton to Baxter arm joint positions with the Scaling factor mapping

    :param angles: (list({float})) - Exoskeleton angles
    :return: (list({float})) - Baxter angles
    """
        
    #the scaling factor can be modified by changing the multiplying number on each individual angle
    mapped_angles=np.zeros((7))
    mapped_angles[0]= -angles[0]+pi/4
    mapped_angles[1]= -(angles[1]-pi/4)*1.2-pi/4+pi/2
    mapped_angles[2]= angles[2]*1.5+pi
    mapped_angles[3]= angles[3]
    mapped_angles[4]= angles[4]*1.3
    mapped_angles[5]= angles[5]*1.5+pi/3
    mapped_angles[6]= angles[6]*1.5-pi+pi/2

    return mapped_angles


def l_exo_baxter_map(angles):
    """
    Maps the joint positions of the left arm of exoskeleton to Baxter arm joint positions with a one to one mapping

    :param angles: (list({float})) - Exoskeleton angles
    :return: (list({float})) - Baxter angles
    """
    #one to one mapping
    mapped_angles=np.zeros((7))
    mapped_angles[0]= -angles[0]-pi/6
    mapped_angles[1]= -angles[1]+pi/2
    mapped_angles[2]= angles[2]-pi*5/6
    mapped_angles[3]= angles[3]
    mapped_angles[4]= angles[4]
    mapped_angles[5]= angles[5]+pi/2
    mapped_angles[6]= angles[6]+pi-pi/2

    return mapped_angles


def l_exo_baxter_map_ex(angles):
    """
    Maps the joint positions of the left arm of exoskeleton to Baxter arm joint positions with the Scaling factor mapping

    :param angles: (list({float})) - Exoskeleton angles
    :return: (list({float})) - Baxter angles
    """
    #the scaling factor can be modified by changing the multiplying number on each individual angle
    mapped_angles=np.zeros((7))
    mapped_angles[0]= -angles[0]-pi/4
    mapped_angles[1]= -(angles[1]-pi/4)*1.2-pi/4+pi/2
    mapped_angles[2]= angles[2]*1.5-pi
    mapped_angles[3]= angles[3]
    mapped_angles[4]= angles[4]*1.3
    mapped_angles[5]= angles[5]*1.5+pi/3
    mapped_angles[6]= angles[6]*1.5+pi-pi/2

    return mapped_angles


def threshold(a,b):
    """
    Thresholding a value of a by the absolute value of b

    :param a: float - value to be compared
    :param b: float - value to be cut off
    :return: thresholded value of a by an absolute value of b
    """

    if b<0:
            b=-1*b
    if a<=b and a>=-b:
            a=0
    return a


def get_state(data,limb_r,limb_l,des_baxter_state,act_baxter_state):
    """
    Getting all mapped angles from given exoskeleton message

    :param data: exoskeleton message
    :param limb_r: (str:Limb) Interface class for a limb on the Baxter robot
    :param limb_l: (str:Limb) Interface class for a limb on the Baxter robot
    :param des_baxter_state: (list({float})) - mapped exoskeleton angles
    :param act_baxter_state: (list({float})) - Baxter angles
    :return: (list({float})) - mapped exoskeleton angles + (list({float})) - Baxter angles
    """

    #postural mapping
    ##
    ## Change r_exo_baxter_map_ex,l_exo_baxter_map_ex to r_exo_baxter_map,l_exo_baxter_map to get a one to one mapping algorithm
    ##
    des_baxter_angles=np.concatenate([r_exo_baxter_map_ex( data.exo_angles[0:7]),l_exo_baxter_map_ex( data.exo_angles[7:14])]) #convert exo angles to baxter angles


    for i in range(14):
            des_baxter_state[0,i]=des_baxter_angles[i]                                      #assign new desired state

    #read actual baxter states
    r_unordered_angles = limb_r.joint_angles()
    r_angles=[]
    [r_angles.extend([v]) for k,v in r_unordered_angles.items()]
    r_arranged_angles=r_angles[0:2]+r_angles[5:7]+r_angles[2:5]

    l_unordered_angles = limb_l.joint_angles()
    l_angles=[]
    [l_angles.extend([v]) for k,v in l_unordered_angles.items()]
    l_arranged_angles=l_angles[5:7]+l_angles[3:5]+l_angles[0:3]

    #pack all the states
    arranged_angles=r_arranged_angles+l_arranged_angles

    act_baxter_state[0,:]=arranged_angles[:]

    return des_baxter_state,act_baxter_state


def listener():  # TODO function in function?
    """
    Subscribes to topic exo_info

    :return: -
    """

    def map_exo(data):
        """
        Teleoperatates the Baxter robot

        :param data: exo_info message
        :return: -
        """

        global r_last_command_baxter_angles
        global l_last_command_baxter_angles
        global mode
        global max_lin
        global max_ang
        global des_baxter_state
        global act_baxter_state

        [des_baxter_state,act_baxter_state]=get_state(data,right,left,des_baxter_state,act_baxter_state)
        r_end_point=data.end_point[0:3]
        l_end_point=data.end_point[3:6]
        r_rpy=data.rpy[0:3]
        l_rpy=data.rpy[3:6]

        l_z_button=data.buttons[0]
        l_c_button=data.buttons[1]
        r_z_button=data.buttons[2]
        r_c_button=data.buttons[3]  

        r_ax=data.analog[0]/128.0
        r_ay=data.analog[1]/128.0
        l_ax=data.analog[2]/128.0
        l_ay=data.analog[3]/128.0
        imu=data.imu*1.0
        imu=threshold(imu,1)

        ##right arm

        #moving a manipulator if c button of the right Nunchuk is being pressed
        if r_c_button==1:
                if mode==0:
                        set_j(right, rj, des_baxter_state[0,0:7])
                        r_last_command_baxter_angles[:]=des_baxter_state[0,0:7]
                elif (mode==1 or mode==2) :
                        set_pose(right,kin_r,rj,r_end_point,r_rpy,des_baxter_state[0,2],0,mode)
        else :
                set_j(right, rj, r_last_command_baxter_angles)
         
        #Closing the right gripper if z button of the right Nunchuk is being pressed
        if r_z_button==1:
                grip_right.close()
        else:
                grip_right.open()
        
        ##left arm

        #moving a manipulator if c button of the left Nunchuk is being pressed
        if l_c_button==1:
                if mode==0:
                        set_j(left, lj, des_baxter_state[0,7:14])
                        l_last_command_baxter_angles[:]=des_baxter_state[0,7:14]
                elif (mode==1 or mode==2) :
                        set_pose(left,kin_l,lj,l_end_point,l_rpy,des_baxter_state[0,9],1,mode)
        else :
                set_j(left, lj, l_last_command_baxter_angles)
        
        #Closing the left gripper if z button of the right Nunchuk is being pressed
        if l_z_button==1:
                grip_left.close()
        else:
                grip_left.open()
        
        #move both arms in catesian frame by the left joy stick of the Nunchuk
        if l_ay!=0 or l_ax!=0:
                move_catesian(right,left,kin_r,kin_l,rj,lj,0,-l_ax*0.01,l_ay*0.01)

        #drive a mobile base by a right joy stick of the Nunchuk + the yaw rotation can also be controlled by the imu sensor of the exoskeleton
        r_ax=r_ax*max_ang*10
        r_ay=r_ay*max_lin*10
        imu=-threshold(imu,1)/10

        r_ay=threshold(r_ay,max_lin*0.1)
        r_ax=threshold(r_ax,max_ang*0.1)
        print("-- Speed: %.2f Direction: %.2f " % (r_ay, imu+r_ax))

        speed = r_ay
        direction = imu+r_ax

        control_sig = Int32MultiArray()
        sig = Float32MultiArray()
        twist=Twist()
        twist.linear.x=speed
        twist.linear.y=0
        twist.linear.z=0
        twist.angular.x=0
        twist.angular.y=0
        twist.angular.z=direction

        control_sig.data = [speed, direction, 0]
        sig.data = [float(speed), float(direction)]

        ##########Uncomment these 3 lines to enable the IMU control
        #pub_data.publish(twist)
        #pub_resq.publish(control_sig)
        #pub_base.publish(sig)



    print("Initializing node... ")
    rospy.init_node("teleoperation")
    pub_resq = rospy.Publisher('/resqbot/control_sig', Int32MultiArray, queue_size=1)
    pub_base = rospy.Publisher('base_control_sig', Float32MultiArray, queue_size=1)
    pub_data = rospy.Publisher('/robot/quickie/diff_drive/command', Twist, queue_size=1)

    print("Getting robot state... ")
    rs = baxter_interface.RobotEnable(CHECK_VERSION)
    init_state = rs.state().enabled
    left = baxter_interface.Limb('left')
    right = baxter_interface.Limb('right')
    left.set_command_timeout(0.1)
    right.set_command_timeout(0.1)
    grip_left = baxter_interface.Gripper('left', CHECK_VERSION)
    grip_right = baxter_interface.Gripper('right', CHECK_VERSION)
    grip_right.calibrate()
    grip_left.calibrate()

    lj = left.joint_names()
    rj = right.joint_names()


    print("Enabling robot... ")
    rs.enable()
    
    #right.move_to_neutral(timeout=5.0)
    kin_r = baxter_kinematics('right')
    kin_l = baxter_kinematics('left')
    print '\n*** Baxter Description ***\n'
    #kin.print_robot_description()
    print '\n*** Baxter KDL Chain ***\n'
    #kin.print_kdl_chain()

    rospy.Subscriber("exo_info", exo_info, map_exo)
    rospy.spin()

if __name__ == '__main__':
    listener()
