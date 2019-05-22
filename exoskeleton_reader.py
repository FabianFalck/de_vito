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
This python script will create ROS node "Exoskeleton" that is streaming all the exoskeleton data in a form of exoskeleton.msg.
The 3 involved steps are:
   1. Read the binary data from serial port (Arduino)
   2. Decode all the binary data to all exoskeleton data (angles, button states, joystick value, IMU, ..)
   3. Publish the data using the exoskeleton message
"""

import argparse
import rospy
import baxter_interface
from baxter_interface import CHECK_VERSION
import rospy
import numpy as np
import math 
import serial
import time
from exoskeleton.msg import exo_info

pi = round(math.pi,6)

class exoskeleton(object):
    """
    TODOKawin
    """
    def __init__(self):
        """
        TODOKawin
        """
        self._serial = serial.Serial('/dev/ttyACM0', 2000000,timeout=1)     #make sure that Arduino code is defined with the same communication speed "baud rate = 2000000"

        # initialize parameters TODO TODOKawin explain in comment why scalar multiplications
        self._raw_data=[0]*23
        self._filtered_angles=[0]*14
        self._debounce_button=[0]*8
        self._true_button=[0]*4
        self._analog=[0]*4
        self._imu=0
        self._filtered_imu=0
        self._end_point=[0]*6
        self._orientation=[0]*6

    def _get_data(self):
        """
        TODOKawin
        :return:
        """
        #Reading the data via serial port and record those data in the parameter named "raw_data"
        ser=self._serial
        data=self._raw_data
        while True:
            message=ser.read(3)
            if (message== 's\r\n'):
                message=ser.read(23)
                if len(message)==23:
                    # TODO TODOKawin Explain in comment the operations going on
                    for i in range(14):
                        hbyte=(ord(message[14+i/4])>>(2*i%8))&3
                        lbyte=ord(message[i])
                        data[i]=lbyte+(hbyte<<8)
                    data[14]=(ord(message[17])>>4)&1
                    data[15]=(ord(message[17])>>5)&1
                    data[16]=(ord(message[17])>>6)&1
                    data[17]=(ord(message[17])>>7)&1
                    data[18]=ord(message[18])-128
                    data[19]=ord(message[19])-128
                    data[20]=ord(message[20])-128
                    data[21]=ord(message[21])-128
                    data[22]=ord(message[22])-128 
                    break
        self._raw_data=data[:]


    def _get_angle(self):
        """
        TODOKawin
        :return:
        """
        #transform value of analog read (0-1023) to joint angle in (rad)
        equi_90deg=400.0
        set_zero=520.0
        data=self._raw_data
        info=[0]*23
        info[0]=-(data[6]-set_zero)/equi_90deg*pi/2
        info[1]=(data[5]-set_zero)/equi_90deg*pi/2
        info[2]=-(data[4]-set_zero)/equi_90deg*pi/2
        info[3]=(data[3]-set_zero)/equi_90deg*pi/2
        info[4]=-(data[2]-set_zero)/equi_90deg*pi/2
        info[5]=(data[1]-set_zero)/equi_90deg*pi/2+pi/2
        info[6]=-(data[0]-580)/equi_90deg*pi/2
        
        info[7]=-(data[7]-480)/equi_90deg*pi/2
        info[8]=-(data[8]-500)/equi_90deg*pi/2
        info[9]=-(data[9]-440)/equi_90deg*pi/2
        info[10]=-(data[10]-500)/equi_90deg*pi/2
        info[11]=-(data[11]-set_zero)/equi_90deg*pi/2
        info[12]=-(data[12]-520)/equi_90deg*pi/2-pi/2
        info[13]=-(data[13]-440)/equi_90deg*pi/2

        #filtered the ADC noise
        for i in range(14):
            info[i]=round(info[i],3)
            info[i]=round((19*self._filtered_angles[i]+info[i])/20,3)

        #debounce switch
        for i in range(0,4):
            sum=(data[14+i]+self._debounce_button[i]+self._debounce_button[i+4])
            if sum==3 or sum==0:
                info[14+i]=data[14+i]
            else :
                info[14+i]=self._true_button[i]          
                
        #threshold the analog stick
        for i in range(0,4):
            if data[18+i]>4 or data[18+i]<-4:
                info[18+i]=data[18+i]
            else:
                info[18+i]=0
        
        #getting IMU data        
        info[22]=data[22]

        self._debounce_button[0:4]=self._debounce_button[4:8]
        self._debounce_button[4:8]=data[14:18]
        self._filtered_angles[:]=info[0:14]
        self._true_button[:]=info[14:18]
        self._analog[:]=info[18:22]
        self._imu=info[22]


    def _rpy(self):
        """
        TODOKawin
        :return:
        """
        #tx=theta from exoskeleton joints
        tx1=self._filtered_angles[0]
        tx2=self._filtered_angles[1]
        tx3=self._filtered_angles[2]
        tx4=self._filtered_angles[3]
        tx5=self._filtered_angles[4]
        tx6=self._filtered_angles[5]
        tx7=self._filtered_angles[6]

        tx8=self._filtered_angles[7]
        tx9=self._filtered_angles[8]
        tx10=self._filtered_angles[9]
        tx11=self._filtered_angles[10]
        tx12=self._filtered_angles[11]
        tx13=self._filtered_angles[12]
        tx14=self._filtered_angles[13]

        #DH configuration right arm
        DHr=[[0 ,0    ,0     ,-tx1],
        [0      ,1.571 ,0   ,tx2],
        [0      ,1.571 ,0.37 ,tx3],
        [0      ,-1.571 ,0   ,tx4],
        [0      ,1.571 ,0.33 ,tx5],
        [0      ,-1.571 ,-0.07   ,tx6-pi/2],
        [0      ,-1.571 ,-0.07   ,tx7],
        [0.01    ,0     ,0   ,0]]

        #DH configuration left arm
        DHl=[[0 ,0    ,0     ,-tx8],
        [0      ,1.571 ,0   ,tx9],
        [0      ,1.571 ,0.37 ,tx10],
        [0      ,-1.571 ,0   ,tx11],
        [0      ,1.571 ,0.33 ,tx12],
        [0      ,-1.571 ,0.07   ,tx13-pi/2],
        [0      ,-1.571 ,-0.07   ,tx14],
        [0.01    ,0     ,0   ,0]]

        #create empty matrix
        tf=np.zeros((4,4))
        tfr=[tf]*8
        tfl=[tf]*8
        tfr_base=[tf]*8
        tfl_base=[tf]*8

        #calculate transformation matrix
        for i in range(8):
            tfr[i]=[[ math.cos(DHr[i][3])                  ,-math.sin(DHr[i][3])                   ,0                   ,DHr[i][0]       ],
                [ math.sin(DHr[i][3])*math.cos(DHr[i][1]) ,  math.cos(DHr[i][3])*math.cos(DHr[i][1]) ,-math.sin(DHr[i][1])   ,-math.sin(DHr[i][1])*DHr[i][2]],
                [ math.sin(DHr[i][3])*math.sin(DHr[i][1]) ,  math.cos(DHr[i][3])*math.sin(DHr[i][1]) ,math.cos(DHr[i][1])    ,math.cos(DHr[i][1])*DHr[i][2] ],
                [ 0                                 ,0                                   ,0                   ,1                        ]]
        for i in range(8):
            tfl[i]=[[ math.cos(DHl[i][3])                  ,-math.sin(DHl[i][3])                   ,0                   ,DHl[i][0]       ],
                [ math.sin(DHl[i][3])*math.cos(DHl[i][1]) ,  math.cos(DHl[i][3])*math.cos(DHl[i][1]) ,-math.sin(DHl[i][1])   ,-math.sin(DHl[i][1])*DHl[i][2]],
                [ math.sin(DHl[i][3])*math.sin(DHl[i][1]) ,  math.cos(DHl[i][3])*math.sin(DHl[i][1]) ,math.cos(DHl[i][1])    ,math.cos(DHl[i][1])*DHl[i][2] ],
                [ 0                                 ,0                                   ,0                   ,1                        ]]

        #get the homogeneous transform
        tfr_base[0]=tfr[0]
        tfr_base[0][1][3]=-0.225
        for i in range(1,8):
            tfr_base[i]=np.matmul(tfr_base[i-1],tfr[i])
        tfl_base[0]=tfl[0]
        tfl_base[0][1][3]=0.225
        for i in range(1,8):
            tfl_base[i]=np.matmul(tfl_base[i-1],tfl[i])

        self._end_point[0]=round((1*self._end_point[0]+round(tfr_base[6][0][3],4))/2,4)                                 #X Right end effector
        self._end_point[1]=round((1*self._end_point[1]+round(tfr_base[6][1][3],4))/2,4)                                 #Y Right end effector
        self._end_point[2]=round((1*self._end_point[2]+round(tfr_base[6][2][3],4))/2,4)                                 #Z Right end effector
        self._end_point[3]=round((1*self._end_point[3]+round(tfl_base[6][0][3],4))/2,4)                                 #X Left end effector
        self._end_point[4]=round((1*self._end_point[4]+round(tfl_base[6][1][3],4))/2,4)                                 #Y LEft end effector                                 
        self._end_point[5]=round((1*self._end_point[5]+round(tfl_base[6][2][3],4))/2,4)                                 #Z Left end effector

        self._orientation[0]=round(np.arctan2(tfr_base[6][1][0],tfr_base[6][0][0]),2)                                   #yaw Right end effector
        self._orientation[1]=round(np.arctan2(-tfr_base[6][2][0],np.sqrt(tfr_base[6][2][1]**2+tfr_base[6][2][2]**2)),2) #pitch Right end effector
        self._orientation[2]=round(np.arctan2(tfr_base[6][2][1],tfr_base[6][2][2]),2)                                   #roll Right end effector

        self._orientation[3]=round(np.arctan2(tfl_base[6][1][0],tfl_base[6][0][0]),2)                                   #yaw Left end effector           
        self._orientation[4]=round(np.arctan2(-tfl_base[6][2][0],np.sqrt(tfl_base[6][2][1]**2+tfl_base[6][2][2]**2)),2) #pitch Left end effector
        self._orientation[5]=round(np.arctan2(tfl_base[6][2][1],tfl_base[6][2][2]),2)                                   #roll Left end effector


    def publish_data(self):
        """
        TODOKawin
        :return:
        """

        msg=exo_info()
        pub = rospy.Publisher('exo_info', exo_info, queue_size=1)
        rate = rospy.Rate(1000)

        while not rospy.is_shutdown():
            if self._serial.in_waiting>52:
                self._serial.flushInput()
            
            self._get_data()
            self._get_angle()
            self._rpy()

            msg.exo_angles=self._filtered_angles
            msg.buttons=self._true_button
            msg.analog=self._analog
            msg.imu=self._imu
            msg.end_point=self._end_point
            msg.rpy=self._orientation
            rospy.loginfo(msg)
            pub.publish(msg)
            rate.sleep()


def main():
    """
    TODOKawin
    :return:
    """

    print("Initializing node... ")
    rospy.init_node("Exoskeleton")
    exo=exoskeleton()
    exo.publish_data()


if __name__ == "__main__":
    main()
