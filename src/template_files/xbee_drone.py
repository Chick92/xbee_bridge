#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist

import struct
import serial
import time


class xbee_bridge:
    def __init__(self):
        port=rospy.get_param("xbee_bridge_gs/xbee_port")
        self.s=serial.Serial(port,57600)
        rospy.init_node('xbee_bridge_drone', anonymous=True)

        self.cv_pub = rospy.Publisher('/bebop/cmd_vel', Twist, queue_size=10)

        self.serial_buffer=""
        self.data=""
        self.write_buffer=list()
        self.read_buffer=list()

    def read_msg(self):
        current_read=''
        current_read=self.s.read(self.s.inWaiting())
        self.serial_buffer=self.serial_buffer + current_read

        while self.serial_buffer.find('\x04\x17\xfe') > 0:
            data,self.serial_buffer=self.serial_buffer.split('\x04\x17\xfe',1)
            self.read_buffer.append(data) 
      

        
    def publish_cmd_vel(self,data):
        cv_fmt='c2f'
        read_msg=struct.unpack(cv_fmt,data)
        T=Twist()
        T.linear.x=read_msg[1]
        T.linear.y=read_msg[2]
        T.linear.z=read_msg[3]
        T.angular.x=read_msg[4]
        T.angular.y=read_msg[5]
        T.angular.z=read_msg[6]
        self.cv_pub.publish(T)
        rospy.loginfo("Published Twist Msg")



    def read_msg_spinner(self):
        while not rospy.is_shutdown():
            try:    
                if self.s.inWaiting() > 0:
                    self.read_msg()
                if self.tf_timer < time.time():
                    self.get_tf()
                if self.gps_timer < time.time():
                    try: self.send_gps()
                    except AttributeError:
                        pass
                self.process_msgs()
                if self.mb_timer < time.time():
                    try: self.send_mb_status()
                    except AttributeError:
                        pass
                if self.gp_timer < time.time():
                    try: self.send_global_plan()
                    except AttributeError:
                        pass
                if self.gcm_timer < time.time():
                    try: self.send_costmap(self.global_CM,'a')
                    except AttributeError:
                        pass
                if self.lcm_timer < time.time():
                    try: self.send_costmap(self.local_CM,'b')
                    except AttributeError:
                        pass
            except IOError:
                rospy.logwarn("Serial IO Error")


    def process_msgs(self):
        if len(self.write_buffer) > 0:
           pass    

        if len(self.read_buffer) > 0:
            msg=self.read_buffer.pop(0)
            msg_type=msg[0]
            try:
                if msg_type == 'c':
                        self.publish_cmd_vel(msg)
            except:
                rospy.logwarn("Failed to Send Twist Msg")
            try:
                if msg_type == 'g':
                        self.publish_goal(msg)
            except AttributeError:
                rospy.logwarn("Failed to Send Goal")


    def spinner(self):
        self.read_msg_spinner()
        rospy.spin()

if __name__ == '__main__':
    aa=xbee_bridge()
    aa.spinner()