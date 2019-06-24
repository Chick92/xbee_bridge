#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
import serial
import struct
import time


class xbee_bridge:
    def __init__(self):
        port=rospy.get_param("xbee_bridge_gs/xbee_port")
        self.s=serial.Serial(port,57600)
        rospy.init_node('xbee_gs', anonymous=True)
        rospy.Subscriber("/bebop/cmd_vel", Twist, self.send_cmd_vel)
   
        self.serial_buffer=""
        self.data=""
        self.cv_fmt='c2f' # char and 2 floats format
        self.cv_rate=5
        self.cv_timer=time.time()
        self.read_buffer=list()

    def read_msg(self):
        current_read=''
        current_read=self.s.read(self.s.inWaiting())
        self.serial_buffer=self.serial_buffer + current_read
        while self.serial_buffer.find('\x04\x17\xfe') > 0:    # deliminator
            data,self.serial_buffer=self.serial_buffer.split('\x04\x17\xfe',1)
            self.read_buffer.append(data)
            

    def send_cmd_vel(self,data):
        if time.time() < self.cv_timer:
            return
        L=data.linear
        A=data.angular
        packet=struct.pack(self.cv_fmt,'c',L.x,L.y,L.z,A.x,A.y,A.z)
        self.s.write(bytes(packet)+'\x04\x17\xfe')
        self.cv_timer=time.time()+1/float(self.cv_rate)


    def process_msgs(self):
        if len(self.read_buffer) > 0:
            msg=self.read_buffer.pop(0)
            msg_type=msg[0]
            try:
                if msg_type == 't':
                        self.publish_tf(msg)
                if msg_type == 'f':
                        self.publish_gps(msg)
                if msg_type == 'b':
                        #self.data=data.strip()
                        self.publish_cm('b',msg)
                if msg_type == 'a':
                        #self.data=data.strip()
                        self.publish_cm('a',msg)
                if msg_type == 's':
                        self.publish_mbs(msg)
                if msg_type == 'd':
                        self.publish_gp(msg)
            except AttributeError:
                rospy.logwarn("AttributeError")
            except struct.error:
                rospy.logwarn("Corrupt Packet")


    def read_msg_spinner(self):
            while not rospy.is_shutdown():
                if self.s.inWaiting() > 0:
                    self.read_msg()
                self.process_msgs()

    def spinner(self):
        self.read_msg_spinner()
        rospy.spin()

    def calc_checksum(self):
        pass
                
aa=RFD900_GCS()
aa.spinner()