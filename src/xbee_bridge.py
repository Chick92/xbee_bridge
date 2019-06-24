#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty
import serial
import struct
import time


class xbee_bridge:
    def __init__(self):
        port=rospy.get_param("xbee_bridge/serial_port")
        self.s=serial.Serial(port,57600)

        rospy.init_node('xbee_bridge_gs', anonymous=True)
        rospy.Subscriber("/drone/cmd_vel", Twist, self.send_cmd_vel)
        rospy.Subscriber("/drone/land", Empty, self.send_land)
        rospy.Subscriber("/drone/reset", Empty, self.send_reset)
        rospy.Subscriber("/drone/takeoff", Empty, self.send_takeoff)

        self.cv_pub = rospy.Publisher('/bebop/cmd_vel', Twist, queue_size=10)
        self.land = rospy.Publisher("/bebop/land", Empty)
        self.reset = rospy.Publisher("/bebop/reset", Empty)
        self.takeoff = rospy.Publisher("/bebop/takeoff", Empty)
   
        self.empty_fmt='c2f' #Actually send something to check, rather than just the command character
        self.cv_fmt='c6f' # char and 2 floats format
        self.cv_rate=5
        self.cv_timer=time.time()
        self.serial_buffer=""
        self.data=""
        self.write_buffer=list()
        self.read_buffer=list()


    def read_msg(self):
        current_read=''
        current_read=self.s.read(self.s.inWaiting())
        self.serial_buffer=self.serial_buffer + current_read

        while self.serial_buffer.find('\r\n') > 0:
            data,self.serial_buffer=self.serial_buffer.split('\r\n',1)
            self.read_buffer.append(data) 
      
            

    def send_cmd_vel(self,data):
        if time.time() < self.cv_timer:
            return
        L=data.linear
        A=data.angular
        packet=struct.pack(self.cv_fmt,'c',L.x,L.y,L.z,A.x,A.y,A.z)
        self.s.write(bytes(packet)+'\r\n')
        self.cv_timer=time.time()+1/float(self.cv_rate)

    def send_takeoff(self,data):
        packet = struct.pack(self.empty_fmt, 't',0.55,0.66) # 2 floats of gash data
        self.s.write(bytes(packet)+'\r\n')

    def send_land(self,data):
        packet = struct.pack(self.empty_fmt, 'l',0.66,0.77)
        self.s.write(bytes(packet)+'\r\n')

    def send_reset(self,data):
        packet = struct.pack(self.empty_fmt, 'r',0.77,0.88)
        self.s.write(bytes(packet)+'\r\n')



    def publish_cmd_vel(self,data):
        cv_fmt='c6f'
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

    def publish_land(self,data):
        empty_fmt='c2f'
        read_msg=struct.unpack(empty_fmt,data)
        E=Empty()
        self.land.publish(E)
        rospy.loginfo("Published Land Msg")

    def publish_reset(self,data):
        empty_fmt='c2f'
        read_msg=struct.unpack(empty_fmt,data)
        E=Empty()
        self.reset.publish(E)
        rospy.loginfo("Published reset Msg")

    def publish_takeoff(self,data):
        empty_fmt='c2f'
        read_msg=struct.unpack(empty_fmt,data)
        E=Empty()
        self.takeoff.publish(E)
        rospy.loginfo("Published takeoff Msg")


    def read_msg_spinner(self):
        while not rospy.is_shutdown():
            try:    
                if self.s.inWaiting() > 0:
                    self.read_msg()
                self.process_msgs()

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
                if msg_type == 't':
                        self.publish_takeoff(msg)
                if msg_type == 'l':
                        self.publish_land(msg)
                if msg_type == 'r':
                        self.publish_reset(msg)

            except:
                rospy.logwarn("Failed to publish a message to ros master")




    def spinner(self):
        self.read_msg_spinner()
        rospy.spin()

    def spinner(self):
        self.read_msg_spinner()
        rospy.spin()

    def calc_checksum(self):
        pass
                
aa=xbee_bridge()
aa.spinner()