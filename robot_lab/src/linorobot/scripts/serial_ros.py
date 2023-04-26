#!/usr/bin/env python
import rospy
from lino_msgs.msg import Velocities
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from test_serial2 import *

class ROS_communication:

    def __init__(self, b_rate= 9600 , port='/dev/ttyUSB0'):
        rospy.init_node('ROS_VDK_communication')
        self.pub  = rospy.Publisher("/raw_vel", Velocities , queue_size=1)
        self.sub = rospy.Subscriber("/cmd_vel", Twist , self.cmd_callback)
        self.tranf = PC_ROBOT_Communication(b_rate=b_rate , port = port)
        self.vx = 0.0
        self.vy = 0.0
        self.theta = 0.0

    
    def cmd_callback(self,msg):
        vx = int(msg.linear.x*1000) 
        vy = int(msg.linear.y*1000)
        theta = int(msg.angular.z*100)  
        d= [vx,vy,theta]
        self.tranf.PC_to_ROBOT(data = d)
        #print(d)       
        


    def get_vel(self):
        data  = self.tranf.ROBOT_to_PC()
        if data != None and len(data) == 3:
            vel = Velocities()
            if abs(self.vx - data[0]/1000.0) < 0.25:
                self.vx = data[0]/1000.0
            if abs(self.vy - data[1]/1000.0) < 0.25:
                self.vy = data[1]/1000.0
            if abs(self.theta - data[2]/100.0) < 0.5:
                self.theta = data[2]/100.0
            vel.linear_x = self.vx
            vel.linear_y = self.vy
            vel.angular_z = self.theta
            self.pub.publish(vel)
            print([self.vx,self.vy,self.theta])


    def run(self):
        rate  = rospy.Rate(30)

        while not rospy.is_shutdown():
            self.get_vel()
           
        
if __name__ == "__main__":
    rsc = ROS_communication()
    rsc.run()
    

        
        
