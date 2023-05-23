import paho.mqtt.client as mqtt
import time
import rclpy
import sys
import json
import numpy as np
from rclpy.node import Node

from geometry_msgs.msg import Pose, PoseStamped
from nav_msgs.msg import Path


class DeviationCalculator(Node):
    def __init__(self, name):
        super().__init__(name)
        self.client = mqtt.Client("WAAM_robot_interpolation")
        self.host = "192.168.1.2"
        self.port = 1883
        self.client.connect(self.host, self.port)
        
        self.deviation_subscriber = self.client.subscribe("test/waypoint")
        self.client.on_message = self.deviation_callback
        self.sum = 0
        self.count = 0
        
    def deviation_callback(self, client, userdata, msg):
        
        # THIS IS FOR CALCULATING THE DEVIATION BETWEEN INTERPOLATED PATH AND PLANNED PATH
        try:
            self.plan_x = myDict['planned']["tcpPose"]["X"] 
            self.plan_y = myDict['planned']["tcpPose"]["Y"] 
            self.plan_z = myDict['planned']["tcpPose"]["Z"] 
            self.interp_x = myDict["toolPose"]["X"] 
            self.interp_y = myDict["toolPose"]["Y"] 
            self.interp_z = myDict["toolPose"]["Z"] 
            
            # 0.36 is the y coordinate of starting point of original path, -0.36 is end point
            if self.plan_y >= -0.36 and self.plan_y <= 0.36:
        
                self.deviation =  math.sqrt((self.plan_x - self.interp_x)**2 + (self.plan_y - self.interp_y)**2 + (self.plan_z - self.interp_z)**2)
                self.sum += self.deviation
                self.count += 1
            else:
                pass
                
            
                
            self.interp_dev = self.sum/self.count
          
            self.get_logger().info(f"The interpolated deviation is {self.interp_dev}")
        except:
            print("No planned and interpolated data...")
        
        
def main(args = None):
    
    rclpy.init(args = args)
    my_dev = DeviationCalculator('interpolated_deviation')
    
    try:
        rclpy.spin(my_dev)
    except KeyboardInterrupt:
        
        print("Quitting TCP Marker program...")
        my_dev.destroy_node()
    sys.exit(0)
    
    
if __name__ == "__main__":
    main()
