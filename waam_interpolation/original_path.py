import paho.mqtt.client as mqtt
import time
import rclpy
import sys
import json
import numpy as np
from rclpy.node import Node
from geometry_msgs.msg import Point

from visualization_msgs.msg import Marker, MarkerArray   ##Marker is the msg type we are going to pub


class TCPMarker(Node):
    def __init__(self, name):
        super().__init__(name)
        self.topic = 'original_path'
        self.marker_pub = self.create_publisher(MarkerArray, "markers", 10)
        self.line_pub = self.create_publisher(MarkerArray, "original_path", 10)
        
        # Create a timer that will gate the node actions twice a second
        timer_period = 1  # seconds
        self.timer = self.create_timer(timer_period, self.callback_mqtt)
    
        self.count = 0
        self.markerArray = MarkerArray()
        self.lines = MarkerArray()

    def callback_mqtt(self): #, client, userdata, msg):
   
            

    
        self.marker = Marker()
        self.line = Marker()

        
        self.marker.header.frame_id = "odom"
        self.marker.ns = ""      #name space is used for grouping markers, don't put anything, only "", separate node and topics from each other 
        self.marker.header.stamp = self.get_clock().now().to_msg()
        
        self.line.header.frame_id = "odom"
        self.line.ns = ""      #name space is used for grouping markers, don't put anything, only "", separate node and topics from each other 
        self.line.header.stamp = self.get_clock().now().to_msg()
    
        #self.tcpMarker.id = index           #index at which we start, since this is consecutive publish
        self.marker.type   = Marker().TEXT_VIEW_FACING
        self.line.type = Marker().LINE_STRIP
        self.marker.action = Marker().ADD  ## ADD REMOVE MODIFY
        self.line.action = Marker().ADD
        self.marker.id = self.count
        self.line.id = self.count+10

        # Note: Must set mesh_resource to a valid URL for a model to appear
        #we can only change the z_val, which means the position of markers can only move vertically
        #self.marker.pose.position = my_point
        
        self.marker.text = str(self.marker.id)
        

        # Scale
        self.marker.scale.x = 1 - self.count*0.2
        self.marker.scale.y = 1 - self.count*0.2
        self.marker.scale.z = 1 - self.count*0.2
        
        
        self.line.scale.x = 0.02


        # Color
        self.marker.color.r = 0.5 +  self.count*0.1
        self.marker.color.g = 0.0
        self.marker.color.b = 0.0
        self.marker.color.a = 1.0 
        
        self.line.color.r = 0.0
        self.line.color.g = 1.0
        self.line.color.b = 0.0
        self.line.color.a = 1.0 

        
        
        # Pose
        
        self.marker.pose.position.x = 0.0 +  self.count   
        self.marker.pose.position.y = 0.0 +  self.count  
        self.marker.pose.position.z = 0.0 +  self.count 
        self.marker.pose.orientation.x = 0.0
        self.marker.pose.orientation.y = 0.0
        self.marker.pose.orientation.z = 0.0
        self.marker.pose.orientation.w = 1.0
        
        
        
        self.point1 = Point()
        self.point2 = Point()
        self.point3 = Point()
        
        
        self.point1.x = 1.2
        self.point1.y = 0.36
        self.point1.z = 1.0
        
        self.point2.x = 1.6
        self.point2.y = 0.0
        self.point2.z = 1.0
        
        self.point3.x = 1.2
        self.point3.y = -0.36
        self.point3.z = 1.0
       
        
        self.line.points.append(self.point1)
        self.line.points.append(self.point2)
        self.line.points.append(self.point3)
        
      
      
            
        if self.count < 5:
            self.markerArray.markers.append(self.marker)
            
            
            self.lines.markers.append(self.line)
        
 
        self.count += 1
        
        self.marker_pub.publish(self.markerArray)
        self.line_pub.publish(self.lines)

        
        
def main(args = None):
    rclpy.init(args = args)
    my_marker = TCPMarker('interpolation')
    
    try:
        rclpy.spin(my_marker)
    except KeyboardInterrupt:
        
        print("Quitting TCP Marker program...")
        my_marker.destroy_node()
    sys.exit(0)
    
    
if __name__ == "__main__":
    main()
