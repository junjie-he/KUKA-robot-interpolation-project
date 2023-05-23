import paho.mqtt.client as mqtt
import time
import rclpy
import sys
import json
import numpy as np
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from geometry_msgs.msg import Pose
from visualization_msgs.msg import Marker, MarkerArray


class TCPPath(Node):
    def __init__(self, name):
        super().__init__(name)
        self.topic = 'interpolated_path'
        self.pub = self.create_publisher(Path, self.topic, 10) # publish the interpolated path
        
        # publish the single velocity marker
        self.pub_1 = self.create_publisher(Marker, 'vel_marker', 10)
        # publish the velocity marker array
        self.pub_2 = self.create_publisher(MarkerArray, "velocity", 10)
        
        #self.host = "192.168.1.2"
        #self.port = 1883
        self.host = "mqtt.eclipseprojects.io"
        self.client_id = 'interp_path'
        self.client = mqtt.Client(self.client_id)
        
        self.client.connect(self.host)
        
        self.marker_sub = self.client.subscribe("waam_interpolation")
        self.client.on_message = self.callback
       
       
        self.count = 0
        
        # create a path to visualize the interpolated path
        self.path = Path()
        
        # create markerArrray to append the velocity markers
        self.vel_markers = MarkerArray()
        self.client.loop_forever()
     
        
    def callback(self, client, userdata, msg):
        data = str(msg.payload.decode("utf-8"))
        myDict = json.loads(data)
            
 
        # Pose
        
        self.pose = PoseStamped()
        
        self.pose.header.frame_id = "odom"
        
        self.pose.pose.position.x = myDict['toolPose']['X'] / 1000.0
        self.pose.pose.position.y = myDict['toolPose']['Y'] / 1000.0
        self.pose.pose.position.z = myDict['toolPose']['Z'] / 1000.0
        
        self.path.header.frame_id = "odom"
        self.path.header.stamp = self.get_clock().now().to_msg()
        self.pose.header.stamp = self.path.header.stamp
        self.path.poses.append(self.pose)
        
        # create one marker for velocity for every 17 messages
        if self.count % 17 == 0:
            self.marker = Marker()
            self.marker.id = self.count
            self.marker.ns = ""
            self.marker.header.frame_id = "odom"
            self.marker.type = Marker().SPHERE
            self.marker.pose.position.x = myDict['toolPose']['X'] / 1000.0
            self.marker.pose.position.y = myDict['toolPose']['Y'] / 1000.0
            self.marker.pose.position.z = myDict['toolPose']['Z'] / 1000.0
            self.marker.scale.x = myDict["toolVel"]*0.0005 + 0.000001 #add one small number to prevent it get 0
            self.marker.scale.y = myDict["toolVel"]*0.0005 + 0.000001
            self.marker.scale.z = myDict["toolVel"]*0.0005 + 0.000001
            self.marker.color.r = 0.7
            self.marker.color.g = 0.0
            self.marker.color.b = 0.7
            self.marker.color.a = 1.0
            self.pub_1.publish(self.marker)
        
            self.vel_markers.markers.append(self.marker)
 
        self.count += 1
        
        self.pub.publish(self.path)
        
        self.pub_2.publish(self.vel_markers)

        
        
def main(args = None):
    
    rclpy.init(args = args)
    my_path = TCPPath('interpolated_path')
    
    try:
        rclpy.spin(my_path)
    except KeyboardInterrupt:
        
        print("Quitting TCP Marker program...")
        my_path.destroy_node()
    sys.exit(0)
    
    
if __name__ == "__main__":
    main()
