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
        self.topic = 'tool_veltxt'
        
        self.pub = self.create_publisher(MarkerArray, self.topic, 10)
        
       
       
        self.host = "mqtt.eclipseprojects.io"
        self.client_id = 'mqtt_markerss1'
        self.client = mqtt.Client(self.client_id)
        
        self.client.connect(self.host)
        
        self.marker_sub = self.client.subscribe("waam_interpolation")
        self.client.on_message = self.unpackJSON
        
        
       
        self.count = 0
       
        self.markerArray = MarkerArray()
       
        
        self.client.loop_forever()
        


    def unpackJSON(self, client, userdata, msg):
        data = str(msg.payload.decode("utf-8"))
        myDict = json.loads(data)
       
   
        self.text = Marker()

        
        self.text.header.frame_id = "odom"
        self.text.ns = ""
        self.text.header.stamp = self.get_clock().now().to_msg()
        
        # For text
        self.text.type   = Marker().TEXT_VIEW_FACING
        self.text.action = Marker().ADD
        self.text.id = self.count

        self.text.text = myDict['toolVel']

        
        # Scale
        self.text.scale.x = 0.2
        self.text.scale.y = 0.2
        self.text.scale.z = 0.2

     
        self.text.color.r = 1.0
        self.text.color.g = 1.0
        self.text.color.b = 1.0
        self.text.color.a = 1.0 
      

        # Pose
       
        self.text.pose.position.x = myDict['toolPose']['X'] / 1000.0
        self.text.pose.position.y = myDict['toolPose']['Y'] / 1000.0
        self.text.pose.position.z = myDict['toolPose']['Z'] / 1000.0 - 0.02
        
        self.text.pose.orientation.x = 0.0
        self.text.pose.orientation.y = 0.0
        self.text.pose.orientation.z = 0.0
        self.text.pose.orientation.w = 1.0
       
        self.markerArray.markers.append(self.text)

        self.count += 1
        
        #self.test.publish(self.marker)
        self.marker_pub.publish(self.markerArray)
        
       

        
        
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
