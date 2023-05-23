import paho.mqtt.client as mqtt

import rclpy
import json
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import JointState

class RobotModel(Node):

    def __init__(self, name):
    
        super().__init__(name)
        
        self.joint_state_pub = self.create_publisher(JointState, "joint_states", 10)
        self.client = mqtt.Client("WAAM_robot_interpolation")
        
        #self.host = "192.168.1.2"
        #self.port = 1883
        self.host = "mqtt.eclipseprojects.io"
        self.client.connect(self.host)
                         
        self.joint_state_sub = self.client.subscribe("waam_interpolation")
        
        #self.joint_state_sub = self.client.subscribe("interpolation")
        self.scale_factor = 0.06
        self.jointstate = JointState()
        self.jointstate.name = ["revolve_base", "lowerarm_revolve", "upperarm_lowerarm", "wrist_lowerarm", "wrist02_wrist", "welding_wrist02"]
        
        self.client.on_message = self.unpackJSON
        
        #Instance variables to define the starting position of the robot, from base to tcp
        self.theta_1 = 0.0
        self.theta_2 = 0.0
        self.theta_3 = 0.0
        self.theta_4 = 0.0
        self.theta_5 = 0.0
        self.theta_6 = 0.0

        self.client.loop_forever()
    
    #THIS IS JSON UNPACKING TO CONTROL ROBOT WITH MQTT    
    def unpackJSON(self, client, userdata, msg):
        data = str(msg.payload.decode("utf-8"))
        myDict = json.loads(data)    #decode json data, convert from a JSON string to a Python object
        

        try:
            self.theta_1 = -myDict['jointPose'][0] 
            self.theta_2 = myDict['jointPose'][1] + 90
            self.theta_3 = myDict['jointPose'][2] - 90
            self.theta_4 = myDict['jointPose'][3]
            self.theta_5 = myDict['jointPose'][4] - 90
            self.theta_6 = myDict['jointPose'][5]
            
            self.get_logger().info(f"theta_1 = {self.theta_1}, theta_2 = {self.theta_2}, theta_3 = {self.theta_3}, theta_4 = {self.theta_4}")
        except KeyError:
            print("No tcpPose data...")


        self.broadcastJointState()  # diff between broadcast and publisher???


    def broadcastJointState(self):

        self.time_now = self.get_clock().now().to_msg()

        self.jointstate.header.stamp = self.time_now
        self.jointstate.position = [float(np.radians(self.theta_1)), float(np.radians(self.theta_2)), float(np.radians(self.theta_3)), float(np.radians(self.theta_4)), float(np.radians(self.theta_5)), float(np.radians(self.theta_6))] 
        
        self.joint_state_pub.publish(self.jointstate)
        

def main(args = None):
    rclpy.init(args = args)
    my_sub = RobotModel('interpolation_group')
    
    rclpy.spin(my_sub)
    
    my_sub.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
