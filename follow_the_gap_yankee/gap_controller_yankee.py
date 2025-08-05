#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import numpy as np
from tf_transformations import euler_from_quaternion
class GapControllerYankee(Node): # Redefine node class
    def __init__(self):
        super().__init__("gap_controller_yankee") # Redefine node name
        # obj (msg_type,topic_name, callback_handler, buffer) 
        self.cmd_pos_ctrl_sub = self.create_subscription(Twist,'/cmd_pos_ctrl',self.cmd_th_ctrl_callback,10)  
        self.cmd_vel_ctrl_pub = self.create_publisher(Twist,'/cmd_vel_gap_ctrl',10)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.th = 0.0
        self.x = 0.0
    def cmd_th_ctrl_callback(self, msg):
        self.th = msg.angular.z
        self.x = msg.linear.x
    def timer_callback(self):
        twist = Twist()
        twist.linear.x = self.x
        twist.angular.z = self.th
        self.cmd_vel_ctrl_pub.publish(twist)
def main(args=None):
    rclpy.init(args=args)
    node = GapControllerYankee() # object definition (creation)

    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()