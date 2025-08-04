#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
# from std_msgs.msg import Int32
from geometry_msgs.msg import Twist
class GapControllerYankee(Node): # Redefine node class
    def __init__(self):
        super().__init__("gap_controller_yankee") # Redefine node name
        # obj (msg_type,topic_name, callback_handler, buffer) 
        self.cmd_pos_ctrl_sub = self.create_subscription(Twist,'/cmd_pos_ctrl',self.cmd_pos_ctrl_callback,10)
    def cmd_pos_ctrl_callback(self, msg):
        self.get_logger().info(f"Received cmd_pos_ctrl: linear.x={msg.linear.x}, angular.z={msg.angular.z}")
def main(args=None):
    rclpy.init(args=args)
    node = GapControllerYankee() # object definition (creation)

    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()