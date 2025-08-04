#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
class GapControllerYankee(Node): # Redefine node class
    def __init__(self):
        super().__init__("gap_controller_yankee") # Redefine node name
        # obj (msg_type,topic_name, callback_handler, buffer) 
        self.gap_sub = self.create_subscription(Int32,'/gap_index',self.gap_callback,10)
    def gap_callback(self, msg):
        self.get_logger().info(f"Received gap index: {msg.data}")
def main(args=None):
    rclpy.init(args=args)
    node = GapControllerYankee() # object definition (creation)

    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()