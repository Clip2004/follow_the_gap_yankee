#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
class GapFinderYankee(Node): # Redefine node class
    def __init__(self):
        super().__init__("gap_finder_yankee_node") # Redefine node name
        # obj (msg_type,topic_name, callback_handler, buffer) 
        self.scan_sub = self.create_subscription(LaserScan,'/scan',self.scan_callback,10)
        self.r = 2 # Redefine range for gap detection
        self.gap_threshold = 5 # Redefine gap threshold
        self.n = 3 # Redefine number of consecutive hits found
    def get_Gaps(self, msg):
        n = 0 # Initialize counter
        indexes = []
        laser_min = min(msg.ranges)
        msg.ranges[laser_min-self.r:laser_min+self.r] = 0.0
        for i in range(len(msg.ranges)):
            if msg.ranges[i] > self.gap_threshold:
                if n-i != 0:
                    indexes.append(i)
                    n = n+1
                else:
                    indexes.append(0)
        return indexes
                
    def scan_callback(self, msg):
        indexes = self.get_Gap(msg)
        if not indexes:
            self.get_logger().info("No gaps found")
            return
        max_gap = []
        current_gap = []
        for idx in indexes:
            if idx != 0:
                current_gap.append(idx)
            else:
                # End of current gap sequence
                if len(current_gap) > len(max_gap):
                    max_gap = current_gap.copy()  # Create a copy to avoid reference issues
                current_gap = []
        # Handle the case where the sequence doesn't end with zero
        if len(current_gap) > len(max_gap):
            max_gap = current_gap
        
        self.get_logger().info(f"Max gap indices: {max_gap}")

def main(args=None):
    rclpy.init(args=args)
    node = GapFinderYankee() # object definition (creation)
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()