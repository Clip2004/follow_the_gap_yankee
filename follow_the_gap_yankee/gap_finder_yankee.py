#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Int32
import numpy as np
class GapFinderYankee(Node): # Redefine node class
    def __init__(self):
        super().__init__("gap_finder_yankee_node") # Redefine node name
        # obj (msg_type,topic_name, callback_handler, buffer) 
        self.scan_sub = self.create_subscription(LaserScan,'/scan',self.scan_callback,10)
        self.gap_pub = self.create_publisher(Int32, '/gap_index', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.r = 2 # Redefine range for gap detection
        self.gap_threshold = 5 # Redefine gap threshold
        self.n = 3 # Redefine number of consecutive hits found
        self.max_distance_index = 0 # Initialize max distance index
    def get_Gaps(self, msg):
        n = 0 # Initialize counter
        indexes = []
        # Convertir a numpy array para usar argmin
        ranges_array = np.array(msg.ranges)
        
        # Encontrar el ÍNDICE del valor mínimo
        laser_min_index = np.argmin(ranges_array)
        
        # Crear una copia para no modificar el mensaje original
        ranges_copy = ranges_array.copy()
        
        # Aplicar slicing con índices válidos
        start_idx = max(0, laser_min_index - self.r)
        end_idx = min(len(ranges_copy), laser_min_index + self.r + 1)
        ranges_copy[start_idx:end_idx] = 0.0
        
        for i in range(len(ranges_copy)):
            if ranges_copy[i] > self.gap_threshold:
                if n-i != 0:
                    indexes.append(i)
                    n = n+1
                else:
                    indexes.append(0)
        return indexes

    def scan_callback(self, msg):
        indexes = self.get_Gaps(msg)
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
        # self.get_logger().info(f"Max gap indices: {max_gap}")
        max_distance_index = 0
        for i in max_gap:
            if msg.ranges[i] > msg.ranges[self.max_distance_index]:
                self.max_distance_index = i

    def timer_callback(self):
        gap_msg = Int32()
        gap_msg.data = self.max_distance_index
        self.gap_pub.publish(gap_msg)
def main(args=None):
    rclpy.init(args=args)
    node = GapFinderYankee() # object definition (creation)
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()