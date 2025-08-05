#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import math
class AEBSystem(Node): # Redefine node class
    def __init__(self):
        super().__init__("AEBSystem_node") # Redefine node name
        self.stop = 0
        self.direction = 0
        # create a topic subscriber
        # obj (msg_type,topic_name, callback_handler, buffer) 
        self.scan_subs = self.create_subscription(LaserScan,'/scan',self.scan_callback,10)
        # publisher obj (msg_type, topic_name, queue==buffer)
        self.cmd_pub = self.create_publisher(Twist,'cmd_vel_stop',10)
        # create a timer function to send msg
        timer_period = 0.1 # in [s]
        self.timer = self.create_timer(timer_period,self.stop_vehicle)
    def scan_callback(self,msg):
        # Example: Accessing scan data
        angle_range = math.radians(5)  # ±5 grados
        min_angle = -angle_range
        max_angle = angle_range
        i_min = int((min_angle - msg.angle_min) / msg.angle_increment)
        i_max = int((max_angle - msg.angle_min) / msg.angle_increment)
        front_ranges = msg.ranges[i_min:i_max+1]
        valid_ranges = [r for r in front_ranges if r != float('inf')]
        index_a = 0
        index_b = 180
        if valid_ranges:
            min_distance = min(valid_ranges)
            if min_distance < 0.4:
                self.stop = 1
                if msg.ranges[index_a] < msg.ranges[index_b]:
                    self.direction = 1
                else:
                    self.direction = 0
                # self.get_logger().info(f'Obstacle detected at {min_distance:.2f} m in front.')
            else:
                self.stop = 0
                # self.get_logger().info(f'No obstacle detected in front. Minimum distance: {min_distance:.2f} m')    
    def stop_vehicle(self):
        angular_z = -1.0  # Example angular velocity
        cmd_vel_stop = Twist()
        if self.stop == 1:
            self.get_logger().info('Stopping vehicle due to obstacle detection.')
            # Here you would implement the logic to stop the vehicle, e.g., publishing a stop command
            cmd_vel_stop.linear.x = -0.75
            if self.direction == 1:
                cmd_vel_stop.angular.z = -angular_z  # Adjust as needed
            else:
                cmd_vel_stop.angular.z = angular_z  # Adjust as needed
            self.cmd_pub.publish(cmd_vel_stop)

def main(args=None):
    rclpy.init(args=args)
    node = AEBSystem() # object definition (creation)
    rclpy.spin(node)
    rclpy.shutdown()
if __name__ == "__main__":
    main()