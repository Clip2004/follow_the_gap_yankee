#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Int32
from geometry_msgs.msg import Twist
import numpy as np
class GapFinderYankee(Node): # Redefine node class
    def __init__(self):
        super().__init__("gap_finder_yankee_node") # Redefine node name
        # obj (msg_type,topic_name, callback_handler, buffer) 
        self.scan_sub = self.create_subscription(LaserScan,'/scan',self.scan_callback,10)
        self.cmd_pos_ctrl_pub = self.create_publisher(Twist, '/cmd_pos_ctrl', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.r = 15 # Redefine range for gap detection
        self.gap_threshold = 1 # Redefine gap threshold
        self.n = 15 # Redefine number of consecutive hits found
        self.max_distance_index = 0 # Initialize max distance index
        self.x = 0.0
        self.angle = 0.0
    def get_Gaps(self, msg):
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
        
        consecutive_count = 0  # Contador de elementos consecutivos
        temp_indexes = []      # Índices temporales del gap actual
        
        for i in range(len(ranges_copy)):
            if ranges_copy[i] > self.gap_threshold:
                # Elemento cumple threshold
                consecutive_count += 1
                temp_indexes.append(i)
                
                # Si alcanzamos self.n consecutivos, agregar este gap
                if consecutive_count >= self.n:
                    # Agregar todos los índices del gap actual
                    if indexes and indexes[-1] != 0:  # Separar gaps con 0
                        indexes.append(0)
                    indexes.extend(temp_indexes)
            else:
                # Resetear contador si se rompe la secuencia
                consecutive_count = 0
                temp_indexes = []
        
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
        self.max_distance_index = max_gap[int(len(max_gap)//2)]
        self.get_logger().info(f"Max distance index: {self.max_distance_index}, Distance: {msg.ranges[self.max_distance_index]}")
        self.cmd_pos_ctrl(msg, self.max_distance_index)
    def cmd_pos_ctrl(self, msg, max_distance_index):
        self.x = msg.ranges[max_distance_index]
        if self.x == float('inf'):
            self.x = msg.range_max
        self.angle = msg.angle_min + (max_distance_index * msg.angle_increment)

    def timer_callback(self):
        twist_msg = Twist()
        
        # Velocidad lineal constante o basada en distancia
        if self.x > 2.0:
            twist_msg.linear.x = 0.8  # Velocidad alta si hay mucho espacio
        elif self.x > 1.0:
            twist_msg.linear.x = 0.5  # Velocidad media
        else:
            twist_msg.linear.x = 0.2  # Velocidad baja si está cerca
        
        # Ángulo relativo para steering
        twist_msg.angular.z = self.angle * 1.35  # Reducir factor de 1.35 a 1.0
        
        self.cmd_pos_ctrl_pub.publish(twist_msg)
def main(args=None):
    rclpy.init(args=args)
    node = GapFinderYankee() # object definition (creation)
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()