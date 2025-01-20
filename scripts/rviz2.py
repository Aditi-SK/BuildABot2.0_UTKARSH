import rclpy 
from rclpy.node import Node 
from sensor_msgs.msg import LaserScan 
class LidarVisualizer(Node): 
def __init__(self): 
super().__init__('lidar_visualizer') 
self.publisher_ = self.create_publisher(LaserScan, '/scan', 10) 
def publish_scan(self, ranges, angles): 
scan = LaserScan() 
scan.header.frame_id = 'lidar_frame' 
scan.ranges = ranges 
scan.angle_min = angles[0] 
scan.angle_max = angles[-1] 
self.publisher_.publish(scan) 
rclpy.init() 
node = LidarVisualizer() 
node.publish_scan([1.0, 2.0, 3.0], [-1.57, 0, 1.57]) 
rclpy.shutdown()
