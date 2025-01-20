import time
import math
from statistics import median
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

# Placeholder functions for hardware interactions (to be implemented)
def initialize_sensor():
    print("Sensor initialized.")

def initialize_actuator():
    print("Actuator initialized.")

def measure_distance():
    # Replace this with actual sensor distance measurement code
    return round(1 + 0.1 * math.sin(time.time()), 2)  # Simulated distance

def rotate_sensor(angle):
    print(f"Rotating sensor to {angle} degrees.")

def polar_to_cartesian(angle, distance):
    x = distance * math.cos(math.radians(angle))
    y = distance * math.sin(math.radians(angle))
    return x, y

# Data filtering function
def filter_data(data_points):
    filtered_points = []
    for i in range(1, len(data_points) - 1):
        median_distance = median([
            data_points[i - 1][1],
            data_points[i][1],
            data_points[i + 1][1]
        ])
        filtered_points.append((data_points[i][0], median_distance))
    return filtered_points

# ROS2 Node for publishing data
class LidarPublisher(Node):
    def __init__(self):
        super().__init__('lidar_publisher')
        self.publisher_ = self.create_publisher(LaserScan, '/scan', 10)

    def publish_scan(self, ranges, angles):
        scan = LaserScan()
        scan.header.frame_id = 'lidar_frame'
        scan.header.stamp = self.get_clock().now().to_msg()
        scan.angle_min = math.radians(min(angles))
        scan.angle_max = math.radians(max(angles))
        scan.angle_increment = math.radians(angles[1] - angles[0])
        scan.time_increment = 0.0
        scan.range_min = 0.1  # Minimum range of the sensor
        scan.range_max = 5.0  # Maximum range of the sensor
        scan.ranges = ranges
        self.publisher_.publish(scan)

# Main execution
def main():
    # Initialization
    initialize_sensor()
    initialize_actuator()
    rclpy.init()
    node = LidarPublisher()

    SCAN_ANGLE = 180  # Degrees
    SAMPLE_RATE = 10  # Data points per degree
    SCAN_SPEED = 0.1  # Seconds per step

    try:
        while rclpy.ok():
            raw_data = []

            # Data collection loop
            for angle in range(0, SCAN_ANGLE + 1, SAMPLE_RATE):
                rotate_sensor(angle)
                distance = measure_distance()
                raw_data.append((angle, distance))
                time.sleep(SCAN_SPEED)

            # Filter data
            filtered_data = filter_data(raw_data)

            # Prepare data for ROS2
            angles = [point[0] for point in filtered_data]
            ranges = [point[1] for point in filtered_data]

            # Publish to ROS2
            node.publish_scan(ranges, angles)

    except KeyboardInterrupt:
        print("Shutting down...")

    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()

