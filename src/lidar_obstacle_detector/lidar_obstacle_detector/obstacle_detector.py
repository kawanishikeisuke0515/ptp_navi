import rclpy
import math
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool

class ScanSubscriber(Node):
    def __init__(self):
        super().__init__('scan_subscriber')

        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            qos_profile_sensor_data,
        )
        self.obstacle_publisher = self.create_publisher(
            Bool,
            '/lidar/obstacle_detected',
            10,
        )
        self.get_logger().info('waitng /scan')

    def scan_callback(self, msg):
           lines = []
           d_stop = 0.3
           w_stop = 0.35
           detected = False

           for i, distance in enumerate(msg.ranges):
               angle_rad = msg.angle_min + i * msg.angle_increment
               angle_deg = math.degrees(angle_rad)
               rcos = distance * math.cos(angle_rad)
               rsin = abs(distance * math.sin(angle_rad))

               if (
                   rcos < d_stop
                   and rsin < w_stop
                   and distance > 0.0
                   and msg.range_min <= distance <= msg.range_max
               ):
                   detected = True
           
           self.obstacle_publisher.publish(Bool(data=detected))

def main(args=None):
    rclpy.init(args=args)
    node = ScanSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
