import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from lidar_obstacle_interfaces.msg import ObstacleGroup, ObstacleGroupArray

class ObstacleGroupNode(Node):
    def __init__(self):
        super().__init__('obstacle_group')

        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            qos_profile_sensor_data,
        )

        self.publisher = self.create_publisher(
            ObstacleGroupArray,
            '/lidar/obstacle_groups',
            10,
        )
        self.get_logger().info('Obstacle Group Node started.')

    def is_valid_distance(self, distance, min_distance, max_distance):
        return min_distance <= distance <= max_distance


    def scan_callback(self, msg):
        # Parameters for obstacle grouping
        max_groups = 3
        max_detection_distance_m = 1.0
        min_detection_distance_m = 0.1
        distance_threshold_m = 0.3
        min_group_points = 3

        # Initialize variables for obstacle grouping
        current_distance = None
        group_point_count = 0
        obstacle_groups = []
        closest_obstacle_distance = None
        closest_obstacle_angle = None

        for i, distance in enumerate(msg.ranges):
            angle_rad = msg.angle_min + i * msg.angle_increment
            if self.is_valid_distance(distance, min_detection_distance_m, max_detection_distance_m):
                if group_point_count == 0:
                    current_distance = distance
                    group_point_count = 1
                    closest_obstacle_distance = distance
                    closest_obstacle_angle = angle_rad
                else:
                    if abs(distance - current_distance) < distance_threshold_m:
                        group_point_count += 1
                        current_distance = distance
                        if distance < closest_obstacle_distance:
                            closest_obstacle_distance = distance
                            closest_obstacle_angle = angle_rad

                    else:
                        if group_point_count >= min_group_points:
                            group = ObstacleGroup()
                            group.distance_m = closest_obstacle_distance
                            group.angle_rad= closest_obstacle_angle
                            obstacle_groups.append(group)

                        current_distance = distance    
                        closest_obstacle_distance = distance
                        closest_obstacle_angle = angle_rad
                        group_point_count = 1
            else:
                if group_point_count >= min_group_points:
                     group = ObstacleGroup()
                     group.distance_m = closest_obstacle_distance
                     group.angle_rad = closest_obstacle_angle
                     obstacle_groups.append(group)

                current_distance = None
                closest_obstacle_distance = None
                closest_obstacle_angle = None
                group_point_count = 0

        if group_point_count >= min_group_points:
            group = ObstacleGroup()
            group.distance_m = closest_obstacle_distance
            group.angle_rad = closest_obstacle_angle
            obstacle_groups.append(group)

        obstacle_groups.sort(key=lambda group: group.distance_m)
        selected_groups = obstacle_groups[:max_groups]

        output_msg = ObstacleGroupArray()
        output_msg.header = msg.header
        output_msg.groups = selected_groups
        self.publisher.publish(output_msg)

def main(args=None):
    rclpy.init(args=args)
    node = ObstacleGroupNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()