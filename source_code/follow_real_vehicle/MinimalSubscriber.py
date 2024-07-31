#!/usr/bin/env python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
import trans_utils as trans 

class MinimalSubscriber(Node):
    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            PoseStamped,
            "/real_world/follow_adtruck/transformed",
            self.listener_callback,
            10)
    def run(self):
        """
        Start ROS node loop.
        """
        self.get_logger().info(f"starting spin")
        rclpy.spin(self)
    def listener_callback(self, msg):
        self.get_logger().info('I heard the origin info as : "%s"' % msg.pose)
        carla_pose_transform = trans.ros_pose_to_carla_transform(msg.pose)
        print(carla_pose_transform)
        #self.get_logger().info(carla_pose_transform)

def main(args=None):
    rclpy.init(args=args)
    minimal_subscriber = MinimalSubscriber()
    minimal_subscriber.run()
    minimal_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()