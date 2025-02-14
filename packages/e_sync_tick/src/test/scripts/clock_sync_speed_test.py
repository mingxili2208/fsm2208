import rclpy
from rclpy.node import Node
from rosgraph_msgs.msg import Clock

class ClockSyncSpeedTest(Node):
    def __init__(self):
        super().__init__('clock_sync_speed_test')
        self.subscription = self.create_subscription(
            Clock,
            '/clock',
            self.clock_callback,
            10
        )
        self.last_time = None
        self.tick_count = 0

    def clock_callback(self, msg):
        current_time = msg.clock.sec + msg.clock.nanosec * 1e-9  # 获取模拟时间
        if self.last_time is not None:
            self.tick_count += 1
            delta_time = current_time - self.last_time
            if delta_time >= 1.0:
                self.get_logger().info(f"Ticks in last second: {self.tick_count}")
                self.tick_count = 0
                self.last_time = current_time
        else:
            self.last_time = current_time


def main(args=None):
    rclpy.init(args=args)
    node = ClockSyncSpeedTest()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()