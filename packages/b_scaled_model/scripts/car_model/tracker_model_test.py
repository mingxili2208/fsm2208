import os
import random
import carla
import rclpy
import signal
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from transforms3d.euler import quat2euler
from srunner.scenariomanager.carla_data_provider import CarlaDataProvider
import math
import time


class VehicleSpawner(Node):
    def __init__(self):
        super().__init__('vehicle_spawner')

        # CARLA 配置
        self.local_host = "localhost"
        self.port = 2000
        self.agent_role_name = "test_spawner"
        self.agent_model_type = "vehicle.audi.etron"

        # 订阅PoseWithCovarianceStamped话题
        self.subscription = self.create_subscription(
            PoseWithCovarianceStamped,
            '/real_world/follow_adtruck/transformed_with_covariance',
            self.pose_callback,
            10
        )

        self.world = None
        self.pose_initialized = False  # 标记是否已收到位姿消息
        self.spawn_point = None

    def pose_callback(self, msg):
        """
        回调函数，接收来自 /real_world/follow_adtruck/transformed_with_covariance 话题的位姿信息
        """
        self.get_logger().info(f"x={msg.pose.pose.position.x}, y={msg.pose.pose.position.y}, z={msg.pose.pose.position.z}")

        # 使用接收到的位姿消息来设置生成点
        self.spawn_point = carla.Transform()
        self.spawn_point.location.x = msg.pose.pose.position.x
        self.spawn_point.location.y = -msg.pose.pose.position.y  # ROS坐标系与CARLA坐标系的y轴相反
        self.spawn_point.location.z = msg.pose.pose.position.z + 1.0  # 微调Z高度，避免车辆嵌入地面

        # 四元数转欧拉角（CARLA 使用欧拉角来表示旋转）
        orientation_q = msg.pose.pose.orientation
        roll, pitch, yaw = quat2euler([orientation_q.w, orientation_q.x, orientation_q.y, orientation_q.z])
        self.spawn_point.rotation.roll = math.degrees(roll)
        self.spawn_point.rotation.pitch = math.degrees(pitch)
        self.spawn_point.rotation.yaw = -math.degrees(yaw)  # ROS到CARLA的yaw角调整

        self.pose_initialized = True  # 标记为已初始化

    def load_vehicle(self):
        """
        加载车辆，确保已经接收到位姿信息
        """
        self.client = carla.Client(self.local_host, self.port)
        self.client.set_timeout(20)
        self.world = self.client.get_world()
        CarlaDataProvider.set_world(self.world)
        CarlaDataProvider.set_client(self.client)

        # 等待直到接收到位姿消息
        while not self.pose_initialized:
            self.get_logger().info("Waiting for initial pose from /real_world/follow_adtruck/transformed_with_covariance...")
            rclpy.spin_once(self, timeout_sec=1.0)

        # 确保车辆未被重复生成
        if VehicleSpawner.get_agent_actor(CarlaDataProvider.get_world(), self.agent_role_name) is None:
            red = str(random.randint(0, 255))
            blue = str(random.randint(0, 255))
            green = str(random.randint(0, 255))
            ego_vehicle_color = ",".join([red, blue, green])

            # 生成车辆
            CarlaDataProvider.request_new_actor(
                self.agent_model_type,
                self.spawn_point,
                self.agent_role_name,
                color=ego_vehicle_color
            )
            self.get_logger().info(f"Vehicle {self.agent_role_name} spawned successfully.")
            
        else:
            self.get_logger().warning(f"Vehicle {self.agent_role_name} already exists!")
            raise RuntimeError(f"The vehicle {self.agent_role_name} has already been spawned!")

        # 返回 client
        return self.client

    @staticmethod
    def get_agent_actor(world, role_name):
        """
        获取指定 role_name 的车辆 Actor
        """
        actors = world.get_actors().filter("vehicle.*")
        for car in actors:
            if car.attributes["role_name"] == role_name:
                return car
        return None

    def cleanup(self):
        """
        清理车辆和资源
        """
        self.get_logger().info("Cleaning up...")
        vehicle = VehicleSpawner.get_agent_actor(CarlaDataProvider.get_world(), self.agent_role_name)
        if vehicle is not None:
            try:
                self.get_logger().info(f"Destroying vehicle: {self.agent_role_name}")
                vehicle.destroy()

                # 等待销毁完成
                time.sleep(0.1)
                self.get_logger().info(f"Vehicle {self.agent_role_name} destroyed successfully.")
            except Exception as e:
                self.get_logger().error(f"Error while destroying vehicle: {e}")
        else:
            self.get_logger().warning(f"Vehicle {self.agent_role_name} does not exist or is already destroyed.")

        # 清理CARLA数据
        CarlaDataProvider.cleanup()


def signal_handler(spawner, signum, frame):
    """
    信号处理函数，在程序中断时清理资源
    """
    spawner.cleanup()
    rclpy.shutdown()  # 确保 ROS 2 节点正确关闭
    print("Exiting gracefully...")
    exit(0)



if __name__ == "__main__":
    rclpy.init(args=None)
    spawner = VehicleSpawner()

    # 注册 SIGINT 信号处理函数，用于处理 CTRL+C 中断
    signal.signal(signal.SIGINT, lambda signum, frame: signal_handler(spawner, signum, frame))

    try:
        spawner.load_vehicle()
        while True:
            pass
    finally:
        spawner.cleanup()