#!/usr/bin/env python

import os
import sys
import random
import logging
import pygame
import carla
import serial  # 串口通信模块
import numpy as np
from pygame.locals import K_ESCAPE

class BridgeHelpers(object):
    @staticmethod
    def get_agent_actor(world, role_name):
        actors = world.get_actors().filter("vehicle.*")
        for car in actors:
            if car.attributes["role_name"] == role_name:
                return car
        return None

class Ps2Controller:
    def __init__(self, client=None, host="127.0.0.1", port=2000, serial_port="/dev/ttyUSB1", baud_rate=9600):
        # CARLA 设置
        self.client = client or carla.Client(host, port)
        self.client.set_timeout(10.0)
        self.world = self.client.get_world()

        self.vehicle_role_name = os.environ["NPC_ROLE_NAME"]      
        self.vehicle  = BridgeHelpers.get_agent_actor(self.world, self.vehicle_role_name)

        # 串口设置
        self.serial_port = serial_port
        self.baud_rate = baud_rate
        self.serial_conn = None
        # 控制相关参数
        self.steer = 0.0
        self.steer_increment = 0.05
    

        # 初始化串口
        self.setup_serial()
    def setup_serial(self):
        """设置串口连接"""
        try:
            self.serial_conn = serial.Serial(self.serial_port, self.baud_rate, timeout=1)
            logging.info(f"Connected to serial port {self.serial_port} at {self.baud_rate} baud.")
        except Exception as e:
            logging.error(f"Failed to connect to serial port {self.serial_port}: {e}")
            self.serial_conn = None

    def parse_serial_data(self, data):
        """解析串口接收到的 PS2 数据包"""
        try:
            key_value_pairs = data.strip().split(",")
            parsed_data = {}
            for pair in key_value_pairs:
                key, value = pair.split(":")
                parsed_data[key] = int(value)  # 转换值为整数
            return parsed_data
        except Exception as e:
            logging.error(f"Error parsing serial data: {e}")
            return None
        
    def read_serial_input(self):
        """从串口读取数据并解析"""
        if self.serial_conn and self.serial_conn.in_waiting > 0:
            try:
                line = self.serial_conn.readline().decode('utf-8').strip()
                logging.info(f"Raw serial data: {line}")
                return self.parse_serial_data(line)
            except Exception as e:
                logging.error(f"Error reading from serial: {e}")
        return None
    def stop(self):
        if self.serial_conn:
            self.serial_conn.close()


    def run(self):
        """主循环，处理串口输入并控制车辆"""
        try:

            while True:
                for event in pygame.event.get():
                    if event.type == pygame.QUIT:
                        raise KeyboardInterrupt
                    elif event.type == pygame.KEYDOWN and event.key == K_ESCAPE:
                        raise KeyboardInterrupt

                # 初始化车辆控制
                control = carla.VehicleControl()
                control.throttle = 0.0
                control.brake = 0.0
                control.reverse = False

                # 从串口读取数据
                ps2_data = self.read_serial_input()
                if ps2_data:
                    # L2 控制油门
                    if ps2_data.get("L2", 1):
                        control.throttle += 0.1

                    # R2 控制刹车
                    if ps2_data.get("R2", 1):
                        control.brake = 1.0

                    # 左摇杆控制方向
                    lx = ps2_data.get("LX", 128)
                    if lx < 110:  # 左转
                        self.steer = max(self.steer - self.steer_increment, -1.0)*0.8
                    elif lx > 150:  # 右转
                        self.steer = min(self.steer + self.steer_increment, 1.0)*0.8
                    else:  # 停止转向
                        self.steer = 0.0

                    control.steer = self.steer

                    # 右摇杆控制前进和后退
                    ry = ps2_data.get("RY", 127)
                    if ry < 110:  # 前进
                        control.throttle = ((127 - ry) / 127.0)*0.15
                        control.reverse = False
                    elif ry > 150:  # 后退
                        control.throttle = ((ry - 127) / 127.0)
                        control.reverse = True

                self.vehicle.apply_control(control)
                self.visualization()
                pygame.display.flip()
                self.clock.tick(60)
        except KeyboardInterrupt:
            logging.info("Exiting...")
        finally:
            self.stop()


def main():
    logging.basicConfig(format="%(levelname)s: %(message)s", level=logging.INFO)

    terminal = Ps2Controller(
        host="127.0.0.1",
        port=2000,
        serial_port="/dev/ttyUSB1",
        baud_rate=9600
    )
    terminal.run()


if __name__ == "__main__":
    main()