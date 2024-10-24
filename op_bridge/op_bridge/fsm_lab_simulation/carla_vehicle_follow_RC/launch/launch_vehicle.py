#!/usr/bin/env python

import os
import cv2
import sys
import time
import signal
import pygame
import random
import importlib
import traceback

import carla
import rclpy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from leaderboard.envs.sensor_interface import SensorReceivedNoData
from leaderboard.autoagents.agent_wrapper import AgentWrapper, AgentError
from srunner.scenariomanager.timer import GameTime
from srunner.scenariomanager.watchdog import Watchdog
from srunner.scenariomanager.carla_data_provider import CarlaDataProvider


class BridgeHelpers(object):
    @staticmethod
    def get_agent_actor(world, role_name):
        actors = world.get_actors().filter("vehicle.*")
        for car in actors:
            if car.attributes["role_name"] == role_name:
                return car
        return None

class EgoVehicleLauncher(object):
    def __init__(self):
        self.local_host = os.environ["SIMULATOR_LOCAL_HOST"]
        self.port = int(os.environ["SIMULATOR_PORT"])
        self.agent_role_name = os.environ["AGENT_ROLE_NAME"].replace("-", "_")
        self.agent_model_type = os.environ["AGENT_MODEL_TYPE"]
        self.map_name = os.environ["FREE_MAP_NAME"]
        self.spawn_point = os.environ["FREE_AGENT_POSE"]
        self.ego_vehicle = None
        self.world = None
    
    def load_vehicle(self):
        self.client = carla.Client(self.local_host, self.port)
        self.client.set_timeout(20)
        self.world = self.client.get_world()
        CarlaDataProvider.set_world(self.world)
        CarlaDataProvider.set_client(self.client)  

        point_items = self.spawn_point.split(",")
        randomize = False
        if len(point_items) == 6:
            spawn_point = carla.Transform()
            spawn_point.location.x = float(point_items[0])
            spawn_point.location.y = float(point_items[1])
            spawn_point.location.z = float(point_items[2]) + 2
            spawn_point.rotation.roll = float(point_items[3])
            spawn_point.rotation.pitch = float(point_items[4])
            spawn_point.rotation.yaw = float(point_items[5])
        elif len(point_items) == 1 and self.spawn_point != "":
            spawn_points = self.world.get_map().get_spawn_points()
            spawn_point = spawn_points[eval(self.spawn_point)]
        else:
            spawn_point = carla.Transform()
            randomize = True
        
        # if self.agent_model_type == "":
        #     bps = self.world.get_blueprint_library().filter("vehicle")
        #     self.agent_model_type = random.choice(bps).id

        red = str(random.randint(0, 255))
        blue = str(random.randint(0, 255))
        green = str(random.randint(0, 255))
        ego_vehicle_color = ",".join([red, blue, green])

        if BridgeHelpers.get_agent_actor(CarlaDataProvider.get_world(), self.agent_role_name) is None:
            CarlaDataProvider.request_new_actor(self.agent_model_type, spawn_point, self.agent_role_name, random_location=randomize, color=ego_vehicle_color)
        else:
            CarlaDataProvider.cleanup()
            raise RuntimeError(f"The vehicle {self.agent_role_name} has been spawned!")
        
        return self.client
    
    def cleanup(self):
        # if len(CarlaDataProvider.get_world().get_actors().filter("vehicle.*")) == 0:
        vehicle = BridgeHelpers.get_agent_actor(CarlaDataProvider.get_world(), self.agent_role_name)
        if vehicle is not None:
            vehicle.destroy()
        CarlaDataProvider.cleanup()

if __name__ == "__main__":
    
    if os.environ["CONTROL_MODE"] != "pygame" and os.environ["RUNNING_MODE"] == "record":
        raise ValueError("Only in the pygame control mode, the running mode can be set to record!")
    else:
        ego_vehicle_launcher = EgoVehicleLauncher()
        client = ego_vehicle_launcher.load_vehicle()
