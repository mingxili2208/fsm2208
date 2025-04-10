# vehicle_monitor.py

import carla
import pygame
import numpy as np
import math
import logging
import os
import psutil

class VehicleMonitor:
    def __init__(self, client, vehicle, role_name="pygame_agent"):
        self.client = client
        self.world = client.get_world()
        self.vehicle = vehicle
        self.role_name = role_name

        self.cam_width = 400
        self.cam_height = 300
        self.screen_width = self.cam_width * 2
        self.screen_height = self.cam_height * 2

        pygame.init()
        self.screen = pygame.display.set_mode((self.screen_width, self.screen_height), pygame.RESIZABLE)
        pygame.display.set_caption(f"Monitor - {self.role_name}")
        self.clock = pygame.time.Clock()

        self.cameras = {}
        self.camera_images = {}

        self.running = True
        self.control = carla.VehicleControl()
        self.throttle = False
        self.brake = False
        self.steer = None
        self.steer_cache = 0.0

        self.setup_cameras()

    def setup_cameras(self):
        blueprint_library = self.world.get_blueprint_library()

        def create_camera(name, transform):
            cam_bp = blueprint_library.find('sensor.camera.rgb')
            cam_bp.set_attribute('image_size_x', str(self.cam_width))
            cam_bp.set_attribute('image_size_y', str(self.cam_height))
            cam_bp.set_attribute('fov', '90')
            cam = self.world.spawn_actor(cam_bp, transform, attach_to=self.vehicle)
            cam.listen(lambda image: self.process_image(image, name))
            self.cameras[name] = cam

        create_camera('front', carla.Transform(carla.Location(x=1.5, z=2.4)))
        create_camera('back', carla.Transform(carla.Location(x=-1.5, z=2.4), carla.Rotation(yaw=180)))
        create_camera('bev', carla.Transform(carla.Location(x=0, z=15), carla.Rotation(pitch=-90)))

    def process_image(self, image, name):
        array = np.frombuffer(image.raw_data, dtype=np.uint8)
        array = np.reshape(array, (image.height, image.width, 4))
        array = array[:, :, :3]
        array = array[:, :, ::-1]
        self.camera_images[name] = array

    def run(self):
        try:
            while self.running:
                self.handle_events()
                self.visualization()
                pygame.display.flip()
                self.clock.tick(20)
        except KeyboardInterrupt:
            pass
        finally:
            self.stop()

    def handle_events(self):
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                self.running = False
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    self.running = False
                elif event.key == pygame.K_w:
                    self.throttle = True
                elif event.key == pygame.K_s:
                    self.brake = True
                elif event.key == pygame.K_a:
                    self.steer = -1
                elif event.key == pygame.K_d:
                    self.steer = 1
            elif event.type == pygame.KEYUP:
                if event.key == pygame.K_w:
                    self.throttle = False
                elif event.key == pygame.K_s:
                    self.brake = False
                elif event.key in (pygame.K_a, pygame.K_d):
                    self.steer = None

        self.control_vehicle()

    def control_vehicle(self):
        if self.throttle:
            self.control.throttle = min(self.control.throttle + 0.05, 0.6)
        else:
            self.control.throttle = 0.0

        if self.brake:
            self.control.brake = 1.0
        else:
            self.control.brake = 0.0

        if self.steer is not None:
            self.steer_cache += 0.05 * self.steer
            self.steer_cache = max(-1.0, min(1.0, self.steer_cache))
        else:
            self.steer_cache *= 0.8
            if abs(self.steer_cache) < 0.01:
                self.steer_cache = 0.0

        self.control.steer = round(self.steer_cache, 2)
        self.vehicle.apply_control(self.control)

    def visualization(self):
        positions = {
            'bev': (0, 0),
            'back': (self.cam_width, 0),
            'front': (0, self.cam_height)
        }
        for name, image in self.camera_images.items():
            if name in positions:
                surface = pygame.surfarray.make_surface(image.swapaxes(0, 1))
                self.screen.blit(surface, positions[name])

    def stop(self):
        for cam in self.cameras.values():
            cam.stop()
            cam.destroy()
        pygame.quit()