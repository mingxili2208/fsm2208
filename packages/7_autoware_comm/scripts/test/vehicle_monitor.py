#!/usr/bin/env python3
# vehicle_monitor.py

import argparse
import carla
from vehicle_monitor_module import VehicleMonitor  # 注意：这是你定义 VehicleMonitor 类的模块名

def main():
    parser = argparse.ArgumentParser(description="CARLA Vehicle Monitor UI")
    parser.add_argument('--host', type=str, default='127.0.0.1', help='CARLA server host')
    parser.add_argument('--port', type=int, default=2000, help='CARLA server port')
    parser.add_argument('--role-name', type=str, default='pygame_adtruck', help='Vehicle role name in CARLA')
    args = parser.parse_args()

    # Connect to CARLA
    client = carla.Client(args.host, args.port)
    client.set_timeout(10.0)
    world = client.get_world()

    # Find vehicle by role name
    vehicle = None
    for actor in world.get_actors().filter('vehicle.*'):
        if actor.attributes.get('role_name') == args.role_name:
            vehicle = actor
            break

    if vehicle is None:
        print(f"[ERROR] Vehicle with role_name '{args.role_name}' not found.")
        return

    # Run the vehicle monitor
    monitor = VehicleMonitor(client, vehicle, role_name=args.role_name)
    monitor.run()

if __name__ == '__main__':
    main()