#!/usr/bin/env python

# Copyright (c) 2021 Computer Vision Center (CVC) at the Universitat Autonoma de
# Barcelona (UAB).
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""Script to generate traffic in CARLA simulation with Autoware integration and spawn point safety check"""

import glob
import os
import sys
import time
import json

try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

import carla
import math
import argparse
import logging
from numpy import random

def get_actor_blueprints(world, filter, generation):
    bps = world.get_blueprint_library().filter(filter)

    if generation.lower() == "all":
        return bps

    # If the filter returns only one bp, we assume that this one needed
    # and therefore, we ignore the generation
    if len(bps) == 1:
        return bps

    try:
        int_generation = int(generation)
        # Check if generation is in available generations
        if int_generation in [1, 2, 3]:
            bps = [x for x in bps if int(x.get_attribute('generation')) == int_generation]
            return bps
        else:
            print("   Warning! Actor Generation is not valid. No actor will be spawned.")
            return []
    except:
        print("   Warning! Actor Generation is not valid. No actor will be spawned.")
        return []

def is_spawn_point_safe(world, spawn_point, safety_distance=3.0):
    """
    Check if a spawn point is safe (no vehicles or obstacles nearby)
    
    Args:
        world: CARLA world object
        spawn_point: carla.Transform object representing the spawn location
        safety_distance: Minimum safe distance in meters
        
    Returns:
        bool: True if the spawn point is safe, False otherwise
    """
    # Get the spawn point location
    spawn_location = spawn_point.location
    
    # Check for vehicles nearby
    vehicle_list = world.get_actors().filter('vehicle.*')
    for vehicle in vehicle_list:
        distance = vehicle.get_location().distance(spawn_location)
        if distance < safety_distance:
            return False
    
    # Check for obstacles using ray casting
    # Check 4 rays in different directions around the spawn point
    directions = [
        carla.Vector3D(1, 0, 0),   # Forward
        carla.Vector3D(-1, 0, 0),  # Backward
        carla.Vector3D(0, 1, 0),   # Right
        carla.Vector3D(0, -1, 0)   # Left
    ]
    
    # Starting position slightly above the ground to avoid hitting the ground itself
    start_location = carla.Location(
        spawn_location.x, 
        spawn_location.y, 
        spawn_location.z + 0.5
    )
    
    for direction in directions:
        # Cast a ray from the spawn point
        end_location = carla.Location(
            start_location.x + direction.x * safety_distance,
            start_location.y + direction.y * safety_distance,
            start_location.z + direction.z * safety_distance
        )
        
        # Ray tracing to check for obstacles
        hit = world.cast_ray(start_location, end_location)
        if hit:
            return False
    
    # Check for walkers nearby
    walker_list = world.get_actors().filter('walker.*')
    for walker in walker_list:
        distance = walker.get_location().distance(spawn_location)
        if distance < safety_distance:
            return False
    
    return True

def find_safe_spawn_points(world, spawn_points, safety_distance=3.0, max_attempts=100):
    """
    Find safe spawn points for vehicles
    
    Args:
        world: CARLA world object
        spawn_points: List of potential spawn points
        safety_distance: Minimum safe distance in meters
        max_attempts: Maximum number of attempts to find safe points
        
    Returns:
        list: List of safe spawn points
    """
    safe_spawn_points = []
    attempts = 0
    
    # First, check original spawn points
    for spawn_point in spawn_points:
        if is_spawn_point_safe(world, spawn_point, safety_distance):
            safe_spawn_points.append(spawn_point)
    
    # If we don't have enough safe points, try to find more
    while len(safe_spawn_points) < len(spawn_points) and attempts < max_attempts:
        # Pick a random spawn point and try small variations around it
        if not spawn_points:
            break
        
        base_point = random.choice(spawn_points)
        
        # Try a slightly modified position
        offset_x = random.uniform(-2.0, 2.0)
        offset_y = random.uniform(-2.0, 2.0)
        
        new_transform = carla.Transform(
            carla.Location(
                base_point.location.x + offset_x,
                base_point.location.y + offset_y,
                base_point.location.z
            ),
            base_point.rotation
        )
        
        if is_spawn_point_safe(world, new_transform, safety_distance):
            safe_spawn_points.append(new_transform)
        
        attempts += 1
    
    return safe_spawn_points

def main():
    argparser = argparse.ArgumentParser(
        description=__doc__)
    argparser.add_argument(
        '--host',
        metavar='H',
        default='127.0.0.1',
        help='IP of the host server (default: 127.0.0.1)')
    argparser.add_argument(
        '-p', '--port',
        metavar='P',
        default=2000,
        type=int,
        help='TCP port to listen to (default: 2000)')
    argparser.add_argument(
        '-n', '--number-of-vehicles',
        metavar='N',
        default=15,
        type=int,
        help='Number of vehicles (default: 15)')
    argparser.add_argument(
        '-w', '--number-of-walkers',
        metavar='W',
        default=10,
        type=int,
        help='Number of walkers (default: 10)')
    argparser.add_argument(
        '--safe',
        action='store_true',
        help='Avoid spawning vehicles prone to accidents')
    argparser.add_argument(
        '--filterv',
        metavar='PATTERN',
        default='vehicle.*',
        help='Filter vehicle model (default: "vehicle.*")')
    argparser.add_argument(
        '--generationv',
        metavar='G',
        default='All',
        help='restrict to certain vehicle generation (values: "1","2","All" - default: "All")')
    argparser.add_argument(
        '--filterw',
        metavar='PATTERN',
        default='walker.pedestrian.*',
        help='Filter pedestrian type (default: "walker.pedestrian.*")')
    argparser.add_argument(
        '--generationw',
        metavar='G',
        default='2',
        help='restrict to certain pedestrian generation (values: "1","2","All" - default: "2")')
    argparser.add_argument(
        '--asynch',
        action='store_true',
        help='Activate asynchronous mode execution')
    argparser.add_argument(
        '--seed',
        metavar='S',
        type=int,
        default=None,
        help='Set random device seed for reproducibility')
    argparser.add_argument(
        '--no-rendering',
        action='store_true',
        default=False,
        help='Activate no rendering mode')
    argparser.add_argument(
        '--safety-distance',
        metavar='D',
        default=3.0,
        type=float,
        help='Minimum safe distance between vehicles when spawning (default: 3.0 meters)')
    argparser.add_argument(
        '--config-file',
        metavar='C',
        default=None,
        help='Configuration file path (optional)')

    args = argparser.parse_args()

    # Check if a configuration file is provided and load settings from it
    if args.config_file and os.path.exists(args.config_file):
        with open(args.config_file, 'r') as f:
            config = json.load(f)
            # Override arguments with config file values if they exist
            for key, value in config.items():
                if hasattr(args, key.replace('-', '_')):
                    setattr(args, key.replace('-', '_'), value)

    logging.basicConfig(format='%(levelname)s: %(message)s', level=logging.INFO)

    vehicles_list = []
    walkers_list = []
    all_id = []
    client = carla.Client(args.host, args.port)
    client.set_timeout(10.0)
    synchronous_master = False
    random.seed(args.seed if args.seed is not None else int(time.time()))

    try:
        world = client.get_world()

        # Set up Autoware-compatible traffic manager
        traffic_manager = client.get_trafficmanager(8000)
        traffic_manager.set_global_distance_to_leading_vehicle(2.5)
        
        # Enable Autoware compatibility settings
        traffic_manager.set_random_device_seed(args.seed if args.seed is not None else int(time.time()))
        
        settings = world.get_settings()
        if not args.asynch:
            traffic_manager.set_synchronous_mode(True)
            if not settings.synchronous_mode:
                synchronous_master = True
                settings.synchronous_mode = True
                settings.fixed_delta_seconds = 0.05
            else:
                synchronous_master = False
        else:
            print("You are currently in asynchronous mode. If this is a traffic simulation, \
            you could experience some issues. If it's not working correctly, switch to synchronous \
            mode by using traffic_manager.set_synchronous_mode(True)")

        if args.no_rendering:
            settings.no_rendering_mode = True
        world.apply_settings(settings)

        # Get blueprints for vehicles and walkers
        blueprints = get_actor_blueprints(world, args.filterv, args.generationv)
        if not blueprints:
            raise ValueError("Couldn't find any vehicles with the specified filters")
        
        blueprintsWalkers = get_actor_blueprints(world, args.filterw, args.generationw)
        if not blueprintsWalkers:
            raise ValueError("Couldn't find any walkers with the specified filters")

        if args.safe:
            blueprints = [x for x in blueprints if x.get_attribute('base_type') == 'car']

        blueprints = sorted(blueprints, key=lambda bp: bp.id)

        # Get all spawn points from the map
        original_spawn_points = world.get_map().get_spawn_points()
        
        if len(original_spawn_points) < args.number_of_vehicles:
            logging.warning('Requested %d vehicles but only %d spawn points available',
                            args.number_of_vehicles, len(original_spawn_points))
            args.number_of_vehicles = len(original_spawn_points)
        
        # Shuffle spawn points
        random.shuffle(original_spawn_points)
        
        # Get spawn points that are safe (no vehicles or obstacles nearby)
        logging.info('Finding safe spawn points for vehicles...')
        spawn_points = find_safe_spawn_points(
            world, 
            original_spawn_points[:args.number_of_vehicles], 
            args.safety_distance
        )
        
        logging.info('Found %d safe spawn points out of %d requested vehicles',
                     len(spawn_points), args.number_of_vehicles)
        
        # Adjust number of vehicles to match available safe spawn points
        args.number_of_vehicles = min(args.number_of_vehicles, len(spawn_points))

        # Spawn commands
        SpawnActor = carla.command.SpawnActor
        SetAutopilot = carla.command.SetAutopilot
        FutureActor = carla.command.FutureActor

        # --------------
        # Spawn vehicles
        # --------------
        batch = []
        for n, transform in enumerate(spawn_points):
            if n >= args.number_of_vehicles:
                break
            
            # Choose a random blueprint
            blueprint = random.choice(blueprints)
            
            # Set random color if available
            if blueprint.has_attribute('color'):
                color = random.choice(blueprint.get_attribute('color').recommended_values)
                blueprint.set_attribute('color', color)
            
            # Set random driver ID if available
            if blueprint.has_attribute('driver_id'):
                driver_id = random.choice(blueprint.get_attribute('driver_id').recommended_values)
                blueprint.set_attribute('driver_id', driver_id)
            
            # Set all vehicles to be controlled by traffic manager (Autoware)
            blueprint.set_attribute('role_name', 'autopilot')

            # Double-check if spawn point is still safe before adding to batch
            if is_spawn_point_safe(world, transform, args.safety_distance):
                # Spawn the vehicle and set it to autopilot
                batch.append(SpawnActor(blueprint, transform)
                    .then(SetAutopilot(FutureActor, True, traffic_manager.get_port())))
        
        logging.info('Spawning %d vehicles...', len(batch))
        
        # Apply the batch and collect the vehicle IDs
        for response in client.apply_batch_sync(batch, synchronous_master):
            if response.error:
                logging.error(response.error)
            else:
                vehicles_list.append(response.actor_id)
        
        logging.info('Successfully spawned %d vehicles', len(vehicles_list))

        # -------------
        # Spawn Walkers
        # -------------
        logging.info('Spawning walkers...')
        
        # First, generate random spawn points for the walkers
        spawn_points = []
        spawn_attempts = 0
        max_spawn_attempts = args.number_of_walkers * 5  # Try harder to find walker spawn points
        
        while len(spawn_points) < args.number_of_walkers and spawn_attempts < max_spawn_attempts:
            spawn_point = carla.Transform()
            loc = world.get_random_location_from_navigation()
            if loc is not None:
                spawn_point.location = loc
                # Check if this point is safe for spawning
                if is_spawn_point_safe(world, spawn_point, args.safety_distance):
                    spawn_points.append(spawn_point)
            spawn_attempts += 1
        
        logging.info('Found %d safe spawn points for walkers out of %d requested',
                    len(spawn_points), args.number_of_walkers)
        
        # Spawn walker actors
        batch = []
        walker_speed = []
        for spawn_point in spawn_points:
            walker_bp = random.choice(blueprintsWalkers)
            
            # Set walker as not invincible
            if walker_bp.has_attribute('is_invincible'):
                walker_bp.set_attribute('is_invincible', 'false')
            
            # Set random walking speed
            if walker_bp.has_attribute('speed'):
                # All walkers are set to walking speed (not running)
                walker_speed.append(walker_bp.get_attribute('speed').recommended_values[1])
            else:
                print("Walker has no speed")
                walker_speed.append(0.0)
            
            batch.append(SpawnActor(walker_bp, spawn_point))
        
        results = client.apply_batch_sync(batch, True)
        walker_speed2 = []
        for i in range(len(results)):
            if results[i].error:
                logging.error(results[i].error)
            else:
                walkers_list.append({"id": results[i].actor_id})
                walker_speed2.append(walker_speed[i])
        walker_speed = walker_speed2
        
        logging.info('Successfully spawned %d walkers', len(walkers_list))
        
        # Spawn walker controllers for AI movement
        batch = []
        walker_controller_bp = world.get_blueprint_library().find('controller.ai.walker')
        for i in range(len(walkers_list)):
            batch.append(SpawnActor(walker_controller_bp, carla.Transform(), walkers_list[i]["id"]))
        
        results = client.apply_batch_sync(batch, True)
        for i in range(len(results)):
            if results[i].error:
                logging.error(results[i].error)
            else:
                walkers_list[i]["con"] = results[i].actor_id
        
        # Collect all walker and controller IDs
        for i in range(len(walkers_list)):
            all_id.append(walkers_list[i]["con"])
            all_id.append(walkers_list[i]["id"])
        all_actors = world.get_actors(all_id)

        # Wait for a tick to ensure client receives the last transform of the walkers
        if args.asynch or not synchronous_master:
            world.wait_for_tick()
        else:
            world.tick()

        # Initialize walker controllers and set random destinations
        # Set cross factor to 0.5 so pedestrians can cross roads
        world.set_pedestrians_cross_factor(0.5)
        for i in range(0, len(all_id), 2):
            # Start walker
            all_actors[i].start()
            # Set walk to random point
            all_actors[i].go_to_location(world.get_random_location_from_navigation())
            # Set max speed
            all_actors[i].set_max_speed(float(walker_speed[int(i/2)]))

        print('Spawned %d vehicles and %d walkers, managed by Autoware' % (len(vehicles_list), len(walkers_list)))
        print('Press Ctrl+C to exit.')

        # Set traffic manager global speed adjustment
        traffic_manager.global_percentage_speed_difference(0.0)  # Normal speed

        # Main simulation loop
        while True:
            if not args.asynch and synchronous_master:
                world.tick()
            else:
                world.wait_for_tick()

    finally:
        # Clean up when exiting
        if not args.asynch and synchronous_master:
            settings = world.get_settings()
            settings.synchronous_mode = False
            settings.no_rendering_mode = False
            settings.fixed_delta_seconds = None
            world.apply_settings(settings)

        print('\nDestroying %d vehicles' % len(vehicles_list))
        client.apply_batch([carla.command.DestroyActor(x) for x in vehicles_list])

        # Stop walker controllers
        for i in range(0, len(all_id), 2):
            all_actors[i].stop()

        print('\nDestroying %d walkers' % len(walkers_list))
        client.apply_batch([carla.command.DestroyActor(x) for x in all_id])

        time.sleep(0.5)
        print('\nDone.')

if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        pass
    except Exception as e:
        logging.error(e)
    finally:
        print('\nDone.')