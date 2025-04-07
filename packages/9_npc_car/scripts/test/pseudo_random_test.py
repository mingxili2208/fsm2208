#!/usr/bin/env python

# Copyright (c) 2021 Computer Vision Center (CVC) at the Universitat Autonoma de
# Barcelona (UAB).
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""Minimal script to generate NPC vehicles navigating between spawn points in CARLA"""

import glob
import os
import sys
import time
import json
import random as py_random
import signal
import atexit

try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

import carla
import argparse
import logging
import numpy as np

# Global variables for cleanup
global_client = None
global_vehicles_list = []
cleanup_done = False

def cleanup_all_actors():
    """Cleanup function to remove all spawned actors"""
    global global_client, global_vehicles_list, cleanup_done
    
    if cleanup_done or not global_client:
        return
    
    cleanup_done = True
    
    try:
        if global_vehicles_list and len(global_vehicles_list) > 0:
            print(f"\nDestroying {len(global_vehicles_list)} vehicles...")
            # Use apply_batch instead of apply_batch_sync to avoid waiting for responses
            # which could cause issues if the simulation is already shutting down
            global_client.apply_batch([carla.command.DestroyActor(x) for x in global_vehicles_list])
            global_vehicles_list = []
        time.sleep(0.5)
        print("Cleanup complete.")
    except Exception as e:
        print(f"Error during cleanup: {e}")

def signal_handler(sig, frame):
    """Handle signals like Ctrl+C"""
    print("\nReceived termination signal. Cleaning up...")
    cleanup_all_actors()
    sys.exit(0)

def get_actor_blueprints(world, filter, generation):
    bps = world.get_blueprint_library().filter(filter)

    if generation.lower() == "all":
        return bps

    if len(bps) == 1:
        return bps

    try:
        int_generation = int(generation)
        if int_generation in [1, 2, 3]:
            bps = [x for x in bps if int(x.get_attribute('generation')) == int_generation]
            return bps
        else:
            print("   Warning! Actor Generation is not valid. No actor will be spawned.")
            return []
    except:
        print("   Warning! Actor Generation is not valid. No actor will be spawned.")
        return []

def simple_is_spawn_point_safe(world, spawn_point, safety_distance=3.0):
    """
    Simplified safe spawn check that only checks for nearby vehicles
    """
    # Get the spawn point location
    spawn_location = spawn_point.location
    
    # Only check for vehicles (most common obstacle)
    vehicle_list = world.get_actors().filter('vehicle.*')
    for vehicle in vehicle_list:
        distance = vehicle.get_location().distance(spawn_location)
        if distance < safety_distance:
            return False
    
    return True

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
    argparser.add_argument(
        '--num-vehicles',
        type=int,
        default=5,
        help='Number of NPC vehicles to spawn (default: 5)')
    argparser.add_argument(
        '--nav-style',
        choices=['circular', 'back_and_forth', 'random'],
        default='circular',
        help='Navigation style between points (default: circular)')
    argparser.add_argument(
        '--num-waypoints',
        type=int,
        default=10,
        help='Number of waypoints to use in the navigation route (default: 10)')
    argparser.add_argument(
        '--speed-factor',
        type=float,
        default=30.0,
        help='Percentage to reduce speed from speed limit (default: 30.0, means 30%% slower than speed limit)')
    argparser.add_argument(
        '--target-speed',
        type=float,
        default=20.0,
        help='Target speed in km/h (default: 20.0 km/h)')
    argparser.add_argument(
        '--check-interval',
        type=int,
        default=100,
        help='Number of ticks between destination checks (default: 100)')

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

    global global_client, global_vehicles_list
    vehicles_list = []
    
    client = carla.Client(args.host, args.port)
    client.set_timeout(10.0)
    global_client = client
    
    synchronous_master = False
    
    # Set random seed for reproducibility
    if args.seed is not None:
        py_random.seed(args.seed)
        np.random.seed(args.seed)

    try:
        world = client.get_world()

        # Set up traffic manager
        traffic_manager = client.get_trafficmanager(8000)
        traffic_manager.set_global_distance_to_leading_vehicle(3.0)
        
        if args.seed is not None:
            traffic_manager.set_random_device_seed(args.seed)

        # Configure synchronous mode
        settings = world.get_settings()
        if not args.asynch:
            traffic_manager.set_synchronous_mode(True)
            if not settings.synchronous_mode:
                synchronous_master = True
                settings.synchronous_mode = True
                settings.fixed_delta_seconds = 0.05
            else:
                synchronous_master = False
        world.apply_settings(settings)

        # Get vehicle blueprints
        blueprints = get_actor_blueprints(world, args.filterv, args.generationv)
        if not blueprints:
            raise ValueError("Couldn't find any vehicles with the specified filters")
        
        if args.safe:
            blueprints = [x for x in blueprints if x.get_attribute('base_type') == 'car']

        # Get spawn points
        spawn_points = world.get_map().get_spawn_points()
        
        if len(spawn_points) < 2:
            raise ValueError("Not enough spawn points available (need at least 2)")
        
        # Shuffle spawn points
        py_random.shuffle(spawn_points)
        
        # Select subset of points based on num_waypoints
        num_waypoints = min(args.num_waypoints, len(spawn_points))
        nav_points = spawn_points[:num_waypoints]
        
        # For circular navigation, sort points by angle around center
        if args.nav_style == 'circular':
            # Find center of all points
            center_x = sum(p.location.x for p in nav_points) / len(nav_points)
            center_y = sum(p.location.y for p in nav_points) / len(nav_points)
            center = carla.Location(x=center_x, y=center_y)
            
            # Sort by angle
            nav_points = sorted(nav_points, 
                                key=lambda p: np.arctan2(p.location.y - center.y, 
                                                        p.location.x - center.x))
        
        elif args.nav_style == 'back_and_forth':
            # Just use first two points
            start_point = nav_points[0]
            end_point = max(nav_points[1:], key=lambda p: p.location.distance(start_point.location))
            nav_points = [start_point, end_point]
        
        logging.info(f"Selected {len(nav_points)} navigation points using {args.nav_style} style")
        
        # Limit vehicles to number of nav points
        num_vehicles = min(args.num_vehicles, len(nav_points))
        logging.info(f"Preparing to spawn {num_vehicles} vehicles")
        
        # Simple structure to track vehicle destinations
        vehicle_destinations = {}
        
        # Spawn commands
        batch = []
        vehicle_navigation_indices = {}
        
        for i in range(num_vehicles):
            # Pick a spawn point that's relatively safe
            for j in range(len(nav_points)):
                spawn_idx = (i + j) % len(nav_points)
                if simple_is_spawn_point_safe(world, nav_points[spawn_idx], args.safety_distance):
                    break
            
            # Choose a random blueprint
            blueprint = py_random.choice(blueprints)
            
            # Set random color if available
            if blueprint.has_attribute('color'):
                color = py_random.choice(blueprint.get_attribute('color').recommended_values)
                blueprint.set_attribute('color', color)
            
            # Set vehicle role name
            blueprint.set_attribute('role_name', f'npc_vehicle_{i}')
            
            # Spawn vehicle with autopilot
            batch.append(carla.command.SpawnActor(blueprint, nav_points[spawn_idx])
                         .then(carla.command.SetAutopilot(carla.command.FutureActor, True, traffic_manager.get_port())))
            
            # Store the navigation index for this vehicle (pending ID)
            vehicle_navigation_indices[i] = {
                'current_idx': spawn_idx,
                'next_idx': (spawn_idx + 1) % len(nav_points)
            }
        
        # Apply batch spawn
        for i, response in enumerate(client.apply_batch_sync(batch, synchronous_master)):
            if response.error:
                logging.error(f"Error spawning vehicle: {response.error}")
            else:
                # Success - store the vehicle ID
                vehicles_list.append(response.actor_id)
                # Copy navigation indices to destination dict with actual vehicle ID
                if i in vehicle_navigation_indices:
                    vehicle_destinations[response.actor_id] = vehicle_navigation_indices[i]
        
        # Update global vehicle list for cleanup
        global_vehicles_list = vehicles_list.copy()
        
        logging.info(f"Successfully spawned {len(vehicles_list)} vehicles")
        
        # Wait for a tick to ensure vehicles are properly spawned
        if not args.asynch and synchronous_master:
            world.tick()
        else:
            world.wait_for_tick()
        
        # Get spawned vehicle actors
        vehicle_actors = world.get_actors(vehicles_list)
        
        # Configure vehicle speeds and behaviors
        for vehicle in vehicle_actors:
            # Set target speed directly if specified
            if args.target_speed > 0:
                traffic_manager.set_desired_speed(vehicle, args.target_speed / 3.6)  # Convert km/h to m/s
            else:
                # Otherwise use speed factor
                traffic_manager.vehicle_percentage_speed_difference(vehicle, args.speed_factor)
            
            # Configure for safer driving
            try:
                traffic_manager.auto_lane_change(vehicle, False)  # Disable lane changes
                traffic_manager.distance_to_leading_vehicle(vehicle, 5.0)  # Keep safe distance
                traffic_manager.ignore_lights_percentage(vehicle, 0)  # Obey traffic lights
                traffic_manager.ignore_vehicles_percentage(vehicle, 0)  # Respect other vehicles
            except Exception:
                logging.warning("Some traffic manager settings failed - using defaults")
            
            # Set initial destination
            vehicle_id = vehicle.id
            if vehicle_id in vehicle_destinations:
                next_idx = vehicle_destinations[vehicle_id]['next_idx']
                next_point = nav_points[next_idx]
                
                try:
                    # Try to set explicit path if supported
                    traffic_manager.set_path(vehicle, [next_point.location])
                except:
                    # If path setting fails, rely on autopilot
                    pass
        
        # Main simulation loop
        print(f'Spawned {len(vehicles_list)} vehicles. Press Ctrl+C to exit.')
        print(f'Target speed: {args.target_speed} km/h' if args.target_speed > 0 else 
              f'Speed: {args.speed_factor}% slower than limits')
        
        # Counter for destination checks
        check_counter = 0
        arrival_threshold = 5.0
        
        while True:
            # Tick the world
            if not args.asynch and synchronous_master:
                world.tick()
            else:
                world.wait_for_tick()
            
            # Only check destinations periodically to reduce CPU usage
            check_counter += 1
            if check_counter >= args.check_interval:
                check_counter = 0
                
                # Check which vehicles have reached their destinations
                try:
                    for vehicle_id in list(vehicle_destinations.keys()):
                        try:
                            vehicle = world.get_actor(vehicle_id)
                            nav_idx = vehicle_destinations[vehicle_id]
                            
                            # Current destination
                            current_idx = nav_idx['next_idx']
                            current_point = nav_points[current_idx]
                            
                            # Check if vehicle reached destination
                            distance = vehicle.get_location().distance(current_point.location)
                            
                            if distance < arrival_threshold:
                                # Update indices - move to next point
                                new_current = current_idx
                                new_next = (current_idx + 1) % len(nav_points)
                                
                                vehicle_destinations[vehicle_id] = {
                                    'current_idx': new_current,
                                    'next_idx': new_next
                                }
                                
                                # Set new destination
                                try:
                                    next_point = nav_points[new_next]
                                    traffic_manager.set_path(vehicle, [next_point.location])
                                except:
                                    pass  # Autopilot will handle it
                        except:
                            # Vehicle may have been destroyed
                            if vehicle_id in vehicle_destinations:
                                del vehicle_destinations[vehicle_id]
                except Exception as e:
                    logging.warning(f"Error in destination check: {e}")

    except KeyboardInterrupt:
        pass
    
    except Exception as e:
        logging.error(f"An error occurred: {e}")
    
    finally:
        # Reset synchronous mode if needed
        if not args.asynch and synchronous_master:
            try:
                settings = world.get_settings()
                settings.synchronous_mode = False
                settings.fixed_delta_seconds = None
                world.apply_settings(settings)
            except:
                pass

        # Call cleanup function
        cleanup_all_actors()

if __name__ == '__main__':
    # Register signal handlers
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    
    # Register cleanup for normal exits
    atexit.register(cleanup_all_actors)
    
    try:
        main()
    except KeyboardInterrupt:
        pass
    except Exception as e:
        logging.error(f"Fatal error: {e}")
    finally:
        cleanup_all_actors()
        print('Done.')