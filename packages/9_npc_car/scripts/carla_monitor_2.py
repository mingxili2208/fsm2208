#!/usr/bin/env python

import carla
import time
import numpy as np
import threading
import sys
import os
from collections import deque

class CarlaMonitor:
    def __init__(self, host='localhost', port=2000):
        # Connect to CARLA
        self.client = carla.Client(host, port)
        self.client.set_timeout(10.0)
        self.world = self.client.get_world()
        self.tm = self.client.get_trafficmanager()
        
        # Initialize performance monitoring variables
        self.frame_times = deque(maxlen=100)
        self.frame_count = 0
        self.last_frame_time = time.time()
        
        # Physics timing tracking
        self.physics_step_times = deque(maxlen=100)
        self.last_physics_step = 0
        self.last_physics_time = 0
        
        # NPC movement tracking
        self.npc_positions = {}
        self.npc_velocities = {}
        self.npc_accelerations = {}
        self.npc_last_update = {}
        self.npc_jerk = {}
        self.npc_motion_history = {}
        
        # Setup callback
        self.callback_id = self.world.on_tick(self.on_world_tick)
        
        # Monitoring flags
        self.running = True
        
        # Monitoring thread
        self.monitor_thread = threading.Thread(target=self.monitor_loop)

    def start(self):
        """Start monitoring"""
        self.monitor_thread.start()
        self.main_loop()

    def on_world_tick(self, timestamp):
        """CARLA world update callback"""
        current_time = time.time()
        current_step = timestamp.frame
        
        # Track frame rate
        if self.frame_count > 0:
            frame_time = current_time - self.last_frame_time
            self.frame_times.append(frame_time)
        
        self.frame_count += 1
        self.last_frame_time = current_time
        
        # Track physics steps
        if self.last_physics_step > 0:
            # Only record consecutive steps
            if current_step - self.last_physics_step == 1:
                step_time = timestamp.elapsed_seconds - self.last_physics_time
                self.physics_step_times.append(step_time)
        
        self.last_physics_step = current_step
        self.last_physics_time = timestamp.elapsed_seconds
        
        # Get all NPCs
        vehicles = self.world.get_actors().filter('vehicle.*')
        for vehicle in vehicles:
            if vehicle.attributes.get('role_name') != 'hero':  # Exclude player vehicle
                veh_id = vehicle.id
                
                # Record position and velocity
                current_pos = vehicle.get_location()
                current_vel = vehicle.get_velocity()
                current_vel_magnitude = np.sqrt(current_vel.x**2 + current_vel.y**2 + current_vel.z**2)
                
                # Calculate motion metrics
                if veh_id in self.npc_positions and veh_id in self.npc_velocities:
                    last_pos = self.npc_positions[veh_id]
                    last_vel = self.npc_velocities[veh_id]
                    last_vel_magnitude = np.sqrt(last_vel.x**2 + last_vel.y**2 + last_vel.z**2)
                    last_update = self.npc_last_update.get(veh_id, current_time - 0.033)
                    
                    # Calculate time delta
                    dt = current_time - last_update
                    if dt > 0.001:  # Avoid division by near-zero
                        # Calculate acceleration (change in velocity)
                        accel = (current_vel_magnitude - last_vel_magnitude) / dt
                        
                        # Store in motion history
                        if veh_id not in self.npc_motion_history:
                            self.npc_motion_history[veh_id] = deque(maxlen=30)
                        
                        self.npc_motion_history[veh_id].append({
                            'time': current_time,
                            'position': current_pos,
                            'velocity': current_vel_magnitude,
                            'acceleration': accel
                        })
                        
                        # If we have previous acceleration, calculate jerk
                        if veh_id in self.npc_accelerations:
                            prev_accel = self.npc_accelerations[veh_id]
                            jerk = abs(accel - prev_accel) / dt
                            
                            if veh_id in self.npc_jerk:
                                self.npc_jerk[veh_id].append(jerk)
                                if len(self.npc_jerk[veh_id]) > 30:
                                    self.npc_jerk[veh_id].pop(0)
                            else:
                                self.npc_jerk[veh_id] = [jerk]
                        
                        # Store current acceleration for next frame
                        self.npc_accelerations[veh_id] = accel
                
                # Update data
                self.npc_positions[veh_id] = current_pos
                self.npc_velocities[veh_id] = current_vel
                self.npc_last_update[veh_id] = current_time

    def get_traffic_manager_info(self):
        """Get traffic manager information"""
        try:
            # Get traffic manager settings
            is_sync = self.tm.get_synchronous_mode()
            global_distance = self.tm.get_global_distance_to_leading_vehicle()
            tm_port = self.tm.get_port()
            
            # Get vehicles controlled by TM
            tm_controlled = 0
            vehicles = self.world.get_actors().filter('vehicle.*')
            for vehicle in vehicles:
                if not vehicle.attributes.get('role_name') == 'hero':
                    tm_controlled += 1
            
            # Try to get hybrid physics mode
            hybrid_physics = False
            try:
                hybrid_physics = self.tm.get_hybrid_physics_mode()
            except:
                hybrid_physics = "Unknown"
                
            # Estimate TM update rate from world settings if in sync mode
            settings = self.world.get_settings()
            update_rate = 0.0
            if settings.synchronous_mode and is_sync:
                if settings.fixed_delta_seconds:
                    update_rate = 1.0 / settings.fixed_delta_seconds
            
            return {
                'sync_mode': is_sync,
                'global_distance': global_distance,
                'port': tm_port,
                'controlled_vehicles': tm_controlled,
                'hybrid_physics': hybrid_physics,
                'estimated_update_rate': update_rate
            }
        except Exception as e:
            return {'error': str(e)}

    def get_carla_settings(self):
        """Get CARLA world settings"""
        settings = self.world.get_settings()
        return {
            'synchronous_mode': settings.synchronous_mode,
            'fixed_delta_seconds': settings.fixed_delta_seconds,
            'substepping': settings.substepping,
            'max_substep_delta_time': settings.max_substep_delta_time,
            'max_substeps': settings.max_substeps,
            'no_rendering_mode': settings.no_rendering_mode
        }

    def analyze_npc_movement(self):
        """Analyze NPC movement data for smoothness metrics"""
        results = {}
        
        for veh_id, history in self.npc_motion_history.items():
            if len(history) < 3:
                continue
                
            # Extract data series
            times = [entry['time'] for entry in history]
            positions = [entry['position'] for entry in history]
            velocities = [entry['velocity'] for entry in history]
            accelerations = [entry.get('acceleration', 0) for entry in history]
            
            # Calculate time differences
            time_diffs = [times[i] - times[i-1] for i in range(1, len(times))]
            time_variance = np.var(time_diffs) if len(time_diffs) > 1 else 0
            
            # Calculate position differences
            pos_diffs = []
            for i in range(1, len(positions)):
                prev = positions[i-1]
                curr = positions[i]
                diff = np.sqrt((curr.x - prev.x)**2 + (curr.y - prev.y)**2 + (curr.z - prev.z)**2)
                pos_diffs.append(diff)
            
            pos_diff_variance = np.var(pos_diffs) if len(pos_diffs) > 1 else 0
            
            # Calculate velocity differences
            vel_diffs = [velocities[i] - velocities[i-1] for i in range(1, len(velocities))]
            vel_diff_variance = np.var(vel_diffs) if len(vel_diffs) > 1 else 0
            
            # Calculate acceleration variance
            acc_variance = np.var(accelerations) if len(accelerations) > 1 else 0
            
            # Calculate jerk if we have it
            jerks = self.npc_jerk.get(veh_id, [])
            avg_jerk = sum(jerks) / len(jerks) if jerks else 0
            max_jerk = max(jerks) if jerks else 0
            
            # Calculate movement irregularity score
            # Lower score means more smooth movement
            smoothness_score = (pos_diff_variance + vel_diff_variance + acc_variance + avg_jerk) 
            
            results[veh_id] = {
                'time_variance': time_variance,
                'position_variance': pos_diff_variance,
                'velocity_variance': vel_diff_variance,
                'acceleration_variance': acc_variance,
                'average_jerk': avg_jerk,
                'max_jerk': max_jerk,
                'smoothness_score': smoothness_score
            }
        
        # Calculate averages across all NPCs
        if results:
            avg_time_var = sum(r['time_variance'] for r in results.values()) / len(results)
            avg_pos_var = sum(r['position_variance'] for r in results.values()) / len(results)
            avg_vel_var = sum(r['velocity_variance'] for r in results.values()) / len(results)
            avg_acc_var = sum(r['acceleration_variance'] for r in results.values()) / len(results)
            avg_jerk = sum(r['average_jerk'] for r in results.values()) / len(results)
            max_jerk = max(r['max_jerk'] for r in results.values())
            avg_smoothness = sum(r['smoothness_score'] for r in results.values()) / len(results)
            
            return {
                'npc_count': len(results),
                'avg_time_variance': avg_time_var,
                'avg_position_variance': avg_pos_var,
                'avg_velocity_variance': avg_vel_var,
                'avg_acceleration_variance': avg_acc_var,
                'avg_jerk': avg_jerk,
                'max_jerk': max_jerk,
                'avg_smoothness_score': avg_smoothness,
                'interpretation': 'Lower score = smoother movement. High variances indicate stuttering.'
            }
        else:
            return {'error': 'No NPC movement data available'}

    def calculate_fps(self, time_queue):
        """Calculate FPS from time measurements"""
        if len(time_queue) < 2:
            return 0
        
        avg_time = sum(time_queue) / len(time_queue)
        return 1.0 / avg_time if avg_time > 0 else 0

    def monitor_loop(self):
        """Background monitoring thread"""
        while self.running:
            time.sleep(1.0)  # Check every second

    def main_loop(self):
        """Main loop - print stats to terminal"""
        try:
            while self.running:
                # Calculate all performance metrics
                render_fps = self.calculate_fps(self.frame_times)
                physics_fps = self.calculate_fps(self.physics_step_times)
                
                # Get all monitoring data
                carla_settings = self.get_carla_settings()
                tm_info = self.get_traffic_manager_info()
                movement_analysis = self.analyze_npc_movement()
                
                # Clear terminal
                os.system('cls' if os.name == 'nt' else 'clear')
                
                # Print statistics
                print("=== CARLA NPC MONITOR ===")
                print(f"Render FPS: {render_fps:.2f}")
                print(f"Physics Update Rate: {physics_fps:.2f} Hz")
                
                # Print NPC smoothness metrics
                print("\nNPC Motion Smoothness Analysis:")
                if 'error' in movement_analysis:
                    print(f"  {movement_analysis['error']}")
                else:
                    print(f"  NPC Count: {movement_analysis.get('npc_count', 0)}")
                    print(f"  Average Jerk: {movement_analysis.get('avg_jerk', 0):.6f}")
                    print(f"  Maximum Jerk: {movement_analysis.get('max_jerk', 0):.6f}")
                    print(f"  Position Variance: {movement_analysis.get('avg_position_variance', 0):.6f}")
                    print(f"  Velocity Variance: {movement_analysis.get('avg_velocity_variance', 0):.6f}")
                    print(f"  Time Variance: {movement_analysis.get('avg_time_variance', 0):.6f}")
                    print(f"  Smoothness Score: {movement_analysis.get('avg_smoothness_score', 0):.6f}")
                    print(f"  Note: {movement_analysis.get('interpretation', '')}")
                
                # Print CARLA settings
                print("\nCARLA World Settings:")
                print(f"  Synchronous Mode: {carla_settings['synchronous_mode']}")
                fps_text = f"{1/carla_settings['fixed_delta_seconds']:.2f}Hz" if carla_settings['fixed_delta_seconds'] else "N/A"
                print(f"  Fixed Delta Seconds: {carla_settings['fixed_delta_seconds']}s ({fps_text})")
                print(f"  Substepping: {carla_settings['substepping']}")
                print(f"  Max Substep Delta Time: {carla_settings['max_substep_delta_time']}")
                print(f"  Max Substeps: {carla_settings['max_substeps']}")
                
                # Print Traffic Manager info
                print("\nTraffic Manager Status:")
                print(f"  Synchronous Mode: {tm_info.get('sync_mode', 'N/A')}")
                print(f"  Global Distance to Leading Vehicle: {tm_info.get('global_distance', 'N/A')}")
                print(f"  Controlled Vehicles: {tm_info.get('controlled_vehicles', 'N/A')}")
                print(f"  Hybrid Physics Mode: {tm_info.get('hybrid_physics', 'N/A')}")
                print(f"  Estimated Update Rate: {tm_info.get('estimated_update_rate', 'N/A')}")
                
                print("\nEditor Tools:")
                print("  To access detailed performance metrics in UE4 Editor:")
                print("  1. Press ` (backtick) to open console and type: stat fps")
                print("  2. Or type: stat unitgraph")
                print("  3. For traffic debugging type: show TRAFFIC")
                
                print("\nPress Ctrl+C to exit")
                
                time.sleep(1)  # Update statistics every second
                
        except KeyboardInterrupt:
            print("Monitoring interrupted by user")
        finally:
            self.running = False
            # Remove callback to prevent memory leaks
            self.world.remove_on_tick(self.callback_id)
            if self.monitor_thread.is_alive():
                self.monitor_thread.join()
            print("Monitoring stopped")

if __name__ == '__main__':
    try:
        host = 'localhost'
        port = 2000
        
        # Check command line arguments
        if len(sys.argv) > 1:
            host = sys.argv[1]
        if len(sys.argv) > 2:
            port = int(sys.argv[2])
            
        monitor = CarlaMonitor(host, port)
        print(f"Connecting to CARLA server: {host}:{port}")
        monitor.start()
        
    except Exception as e:
        print(f"Error occurred: {e}")
        sys.exit(1)