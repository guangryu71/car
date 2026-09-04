#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Direct control script
Uses environment variables and direct commands
"""

import os
import sys
import time
import signal
import subprocess
import threading

class DirectController:
    """Direct controller with environment setup"""
    
    def __init__(self):
        # Control parameters
        self.target_speed = 1.0
        self.target_angle = 0.0
        self.is_running = False
        self.env = None
        
        # Set up signal handler
        signal.signal(signal.SIGINT, self.signal_handler)
        
        # Initialize environment
        self.setup_environment()
    
    def setup_environment(self):
        """Setup ROS2 environment"""
        print("Setting up ROS2 environment...")
        
        # Get current environment
        self.env = os.environ.copy()
        
        # Try to find and source ROS2 setup
        ros2_setup = None
        for distro in ['humble', 'foxy', 'galactic']:
            setup_file = f"/opt/ros/{distro}/setup.bash"
            if os.path.exists(setup_file):
                ros2_setup = setup_file
                print(f"Found ROS2 {distro} at {setup_file}")
                break
        
        if not ros2_setup:
            print("Error: ROS2 not found")
            sys.exit(1)
        
        # Source ROS2 setup and update environment
        try:
            # Use bash to source setup and export environment
            cmd = f"source {ros2_setup} && env"
            result = subprocess.run(cmd, shell=True, executable="/bin/bash", 
                                  capture_output=True, text=True)
            
            if result.returncode == 0:
                # Parse environment variables
                for line in result.stdout.split('\n'):
                    if '=' in line:
                        key, value = line.split('=', 1)
                        self.env[key] = value
                print("ROS2 environment loaded successfully")
            else:
                print(f"Error loading ROS2 environment: {result.stderr}")
        except Exception as e:
            print(f"Error setting up environment: {e}")
        
        # Try to find workspace setup
        workspace_dirs = [
            "/home/davinci-mini/racecar/install",
            "/home/davinci-mini/racecar/install/local_setup.bash",
            os.path.expanduser("~/racecar/install"),
            os.path.expanduser("~/racecar/install/setup.bash")
        ]
        
        for workspace_path in workspace_dirs:
            if os.path.exists(workspace_path):
                print(f"Found workspace at {workspace_path}")
                try:
                    cmd = f"source {workspace_path}/setup.bash && env"
                    result = subprocess.run(cmd, shell=True, executable="/bin/bash", 
                                          capture_output=True, text=True)
                    
                    if result.returncode == 0:
                        for line in result.stdout.split('\n'):
                            if '=' in line:
                                key, value = line.split('=', 1)
                                self.env[key] = value
                        print("Workspace environment loaded successfully")
                        break
                except Exception as e:
                    print(f"Error loading workspace environment: {e}")
        else:
            print("Warning: Could not find workspace setup")
    
    def publish_command(self, speed=None, angle=None):
        """Publish a single control command"""
        if speed is None:
            speed = self.target_speed
        if angle is None:
            angle = self.target_angle
        
        # Use the simplest possible command
        cmd = [
            "ros2", "topic", "pub", "-1", 
            "/car_cmd_vel", "geometry_msgs/msg/Twist", 
            f"{{linear: {{x: {speed:.2f}}}, angular: {{z: {angle:.4f}}}}}"
        ]
        
        try:
            result = subprocess.run(
                cmd, 
                env=self.env,
                capture_output=True, 
                text=True, 
                timeout=2
            )
            
            if result.returncode == 0:
                print(f"? Command sent: speed={speed:.2f}, angle={angle:.4f}")
                return True
            else:
                print(f"? Command failed: {result.stderr}")
                return False
        except subprocess.TimeoutExpired:
            print("? Command timed out")
            return False
        except Exception as e:
            print(f"? Error: {e}")
            return False
    
    def control_loop(self):
        """Main control loop"""
        print("Control loop started")
        consecutive_failures = 0
        max_failures = 5
        
        while self.is_running:
            success = self.publish_command()
            
            if success:
                consecutive_failures = 0
            else:
                consecutive_failures += 1
                if consecutive_failures >= max_failures:
                    print("Too many consecutive failures, stopping")
                    self.is_running = False
                    break
            
            time.sleep(0.1)  # Publish at 10Hz
    
    def start(self):
        """Start the controller"""
        print("=" * 50)
        print("Direct Car Control Script")
        print("=" * 50)
        
        # Check if racecar_driver is running
        print("Checking if racecar_driver is running...")
        cmd = ["ros2", "node", "list"]
        result = subprocess.run(cmd, env=self.env, capture_output=True, text=True)
        
        if "racecar_driver" in result.stdout:
            print("? racecar_driver is running")
        else:
            print("? racecar_driver is not running")
            print("Please start it with:")
            print("  source /opt/ros/humble/setup.bash")
            print("  source /path/to/your/workspace/setup.bash")
            print("  ros2 run racecar_driver racecar_driver_node")
            print("")
        
        # Test with a stop command first
        print("\nTesting with stop command...")
        self.publish_command(0.0, 0.0)
        
        # Start the control loop
        print("\nStarting control loop...")
        self.is_running = True
        try:
            self.control_loop()
        except KeyboardInterrupt:
            pass
    
    def stop(self):
        """Stop the car"""
        print("\nStopping the car...")
        self.is_running = False
        
        # Send stop command
        print("Sending stop command...")
        self.publish_command(0.0, 0.0)
        print("Car stopped")
    
    def signal_handler(self, sig, frame):
        """Handle Ctrl+C"""
        self.stop()
        sys.exit(0)

def main():
    """Main function"""
    controller = DirectController()
    
    try:
        controller.start()
    except Exception as e:
        print(f"Error: {e}")
        controller.stop()

if __name__ == '__main__':
    main()