#!/usr/bin/env python3
"""
ROS2环境设置脚本
帮助用户正确设置ROS2环境并启动小车控制
"""

import os
import sys
import subprocess
import time
from pathlib import Path

def detect_ros2_installations():
    """
    检测系统中的ROS2安装
    """
    print("检测ROS2安装...")
    
    # 检查环境变量
    ros_distro = os.environ.get('ROS_DISTRO', None)
    if ros_distro:
        print(f"已加载的ROS2版本: {ros_distro}")
        ros_prefix = os.environ.get('ROS_PREFIX', None)
        if ros_prefix:
            print(f"ROS2路径: {ros_prefix}")
        return True
    
    # 检查常见安装路径
    ros_paths = {
        '/opt/ros/humble': 'humble',
        '/opt/ros/foxy': 'foxy',
        '/opt/ros/galactic': 'galactic',
        '/opt/ros/eloquent': 'eloquent',
        '/opt/ros/dashing': 'dashing',
        '/opt/ros/crystal': 'crystal',
        '/opt/ros/bouncy': 'bouncy',
        '/opt/ros/ardent': 'ardent'
    }
    
    found_installations = []
    for path, version in ros_paths.items():
        if os.path.exists(path) and os.path.exists(os.path.join(path, 'setup.bash')):
            found_installations.append((path, version))
    
    if found_installations:
        print("发现以下ROS2安装:")
        for path, version in found_installations:
            print(f"  {version}: {path}")
        return found_installations
    else:
        print("未找到ROS2安装")
        return []

def setup_ros2_environment(installation_path):
    """
    设置ROS2环境
    """
    setup_script = os.path.join(installation_path, 'setup.bash')
    if not os.path.exists(setup_script):
        print(f"错误：找不到设置脚本 {setup_script}")
        return False
    
    print(f"设置ROS2环境: {setup_script}")
    
    # 注意：这里只是生成shell命令，实际需要在shell中执行
    shell_commands = [
        f"source {setup_script}",
        "echo 'ROS2环境已加载'",
        "ros2 --version"
    ]
    
    # 创建一个shell脚本
    script_path = "/tmp/setup_ros2.sh"
    with open(script_path, 'w') as f:
        f.write("#!/bin/bash\n")
        for cmd in shell_commands:
            f.write(f"{cmd}\n")
    
    # 设置执行权限
    os.chmod(script_path, 0o755)
    
    print(f"已创建环境设置脚本: {script_path}")
    print("请在shell中执行以下命令设置ROS2环境:")
    print(f"bash {script_path}")
    print("或者:")
    print(f"source {setup_script}")
    
    return True

def check_racecar_launch_files():
    """
    检查racecar启动文件
    """
    print("\n检查racecar启动文件...")
    
    # 检查当前工作目录
    current_dir = os.getcwd()
    print(f"当前目录: {current_dir}")
    
    # 查找racecar包
    racecar_paths = [
        os.path.join(current_dir, 'src', 'racecar'),
        os.path.join(current_dir, 'install', 'racecar'),
        '/opt/ros/humble/share/racecar',
        '/opt/ros/foxy/share/racecar'
    ]
    
    launch_files = []
    for path in racecar_paths:
        launch_dir = os.path.join(path, 'launch')
        if os.path.exists(launch_dir):
            print(f"发现racecar启动目录: {launch_dir}")
            # 查找所有.launch.py文件
            for file in os.listdir(launch_dir):
                if file.endswith('.launch.py'):
                    launch_files.append(os.path.join(launch_dir, file))
    
    if launch_files:
        print("发现以下启动文件:")
        for file in launch_files:
            print(f"  {os.path.basename(file)}")
        
        # 检查Run_car.launch.py
        run_car_file = None
        for file in launch_files:
            if 'Run_car' in file or 'run_car' in file:
                run_car_file = file
                break
        
        if run_car_file:
            print(f"\n主启动文件: {os.path.basename(run_car_file)}")
            return run_car_file
        else:
            print("\n未找到Run_car.launch.py，使用第一个启动文件")
            return launch_files[0]
    else:
        print("未找到racecar启动文件")
        return None

def create_startup_script(launch_file):
    """
    创建启动脚本
    """
    script_path = "/tmp/start_racecar.sh"
    
    # 检测ROS2安装
    installations = detect_ros2_installations()
    
    if isinstance(installations, list) and installations:
        # 使用第一个找到的安装
        installation_path, version = installations[0]
        setup_script = os.path.join(installation_path, 'setup.bash')
    else:
        # 如果环境已加载，尝试获取路径
        ros_prefix = os.environ.get('ROS_PREFIX', None)
        if ros_prefix:
            setup_script = os.path.join(ros_prefix, 'setup.bash')
        else:
            setup_script = "/opt/ros/humble/setup.bash"  # 默认
    
    # 创建启动脚本
    with open(script_path, 'w') as f:
        f.write("#!/bin/bash\n\n")
        f.write("# ROS2环境设置脚本\n")
        f.write(f"source {setup_script}\n\n")
        f.write("echo 'ROS2环境已加载'\n")
        f.write("ros2 --version\n\n")
        f.write("# 等待环境稳定\n")
        f.write("sleep 2\n\n")
        f.write("# 启动racecar驱动\n")
        f.write(f"echo '启动racecar驱动: {launch_file}'\n")
        f.write(f"ros2 launch {launch_file}\n")
    
    # 设置执行权限
    os.chmod(script_path, 0o755)
    
    print(f"\n已创建启动脚本: {script_path}")
    print("请使用以下命令启动racecar:")
    print(f"bash {script_path}")
    
    return script_path

def create_traffic_light_launcher():
    """
    创建红绿灯识别启动脚本
    """
    script_path = "/tmp/start_traffic_light.sh"
    
    # 检测ROS2安装
    installations = detect_ros2_installations()
    
    if isinstance(installations, list) and installations:
        # 使用第一个找到的安装
        installation_path, version = installations[0]
        setup_script = os.path.join(installation_path, 'setup.bash')
    else:
        # 如果环境已加载，尝试获取路径
        ros_prefix = os.environ.get('ROS_PREFIX', None)
        if ros_prefix:
            setup_script = os.path.join(ros_prefix, 'setup.bash')
        else:
            setup_script = "/opt/ros/humble/setup.bash"  # 默认
    
    # 创建启动脚本
    with open(script_path, 'w') as f:
        f.write("#!/bin/bash\n\n")
        f.write("# ROS2环境设置脚本\n")
        f.write(f"source {setup_script}\n\n")
        f.write("echo 'ROS2环境已加载'\n")
        f.write("ros2 --version\n\n")
        f.write("# 启动红绿灯识别与控制\n")
        f.write("python3 traffic_light_control_comprehensive.py\n")
    
    # 设置执行权限
    os.chmod(script_path, 0o755)
    
    print(f"\n已创建红绿灯识别启动脚本: {script_path}")
    print("请使用以下命令启动红绿灯识别:")
    print(f"bash {script_path}")
    
    return script_path

def main():
    """
    主函数
    """
    print("ROS2环境设置与racecar启动助手")
    print("=" * 50)
    
    # 检测ROS2安装
    installations = detect_ros2_installations()
    
    if not installations:
        print("\n错误：未找到ROS2安装")
        print("请先安装ROS2：https://docs.ros.org/en/rolling/Installation.html")
        return
    
    # 检查racecar启动文件
    launch_file = check_racecar_launch_files()
    
    if not launch_file:
        print("\n错误：未找到racecar启动文件")
        print("请确保已正确安装racecar包")
        return
    
    # 创建启动脚本
    startup_script = create_startup_script(launch_file)
    traffic_light_script = create_traffic_light_launcher()
    
    # 提供下一步指导
    print("\n下一步操作:")
    print("1. 启动racecar驱动（新终端窗口）:")
    print(f"   bash {startup_script}")
    print("\n2. 启动红绿灯识别与控制（新终端窗口）:")
    print(f"   bash {traffic_light_script}")
    
    # 如果环境未设置，提供额外指导
    if not os.environ.get('ROS_DISTRO'):
        print("\n注意：当前ROS2环境未加载")
        print("请在新终端中运行上述脚本，它们会自动设置环境")
    
    # 提供手动控制指导
    print("\n手动控制（可选）:")
    print("1. 设置环境: source /opt/ros/your_distro/setup.bash")
    print("2. 启动驱动: ros2 launch racecar Run_car.launch.py")
    print("3. 运行测试: python3 simple_motor_test.py")
    print("4. 运行识别: python3 traffic_light_control_comprehensive.py")

if __name__ == "__main__":
    main()