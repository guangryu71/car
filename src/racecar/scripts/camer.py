import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time
import threading

class CarController(Node):
    """
    小车控制器类，负责控制小车的速度和方向
    """
    def __init__(self):
        super().__init__('car_controller')
        # 创建发布者，用于控制小车速度
        self.cmd_vel_pub = self.create_publisher(Twist, '/car_cmd_vel', 10)
        
        # 小车状态
        self.current_speed = 1.0  # m/s，初始速度
        self.is_stopping = False  # 是否正在减速停止
        self.stop_start_time = None  # 开始减速的时间
        self.stop_duration = 2.0  # 从开始减速到完全停止的时间，保证在2米内停止
        self.initial_speed = 1.0  # 初始速度
        
        # 启动速度控制线程
        self.velocity_thread = threading.Thread(target=self.velocity_control_loop)
        self.velocity_thread.daemon = True
        self.velocity_thread.start()
        
    def velocity_control_loop(self):
        """
        速度控制循环，定期发布速度命令
        """
        rate = self.create_rate(10)  # 10Hz
        
        while rclpy.ok():
            twist = Twist()
            
            if self.is_stopping:
                # 计算减速后的速度
                if self.stop_start_time is not None:
                    elapsed_time = time.time() - self.stop_start_time
                    
                    if elapsed_time >= self.stop_duration:
                        # 已经停止
                        twist.linear.x = 0.0
                    else:
                        # 线性减速
                        speed_ratio = 1.0 - (elapsed_time / self.stop_duration)
                        twist.linear.x = self.initial_speed * speed_ratio
                else:
                    twist.linear.x = 0.0
            else:
                # 保持匀速
                twist.linear.x = self.current_speed
            
            # 保持直线行驶，不转向
            twist.angular.z = 0.0
            
            # 发布速度命令
            self.cmd_vel_pub.publish(twist)
            
            rate.sleep()
    
    def start_stopping(self):
        """
        开始减速停车
        """
        if not self.is_stopping:
            self.is_stopping = True
            self.stop_start_time = time.time()
            print("开始减速停车，将在2米内停止")
    
    def resume(self, speed=1.0):
        """
        恢复匀速行驶
        """
        self.is_stopping = False
        self.stop_start_time = None
        self.current_speed = speed
        self.initial_speed = speed
        print(f"恢复行驶，速度为 {speed} m/s")

def detect_traffic_light_color(frame):
    """
    检测图像中的红绿灯颜色
    返回: (颜色名称, 处理后的图像)
    """
    # 转换为HSV颜色空间，更容易检测颜色
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    
    # 定义红色的HSV范围
    # 红色在HSV中有两个范围
    lower_red1 = np.array([0, 100, 100])
    upper_red1 = np.array([10, 255, 255])
    lower_red2 = np.array([160, 100, 100])
    upper_red2 = np.array([180, 255, 255])
    
    # 定义绿色的HSV范围
    lower_green = np.array([40, 50, 50])
    upper_green = np.array([90, 255, 255])
    
    # 创建颜色掩码
    mask_red1 = cv2.inRange(hsv, lower_red1, upper_red1)
    mask_red2 = cv2.inRange(hsv, lower_red2, upper_red2)
    mask_red = cv2.bitwise_or(mask_red1, mask_red2)
    mask_green = cv2.inRange(hsv, lower_green, upper_green)
    
    # 计算每种颜色的像素数量
    red_pixels = cv2.countNonZero(mask_red)
    green_pixels = cv2.countNonZero(mask_green)
    
    # 判断主要颜色
    detected_color = "未检测到"
    result_frame = frame.copy()
    
    if red_pixels > 500 and red_pixels > green_pixels:
        detected_color = "红灯"
        # 在图像上标记红色区域
        contours, _ = cv2.findContours(mask_red, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if contours:
            largest_contour = max(contours, key=cv2.contourArea)
            if cv2.contourArea(largest_contour) > 100:
                x, y, w, h = cv2.boundingRect(largest_contour)
                cv2.rectangle(result_frame, (x, y), (x+w, y+h), (0, 0, 255), 2)
                cv2.putText(result_frame, "Red Light", (x, y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
    elif green_pixels > 500 and green_pixels > red_pixels:
        detected_color = "绿灯"
        # 在图像上标记绿色区域
        contours, _ = cv2.findContours(mask_green, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if contours:
            largest_contour = max(contours, key=cv2.contourArea)
            if cv2.contourArea(largest_contour) > 100:
                x, y, w, h = cv2.boundingRect(largest_contour)
                cv2.rectangle(result_frame, (x, y), (x+w, y+h), (0, 255, 0), 2)
                cv2.putText(result_frame, "Green Light", (x, y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
    
    # 在图像上显示检测结果
    cv2.putText(result_frame, f"检测: {detected_color}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
    
    return detected_color, result_frame

def open_camera():
    """打开摄像头并进行红绿灯识别，同时控制小车"""
    import subprocess
    import os
    
    # 初始化ROS节点
    rclpy.init()
    
    # 创建小车控制器
    car_controller = CarController()
    
    # 首先检查可用的视频设备
    print("检查可用的视频设备...")
    try:
        result = subprocess.run(['ls', '-la', '/dev/video*'], 
                              capture_output=True, text=True)
        if result.returncode == 0:
            print("发现的视频设备:")
            print(result.stdout)
        else:
            print("未发现任何视频设备")
    except Exception as e:
        print(f"检查视频设备时出错: {e}")
    
    # 检查设备权限
    devices_to_check = [0, 1, 2]  # 扩展到video2
    available_devices = []
    
    for device_idx in devices_to_check:
        device_path = f"/dev/video{device_idx}"
        if os.path.exists(device_path):
            # 检查权限
            try:
                if os.access(device_path, os.R_OK):
                    print(f"✓ 设备 {device_path} 可读")
                    available_devices.append(device_idx)
                else:
                    print(f"✗ 设备 {device_path} 权限不足")
                    # 尝试获取权限信息
                    stat_info = os.stat(device_path)
                    print(f"  设备权限: {oct(stat_info.st_mode)[-3:]}")
                    print(f"  所有者: UID={stat_info.st_uid}, GID={stat_info.st_gid}")
            except Exception as e:
                print(f"✗ 检查设备 {device_path} 时出错: {e}")
        else:
            print(f"✗ 设备 {device_path} 不存在")
    
    if not available_devices:
        print("=== 错误解决方案 ===")
        print("1. 检查摄像头是否正确连接:")
        print("   lsusb | grep -i camera")
        print("2. 检查内核是否识别摄像头:")
        print("   dmesg | grep -i video")
        print("3. 添加用户到video组:")
        print("   sudo usermod -a -G video $USER")
        print("   然后重新登录或重启")
        print("4. 临时修改设备权限:")
        print("   sudo chmod 666 /dev/video*")
        print("5. 检查是否有其他程序占用摄像头:")
        print("   sudo lsof /dev/video*")
        print("6. 尝试重新加载摄像头驱动:")
        print("   sudo rmmod uvcvideo")
        print("   sudo modprobe uvcvideo")
        
        print("错误: 没有可用的摄像头设备")
        rclpy.shutdown()
        return
    
    # 尝试不同的设备和后端
    backends = [cv2.CAP_V4L2, cv2.CAP_GSTREAMER, cv2.CAP_ANY]
    
    cap = None
    successful_device = None
    successful_backend = None
    
    for device_idx in available_devices:
        for backend in backends:
            backend_name = {
                cv2.CAP_V4L2: "V4L2",
                cv2.CAP_GSTREAMER: "GStreamer", 
                cv2.CAP_ANY: "ANY"
            }.get(backend, str(backend))
            
            print(f"尝试打开设备 /dev/video{device_idx}，后端 {backend_name}")
            cap = cv2.VideoCapture(device_idx, backend)
            
            if cap.isOpened():
                # 测试是否能实际读取帧
                ret, test_frame = cap.read()
                if ret and test_frame is not None:
                    print(f"✓ 成功打开设备: /dev/video{device_idx} (后端: {backend_name})")
                    print(f"  分辨率: {test_frame.shape[1]}x{test_frame.shape[0]}")
                    successful_device = device_idx
                    successful_backend = backend_name
                    # 设置参数
                    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
                    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
                    cap.set(cv2.CAP_PROP_FPS, 30)  # 降低帧率
                    break
                else:
                    print(f"✗ 设备 /dev/video{device_idx} 打开但无法读取帧")
                    cap.release()
            else:
                if cap:
                    cap.release()
                    cap = None
        
        if cap and cap.isOpened():
            break
    
    if not cap or not cap.isOpened():
        print("错误: 所有设备和后端都无法打开摄像头")
        rclpy.shutdown()
        return
    
    print("摄像头已打开，红绿灯识别启动，小车以1m/s匀速行驶")
    print("按 'q' 键退出")
    print("小车将自动识别红绿灯，检测到红/绿灯后减速停车")
    
    # 小车初始状态
    last_detected_color = "未检测到"
    is_stopped = False  # 是否已经停止
    
    # 启动ROS节点线程
    ros_thread = threading.Thread(target=rclpy.spin, args=(car_controller,))
    ros_thread.daemon = True
    ros_thread.start()
    
    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                print("警告: 无法读取摄像头帧")
                continue
            
            # 检测红绿灯颜色
            detected_color, result_frame = detect_traffic_light_color(frame)
            
            # 在图像上显示小车状态
            status = "匀速行驶" if not car_controller.is_stopping else "减速停车"
            cv2.putText(result_frame, f"状态: {status}", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            
            # 处理红绿灯检测结果
            if detected_color in ["红灯", "绿灯"] and not is_stopped:
                # 检测到红/绿灯，开始减速停车
                car_controller.start_stopping()
                is_stopped = True
                print(f"识别到{detected_color}，开始减速停车")
            
            # 只在检测结果变化时输出到控制台
            if detected_color != last_detected_color:
                print(f"识别到: {detected_color}")
                last_detected_color = detected_color
            
            # 显示画面
            cv2.imshow('Traffic Light Detection', result_frame)
            
            key = cv2.waitKey(30) & 0xFF
            if key == ord('q'):
                break
            
            # 处理ROS回调
            rclpy.spin_once(car_controller, timeout_sec=0.001)
            
    except KeyboardInterrupt:
        pass
    finally:
        # 清理资源
        car_controller.destroy_node()
        rclpy.shutdown()
        cap.release()
        cv2.destroyAllWindows()
        print("程序已退出")

# 使用示例
if __name__ == "__main__":
    open_camera()

