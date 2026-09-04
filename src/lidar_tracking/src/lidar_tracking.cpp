/**
 * @file lidar_tracking.cpp
 * @brief 激光雷达循迹ROS2节点主程序
 * @details 实现基于激光雷达的锥桶检测和自动循迹功能
 * @author zyh
 * @version 1.0
 * @date 2025-08-24
 */

 #include "rclcpp/rclcpp.hpp"                    // ROS2 C++客户端库
 #include "sensor_msgs/msg/laser_scan.hpp"       // 激光雷达扫描消息类型
 #include "geometry_msgs/msg/twist.hpp"          // 速度控制消息类型
 #include <math.h>                               // 数学函数库
 #include "control.hpp"            //pid控制器
 #include <string>                               // 字符串处理库
 #include <memory>                               // 智能指针库
 #include <chrono>                               // 时间相关库
 
 // ===== 全局常量定义 =====
 #define freq 1440                               // 激光雷达扫描频率：每秒1440次扫描
 #define pi_2 1.57                               // 90度角度的弧度值（π/2）
 
 // ===== 全局变量定义 =====
 int speed = 1535;                               // 全局速度变量：基础行驶速度
 double rate_p[10] = { 0.8, 2.2, 0.8, 0, 0, 0, 0, 0, 0, 0 };  // 路径规划权重数组
 double max_right_dis = 1.5;                     // 最大右侧检测距离
 double max_left_dis = 1.5;                      // 最大左侧检测距离
 
 // ===== 数据结构定义 =====
 /**
  * @brief 极坐标点结构体
  */
 typedef struct 
 {   
     double range;                               // 距离：激光雷达测量的距离值
     double theta;                               // 角度：激光雷达测量的角度值
 } Point_polar;
 
 /**
  * @brief 直角坐标点结构体
  */
 typedef struct
 {
     double x;                                   // X坐标：横向位置（左右）
     double y;                                   // Y坐标：纵向位置（前后）
 } Point_rectangular;
 
 // ===== 全局对象实例 =====
 Control::PID pid;                               // PID控制器实例
 
 /**
  * @brief 极坐标转直角坐标函数
  * @param theta 极坐标角度（弧度）
  * @param range 极坐标距离
  * @param tmp_x 输出X坐标的引用
  * @param tmp_y 输出Y坐标的引用
  */
 void cal_point(double theta, double range, double &tmp_x, double & tmp_y)
 {
      tmp_x = range * sin(theta);                // 计算X坐标：距离 * sin(角度)
      tmp_y = range * cos(theta);                // 计算Y坐标：距离 * cos(角度)
 }
 
 /**
  * @brief 激光雷达循迹节点类
  * @details 继承自rclcpp::Node，实现激光雷达数据处理和车辆控制
  */
 class PubAndSub : public rclcpp::Node
 {
 private:
     // ===== 直角转弯相关变量 =====
     bool is_right_angle_mode = false;              // 当前是否处于直角转弯状态
     double right_angle_start_time;                 // 进入直角转弯状态的时间戳
     int right_angle_turn_count = 0;                // 直角转弯计数器（防止误触发）
     bool right_angle_detected = false;             // 是否检测到直角转弯条件
     
     // ===== 直角转弯参数 =====
     double right_angle_detection_range;            // 直角转弯检测范围
     double right_angle_min_gap;                    // 最小间隙阈值（检测直角）
     double right_angle_slow_speed;                 // 直角转弯减速速度
     double right_angle_turn_speed;                 // 直角转弯转向速度
     int right_angle_confirm_count;                 // 确认直角转弯的连续帧数
     
     // ===== 直角转弯处理函数声明 =====
     /**
      * @brief 检测直角转弯条件 - 使用精确的锥桶检测算法
      * @details 通过分析前方锥桶分布模式检测直角转弯
      *          使用与正常循迹相同的锥桶检测算法，确保一致性
      * @param laser 激光雷达数据
      * @return 是否检测到直角转弯
      */
     bool detectRightAngleTurn(const sensor_msgs::msg::LaserScan::SharedPtr& laser);
     
     /**
      * @brief 处理直角转弯状态 - 基于检测到的锥桶分布
      * @details 根据检测到的直角转弯模式选择转向方向
      * @param laser 激光雷达数据
      */
     void handleRightAngleMode(const sensor_msgs::msg::LaserScan::SharedPtr& laser);
     
     /**
      * @brief 更新直角转弯状态 - 增强的状态管理
      * @param laser 激光雷达数据
      */
     void updateRightAngleMode(const sensor_msgs::msg::LaserScan::SharedPtr& laser);
     
     // ===== ROS2通信组件 =====
     rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;      // 速度控制发布器
     rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_; // 激光雷达数据订阅器
     
     // ===== 定时器组件 =====
     rclcpp::TimerBase::SharedPtr status_timer_;                       // 状态打印定时器
     
     // ===== 控制相关变量 =====
     geometry_msgs::msg::Twist twist;            // 速度控制消息对象
     std::string topic;                          // 发布话题名称
     
     // ===== 状态管理变量 =====
     bool is_no_cone_mode = false;               // 当前是否处于无锥桶巡航状态
     double no_cone_start_time;                  // 进入无锥桶状态的时间戳
     
     // ===== 扇形检测参数 =====
     double no_cone_detection_range;             // 无锥桶检测范围（米）
     double no_cone_sector_angle;                // 无锥桶检测扇形角度（弧度）
     
     // ===== 无锥桶巡航状态参数 =====
     double no_cone_speed;                       // 无锥桶状态下的行驶速度
     double no_cone_turn_angle;                  // 无锥桶状态下的固定转向角度
     
     // ===== 正常循迹状态参数 =====
     double max_right_dis;                       // 正常循迹状态下的最大右侧检测距离
     double max_left_dis;                        // 正常循迹状态下的最大左侧检测距离
     
 public:
     /**
      * @brief 构造函数：初始化节点和参数
      */
     PubAndSub() : Node("lidar_tracking_node")
     {
         std::string ttopic = "/teleop_cmd_vel"; // 默认话题名称
         
         // ===== 声明ROS2参数 =====
         // 基础控制参数
         this->declare_parameter("speed", 1535);                     // 基础行驶速度
         this->declare_parameter("kp", 0.88);                       // PID比例系数
         this->declare_parameter("ki", 0.0);                        // PID积分系数
         this->declare_parameter("kd", 1.2);                        // PID微分系数
         this->declare_parameter("topic", ttopic);                  // 发布话题名称
         
         // 路径规划权重参数
         this->declare_parameter("rate1", 0.8);                     // 第一对锥桶权重
         this->declare_parameter("rate2", 2.2);                     // 第二对锥桶权重
         this->declare_parameter("rate3", 0.8);                     // 第三对锥桶权重
         
         // 扇形检测参数
         this->declare_parameter("no_cone_detection_range", 1.0);    // 无锥桶检测范围
         this->declare_parameter("no_cone_sector_angle", 60.0);      // 无锥桶检测扇形角度（度）
         
         // 无锥桶巡航状态参数
         this->declare_parameter("no_cone_speed", 1525);            // 无锥桶状态下的速度
         this->declare_parameter("no_cone_turn_angle", 120.0);      // 无锥桶状态下的转向角度
         
         // 正常循迹状态参数
         this->declare_parameter("max_right_dis", 1.5);             // 正常循迹状态下的最大右侧检测距离
         this->declare_parameter("max_left_dis", 1.5);              // 正常循迹状态下的最大左侧检测距离
         
         // ===== 直角转弯参数声明 =====
         this->declare_parameter("right_angle_detection_range", 2.5);      // 直角转弯检测范围
         this->declare_parameter("right_angle_min_gap", 0.4);              // 最小间隙阈值
         this->declare_parameter("right_angle_slow_speed", 1510);          // 直角转弯减速速度
         this->declare_parameter("right_angle_turn_speed", 90.0);          // 直角转弯转向角度
         this->declare_parameter("right_angle_confirm_count", 2);          // 确认帧数
         
         // ===== 获取参数值 =====
         speed = this->get_parameter("speed").as_int();             // 获取基础速度参数
         pid.kp = this->get_parameter("kp").as_double();            // 获取PID比例系数
         pid.ki = this->get_parameter("ki").as_double();            // 获取PID积分系数
         pid.kd = this->get_parameter("kd").as_double();            // 获取PID微分系数
         topic = this->get_parameter("topic").as_string();          // 获取发布话题名称
         rate_p[0] = this->get_parameter("rate1").as_double();      // 获取第一对锥桶权重
         rate_p[1] = this->get_parameter("rate2").as_double();      // 获取第二对锥桶权重
         rate_p[2] = this->get_parameter("rate3").as_double();      // 获取第三对锥桶权重
         
         // 获取扇形检测参数（角度单位转弧度）
         no_cone_detection_range = this->get_parameter("no_cone_detection_range").as_double();
         no_cone_sector_angle = this->get_parameter("no_cone_sector_angle").as_double() * M_PI / 180.0;
         
         // 获取无锥桶巡航状态参数
         no_cone_speed = this->get_parameter("no_cone_speed").as_int();
         no_cone_turn_angle = this->get_parameter("no_cone_turn_angle").as_double();
         
         // 获取正常循迹状态参数
         max_right_dis = this->get_parameter("max_right_dis").as_double();
         max_left_dis = this->get_parameter("max_left_dis").as_double();
         
         // ===== 获取直角转弯参数值 =====
         right_angle_detection_range = this->get_parameter("right_angle_detection_range").as_double();
         right_angle_min_gap = this->get_parameter("right_angle_min_gap").as_double();
         right_angle_slow_speed = this->get_parameter("right_angle_slow_speed").as_int();
         right_angle_turn_speed = this->get_parameter("right_angle_turn_speed").as_double();
         right_angle_confirm_count = this->get_parameter("right_angle_confirm_count").as_int();
         
         // ===== 创建ROS2通信组件 =====
         pub_ = this->create_publisher<geometry_msgs::msg::Twist>(topic, 5);      // 创建速度控制发布器
         sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(           // 创建激光雷达数据订阅器
             "/scan", 5, std::bind(&PubAndSub::callback, this, std::placeholders::_1));
         
         // ===== 创建状态打印定时器 =====
         status_timer_ = this->create_wall_timer(
             std::chrono::milliseconds(500),  // 500毫秒 = 0.5秒
             std::bind(&PubAndSub::printStatus, this)
         );
     }
     
     // ===== 核心功能函数声明 =====
     /**
      * @brief 扇形检测：检查指定扇形区域内是否有物体
      * @param laser 激光雷达数据
      * @return 是否应该进入无锥桶巡航状态
      */
     bool shouldEnterNoConeMode(const sensor_msgs::msg::LaserScan::SharedPtr& laser);
     
     /**
      * @brief 更新行驶模式状态
      * @param laser 激光雷达数据
      */
     void updateDrivingMode(const sensor_msgs::msg::LaserScan::SharedPtr& laser);
     
     /**
      * @brief 处理无锥桶巡航状态
      */
     void handleNoConeMode();
     
     /**
      * @brief 处理正常循迹状态
      * @param angle PID控制器输出的转向角度
      */
     void handleNormalMode(double angle);
     
     /**
      * @brief 处理正常循迹状态 - 完整的锥桶检测和路径规划
      * @param laser 激光雷达数据
      */
     void handleNormalTrackingMode(const sensor_msgs::msg::LaserScan::SharedPtr& laser);
     
     /**
      * @brief 定时打印当前状态信息
      */
     void printStatus();
     
     /**
      * @brief 激光雷达数据回调函数
      * @param laser 激光雷达扫描数据指针
      */
     void callback(const sensor_msgs::msg::LaserScan::SharedPtr laser);

     // ===== 新增：分阶段封装函数 =====
     /**
      * @brief 锥桶检测：从LaserScan中提取左右锥桶坐标点
      * @param laser 激光雷达数据
      * @param red_p 输出数组：右侧（红）锥桶坐标集合
      * @param blue_p 输出数组：左侧（蓝）锥桶坐标集合
      * @param j 输出计数：红锥桶数量
      * @param k 输出计数：蓝锥桶数量
      */
     void detectCones(const sensor_msgs::msg::LaserScan::SharedPtr& laser,
                      Point_rectangular red_p[], Point_rectangular blue_p[],
                      int &j, int &k);

     /**
      * @brief 数据关联：处理左右锥桶序列中的断裂并补齐对应序列
      * @param red_p 输入/输出：红锥桶数组
      * @param blue_p 输入/输出：蓝锥桶数组
      * @param j 输入/输出：红锥桶数量
      * @param k 输入/输出：蓝锥桶数量
      */
     void associateCones(Point_rectangular red_p[], Point_rectangular blue_p[],
                         int &j, int &k);

     /**
      * @brief 标准化：裁剪过多的点并将红蓝数量补齐到一致（≤3对）
      * @param red_p 输入/输出：红锥桶数组
      * @param blue_p 输入/输出：蓝锥桶数组
      * @param j 输入/输出：红锥桶数量
      * @param k 输入/输出：蓝锥桶数量
      */
     void standardizeCones(Point_rectangular red_p[], Point_rectangular blue_p[],
                           int &j, int &k);
 };

 // ===== 核心功能函数实现 =====

 /**
  * @brief 扇形检测：检查指定扇形区域内是否有物体
  * @param laser 激光雷达数据
  * @return 是否应该进入无锥桶巡航状态
  */
 bool PubAndSub::shouldEnterNoConeMode(const sensor_msgs::msg::LaserScan::SharedPtr& laser)
 {
     // 遍历激光雷达数据，检查±no_cone_sector_angle范围内、距离≤no_cone_detection_range是否存在障碍
     for (int i = 0; i < laser->ranges.size(); i++) {
         double theta = i * laser->angle_increment + laser->angle_min;
         double range = laser->ranges[i];
         
         // 跳过无效数据
         if (range <= 0 || std::isnan(range)) {
             continue;
         }
         
         // 扇形区域检测：角度和距离都在范围内
         if (abs(theta) <= no_cone_sector_angle && range <= no_cone_detection_range) {
             return false; // 检测到物体，不进入无锥桶巡航
         }
     }
     
     return true; // 扇形区域内无物体，可以进入无锥桶巡航
 }

 /**
  * @brief 检测直角转弯条件 - 使用精确的锥桶检测算法
  * @details 通过分析前方锥桶分布模式检测直角转弯
  *          使用与正常循迹相同的锥桶检测算法，确保一致性
  */
 bool PubAndSub::detectRightAngleTurn(const sensor_msgs::msg::LaserScan::SharedPtr& laser)
 {
     // ===== 使用原有的锥桶检测算法获取精确的锥桶坐标 =====
     int j = 0, k = 0;
     Point_rectangular red_p[30] = {0,0};
     Point_rectangular blue_p[30] = {0,0};
     
     // 调用原有的锥桶检测算法
     detectCones(laser, red_p, blue_p, j, k);
     
     // ===== 分析锥桶分布模式来判断直角转弯 =====
     
     // 1. 统计前方区域的锥桶数量
     int front_red_count = 0;
     int front_blue_count = 0;
     double min_front_distance = 10.0;
     
     // 分析红锥桶（右侧）的分布
     for (int i = 0; i < j; i++) {
         // 前方区域定义：Y坐标>0（前方），X坐标在一定范围内
         if (red_p[i].y > 0 && red_p[i].y < 2.0 && abs(red_p[i].x) < 1.0) {
             front_red_count++;
             if (red_p[i].y < min_front_distance) {
                 min_front_distance = red_p[i].y;
             }
         }
     }
     
     // 分析蓝锥桶（左侧）的分布
     for (int i = 0; i < k; i++) {
         if (blue_p[i].y > 0 && blue_p[i].y < 2.0 && abs(blue_p[i].x) < 1.0) {
             front_blue_count++;
             if (blue_p[i].y < min_front_distance) {
                 min_front_distance = blue_p[i].y;
             }
         }
     }
     
     int total_front_cones = front_red_count + front_blue_count;
     
     // 2. 检测侧方障碍（形成直角的边界）
     bool has_left_boundary = false;
     bool has_right_boundary = false;
     
     // 检测左侧边界（左侧较远的蓝锥桶）
     for (int i = 0; i < k; i++) {
         if (blue_p[i].x > 0.3 && blue_p[i].x < 1.2 && 
             blue_p[i].y > 0.5 && blue_p[i].y < 1.5) {
             has_left_boundary = true;
             break;
         }
     }
     
     // 检测右侧边界（右侧较远的红锥桶）
     for (int i = 0; i < j; i++) {
         if (red_p[i].x < -0.3 && red_p[i].x > -1.2 && 
             red_p[i].y > 0.5 && red_p[i].y < 1.5) {
             has_right_boundary = true;
             break;
         }
     }
     
     // 3. 检测直角转弯的特征模式
     bool has_right_angle_pattern = false;
     
     // 模式1：L型直角（左侧边界 + 前方空旷）
     if (has_left_boundary && total_front_cones <= 1) {
         has_right_angle_pattern = true;
         RCLCPP_DEBUG(this->get_logger(), "检测到左直角模式");
     }
     
     // 模式2：反L型直角（右侧边界 + 前方空旷）
     if (has_right_boundary && total_front_cones <= 1) {
         has_right_angle_pattern = true;
         RCLCPP_DEBUG(this->get_logger(), "检测到右直角模式");
     }
     
     // 模式3：T型路口（两侧都有边界 + 前方空旷）
     if (has_left_boundary && has_right_boundary && total_front_cones == 0) {
         has_right_angle_pattern = true;
         RCLCPP_DEBUG(this->get_logger(), "检测到T型路口模式");
     }
     
     // 4. 直角转弯判断条件
     bool right_angle_condition = has_right_angle_pattern && 
                                 (min_front_distance > right_angle_min_gap || total_front_cones == 0);
     
     // 调试信息
     RCLCPP_DEBUG(this->get_logger(), 
                 "直角检测 | 前方锥桶: %d(红:%d+蓝:%d) | 最小距离: %.2f | 左边界: %d | 右边界: %d | 符合条件: %d",
                 total_front_cones, front_red_count, front_blue_count, 
                 min_front_distance, has_left_boundary, has_right_boundary, right_angle_condition);
     
     return right_angle_condition;
 }

 /**
  * @brief 处理直角转弯状态 - 基于检测到的锥桶分布
  * @details 根据检测到的直角转弯模式选择转向方向
  */
 void PubAndSub::handleRightAngleMode(const sensor_msgs::msg::LaserScan::SharedPtr& laser)
 {
     // 重新检测锥桶分布来确定转向方向
     int j = 0, k = 0;
     Point_rectangular red_p[30] = {0,0};
     Point_rectangular blue_p[30] = {0,0};
     detectCones(laser, red_p, blue_p, j, k);
     
     // 分析应该左转还是右转
     double turn_direction = 1.0;  // 1.0 = 右转, -1.0 = 左转
     std::string turn_direction_str = "右转";
     
     // 基于锥桶分布决定转向方向
     bool prefer_left_turn = false;
     
     // 如果左侧有连续的边界，优先左转
     int left_boundary_count = 0;
     for (int i = 0; i < k; i++) {
         if (blue_p[i].x > 0.3 && blue_p[i].y > 0.5) {
             left_boundary_count++;
         }
     }
     
     // 如果右侧有连续的边界，优先右转
     int right_boundary_count = 0;
     for (int i = 0; i < j; i++) {
         if (red_p[i].x < -0.3 && red_p[i].y > 0.5) {
             right_boundary_count++;
         }
     }
     
     // 决策逻辑：哪侧边界更完整就向哪侧转
     if (left_boundary_count > right_boundary_count + 1) {
         prefer_left_turn = true;
         turn_direction = -1.0;
         turn_direction_str = "左转";
     } else if (right_boundary_count > left_boundary_count + 1) {
         prefer_left_turn = false;
         turn_direction = 1.0;
         turn_direction_str = "右转";
     } else {
         // 默认右转（根据赛道规则调整）
         prefer_left_turn = false;
         turn_direction = 1.0;
         turn_direction_str = "右转(默认)";
     }
     
     // 直角转弯控制：固定大角度转向 + 减速
     twist.angular.z = 85 + turn_direction * right_angle_turn_speed;
     twist.linear.x = right_angle_slow_speed;
     
     // 记录直角转弯状态
     RCLCPP_WARN(this->get_logger(), 
                "直角转弯控制 | 方向: %s | 转向: %.1f° | 速度: %.0f | 左边界: %d | 右边界: %d", 
                turn_direction_str.c_str(), 
                twist.angular.z, 
                twist.linear.x,
                left_boundary_count,
                right_boundary_count);
 }

 /**
  * @brief 更新直角转弯状态 - 增强的状态管理
  */
 void PubAndSub::updateRightAngleMode(const sensor_msgs::msg::LaserScan::SharedPtr& laser)
 {
     // 检测当前帧是否满足直角转弯条件
     bool current_frame_detected = detectRightAngleTurn(laser);
     
     if (current_frame_detected) {
         right_angle_turn_count++;
         
         // 连续多帧检测到直角转弯才确认（防止误触发）
         if (right_angle_turn_count >= right_angle_confirm_count && !is_right_angle_mode) {
             // 只有在正常循迹模式下才允许进入直角转弯
             if (!is_no_cone_mode) {
                 is_right_angle_mode = true;
                 right_angle_start_time = this->now().seconds();
                 right_angle_detected = true;
                 
                 RCLCPP_WARN(this->get_logger(), 
                            "������ 进入直角转弯模式 | 连续检测帧: %d", 
                            right_angle_turn_count);
             }
         }
     } else {
         right_angle_turn_count = 0;
         
         // 退出直角转弯模式的条件
         if (is_right_angle_mode) {
             double duration = this->now().seconds() - right_angle_start_time;
             
             // 转弯时间超过阈值或重新检测到正常锥桶时退出
             if (duration > 3.0) {
                 is_right_angle_mode = false;
                 RCLCPP_INFO(this->get_logger(), 
                            "退出直角转弯模式 | 持续时间: %.2f秒", duration);
             }
         }
     }
     
     // 重置检测标志（每帧更新）
     right_angle_detected = current_frame_detected;
 }

 /**
  * @brief 更新行驶模式状态
  * @details 根据扇形检测结果，自动切换行驶模式
  * @param laser 激光雷达数据
  */
 void PubAndSub::updateDrivingMode(const sensor_msgs::msg::LaserScan::SharedPtr& laser)
 {
     // ===== 优先处理直角转弯模式 =====
     updateRightAngleMode(laser);
     
     // 如果处于直角转弯模式，跳过其他模式检测
     if (is_right_angle_mode) {
         return;
     }
     
     // ===== 原有的无锥桶巡航检测 =====
     bool should_enter_no_cone = shouldEnterNoConeMode(laser);
     
     // 状态切换逻辑与日志
     if (should_enter_no_cone) {
         if (!is_no_cone_mode) {
             is_no_cone_mode = true;
             no_cone_start_time = this->now().seconds();
             RCLCPP_INFO(this->get_logger(), 
                        "进入无锥桶巡航状态 - 扇形区域[距离:%.1fm, 角度:±%.1f°]内无物体", 
                        no_cone_detection_range, no_cone_sector_angle * 180.0 / M_PI);
         }
     } else {
         if (is_no_cone_mode) {
             is_no_cone_mode = false;
             double duration = this->now().seconds() - no_cone_start_time;
             RCLCPP_INFO(this->get_logger(), 
                        "退出无锥桶巡航状态 - 检测到物体, 持续时间: %.2f秒", duration);
         }
     }
 }
 
 /**
  * @brief 处理无锥桶巡航状态
  * @details 在无锥桶状态下，小车低速向左转向，搜索锥桶
  */
 void PubAndSub::handleNoConeMode()
 {
     // 无锥桶巡航状态控制逻辑：固定向左转、低速
     twist.angular.z = no_cone_turn_angle;                       // 设置固定左转角度（单位按下游定义）
     twist.linear.x = no_cone_speed;                             // 设置低速行驶（单位按下游定义）
 }
 
 /**
  * @brief 处理正常循迹状态
  * @details 在正常循迹状态下，使用PID控制器进行精确路径跟踪
  * @param angle PID控制器输出的转向角度
  */
 void PubAndSub::handleNormalMode(double angle)
 {
     // 正常循迹状态控制逻辑：基准舵角82 + PID输出
     twist.angular.z = 85 + angle;                               // 基础转向角度 + PID输出
     twist.linear.x = speed;                                     // 使用基础行驶速度
     
     // 大转向角度时减速，提高安全性
     if (abs(angle) >= 20) {
         twist.linear.x -= 10;                                    // 转向角度过大时减速
     }
 }
 
 /**
  * @brief 锥桶检测：从LaserScan中提取左右锥桶坐标点
  * @details 使用"距离突变+平台稳定性"判据，将符合窗口范围的点分类为红/蓝锥桶
  */
 void PubAndSub::detectCones(const sensor_msgs::msg::LaserScan::SharedPtr& laser,
                             Point_rectangular red_p[], Point_rectangular blue_p[],
                             int &j, int &k)
 {
     int i = 0;
     double theta = 0, tmp_x = 0, tmp_y = 0;
 
     j = 0; k = 0;                                                // 计数清零
     // 注意：此处沿用原逻辑使用固定freq与i+1/i+9访问，保持行为一致
     for (i = 1; i < freq - 9; i++)
     {
         // 距离突变阈值判定 + 前后点一致性校验
         if (laser->ranges[i - 1] - laser->ranges[i] >= 1.5 &&
             laser->ranges[i] < 2.5 &&
             laser->ranges[i] != 0 &&
             laser->ranges[i - 1] - laser->ranges[i+1] >= 1.5)
         {
             int continue_ranges = 0;                             // 连续平台计数
             int continue_dectect = 0;                            // 通过判据标志
 
             // 检查后续10个点的稳定性（平台）
             for(int idx = 1; idx < 10; idx++)
             {
                 if(abs(laser->ranges[i] - laser->ranges[i+idx]) < 0.2)
                 {
                     continue_ranges++;
                     if(continue_ranges >= 6)
                     {
                         continue_dectect = 1;
                         break;
                     }
                 }
             }
 
             // 通过平台检验，计算坐标并按左右分类
             if(continue_dectect == 1)
             {
                 theta = i * laser->angle_increment + laser->angle_min;  // 扫描角度
                 cal_point(theta, laser->ranges[i], tmp_x, tmp_y);       // 极坐标→直角坐标
 
                 // 位置窗口过滤，剔除无效/非赛道内点
                 if((tmp_x <= max_left_dis && tmp_x >= -max_right_dis) &&
                    tmp_y < 2 && tmp_y > -0.2 && tmp_y != 0)
                 {
                     if(tmp_x > 0)                                       // 右侧（红）
                     {
                         if(laser->ranges[i] < 2.5)
                         {
                             red_p[j].x = tmp_x;
                             red_p[j].y = tmp_y;
                             j++;
                         }
                     }
                     else                                                // 左侧（蓝）
                     {
                         blue_p[k].x = tmp_x;
                         blue_p[k].y = tmp_y;
                         k++;
                     }
                 }
             }
         }
     }
 }
 
 /**
  * @brief 数据关联：处理左右锥桶序列中的断裂并补齐对应序列
  * @details 当一侧多于另一侧时，查找距离突变处，将后段拼接到另一侧，以维持序列匹配
  */
 void PubAndSub::associateCones(Point_rectangular red_p[], Point_rectangular blue_p[],
                                int &j, int &k)
 {
     if(k >= j)                                                   // 蓝多或相等：以蓝为基准找断点
     {
         int m = 100;                                             // 断点索引初始化
         for(int i = 1; i < k; i++)
         {
             // 相邻蓝点距离>1.4视为断点
             if(sqrt(pow((blue_p[i-1].x-blue_p[i].x),2) + pow((blue_p[i-1].y-blue_p[i].y),2)) > 1.4)
             {
                 m = i;
                 break;
             }
         }
 
         // 找到断点：将蓝(m..end)拼到红前部，并右移已有红
         if(m != 100)
         {
             for(int i = j - 1; i >= 0; i--)
             {
                 red_p[i + k - m] = red_p[i];
             }
             for(int i = 0; i < k - m; i++)
             {
                 red_p[i] = blue_p[m + i];
             }
             j += (k - m);                                       // 更新红数
             k = m;                                              // 蓝数更新为断点之前
         }
     }
     else                                                         // 红多：对红做类似处理并入蓝
     {
         int m = 100;                                             // 断点索引初始化
         for(int i = j-2; i >= 0; i--)
         {
             if(sqrt(pow((red_p[i].x-red_p[i+1].x),2) + pow((red_p[i].y-red_p[i+1].y),2)) > 1.4)
             {
                 m = i;
             }
         }
 
         if(m != 100)
         {
             // 红(0..m)拼到蓝尾部
             for(int i = 0; i < m + 1; i++)
             {
                 blue_p[k + i] = red_p[i];
             }
 
             // 红左移去除(0..m)
             for(int i = 0; i < j - m - 1; i++)
             {
                 red_p[i] = red_p[i + m + 1];
             }
             j -= (m+1);                                         // 更新红数
             k += (m + 1);                                       // 更新蓝数
         }
     }
 }
 
 /**
  * @brief 标准化：裁剪过多的点并将红蓝数量补齐到一致（≤3对）
  * @details 保持每侧最多3个点；若一侧不足，通过复制近点补齐到与对侧一致
  */
 void PubAndSub::standardizeCones(Point_rectangular red_p[], Point_rectangular blue_p[],
                                  int &j, int &k)
 {
     // 红侧裁剪：若>3，仅保留靠近末尾的3个（沿用原逻辑：丢弃数组前部）
     if(j > 3)
     {
         for(int i = 0; i < 3; i++)
         {
             red_p[i] = red_p[i+1];
         }
         j = 3;
     }
 
     // 蓝侧裁剪：若>3，直接截断
     if(k > 3)
     {
         k = 3;
     }
 
     // 数量补齐：尽量让红蓝数量相等，且不超过3
     if(j > k && k != 0)
     {
         if(k == 1)
         {
             if(j > 2)
             {
                 for(int i = 0; i < 2; i++)
                 {
                     red_p[i] = red_p[i+1];
                     j = 2;
                 }
             }
             blue_p[1] = blue_p[0];
             k = 2;
         }
         else if(k == 2)
         {
             blue_p[2] = blue_p[1];
             k = 3;
         }
     }
     else if(k > j && j != 0)
     {
         if(j == 1)
         {
             k = 2;
             red_p[1] = red_p[0];
             j = 2;
         }
         else if(j == 2)
         {
             red_p[2] = red_p[1];
             red_p[1] = red_p[0];
             j = 3;
         }
     }
 }
 
 /**
  * @brief 处理正常循迹状态 - 完整的锥桶检测和路径规划
  * @details 包含锥桶检测、数据关联、标准化、误差计算、PID输出与速度控制
  * @param laser 激光雷达数据
  */
 void PubAndSub::handleNormalTrackingMode(const sensor_msgs::msg::LaserScan::SharedPtr& laser)
 {
     // ===== 变量初始化 =====
     int i=0, j=0, k=0;                                      // j:红锥桶数量, k:蓝锥桶数量
     double error = 0, angle = 0, rate_sum = 0, error_sum = 0;
     Point_rectangular red_p[30] = {0,0};                    // 红锥桶坐标数组（最多30个）
     Point_rectangular blue_p[30] = {0,0};                   // 蓝锥桶坐标数组（最多30个）
 
     // 1) 锥桶检测：扫描数据→左右点集
     detectCones(laser, red_p, blue_p, j, k);
 
     // 2) 数据关联：根据断点拼接序列，尽量维持对应关系
     associateCones(red_p, blue_p, j, k);
 
     // 3) 标准化：限制数量、补齐到对齐的成对点（最多3对）
     standardizeCones(red_p, blue_p, j, k);
 
     // ===== 路径误差计算 =====
     // 使用加权平均方法计算路径跟踪误差，按预设权重rate_p
     Point_rectangular* point_to_red = &red_p[j-1];          // 指向最后一个红锥桶
     int error_count = j < k ? j : k;                         // 使用的锥桶对数量（取较小值）
 
     if (error_count)
     {
         for (i = 0; i < error_count; i++)
         {
             // 误差项：蓝[i].x + 红(从末尾向前).x，代表中心线偏移
             error_sum += (blue_p[i].x + point_to_red->x) * rate_p[i];
             rate_sum += rate_p[i];
             point_to_red--;                                  // 指向前一个红锥桶
         }
         // 最终误差：加权平均 × 系数（将米转换到舵机角度域的经验系数）
         error = error_sum / rate_sum * 53.33;
     }
 
     // ===== PID调节与速度控制 =====
     angle = pid.PIDPositional(error);                        // PID位置式输出舵机修正量
     handleNormalMode(angle);                                 // 组合成最终舵机角与速度
 }
 
 /**
  * @brief 定时打印当前状态信息
  */
 void PubAndSub::printStatus()
 {
     // 获取当前状态描述
     std::string current_mode;
     if (is_right_angle_mode) {
         current_mode = "直角转弯状态";
     } else if (is_no_cone_mode) {
         current_mode = "无锥桶巡航状态";
     } else {
         current_mode = "正常循迹状态";
     }
     
     // 获取当前控制参数（注意：此处按下游约定的单位记录显示）
     double current_speed = twist.linear.x;
     double current_angle = twist.angular.z;
     
     // 打印状态信息
     RCLCPP_INFO(this->get_logger(), 
                 "=== 状态监控 === 模式: %s | 速度: %.0f | 舵机角度: %.1f° | 时间: %.1fs", 
                 current_mode.c_str(), 
                 current_speed, 
                 current_angle,
                 this->now().seconds());
 }
 
 /**
  * @brief 激光雷达数据回调函数
  * @details 处理激光雷达扫描数据，进行扇形检测，控制车辆行驶
  * @param laser 激光雷达扫描数据指针
  */
 void PubAndSub::callback(const sensor_msgs::msg::LaserScan::SharedPtr laser)
 {
     // ===== 状态管理和控制策略选择 =====
     updateDrivingMode(laser);                      // 使用扇形检测更新行驶模式
     
     // 根据当前状态选择控制策略（直角转弯优先级最高）
     if (is_right_angle_mode) {
         // 直角转弯状态：专用控制策略
         handleRightAngleMode(laser);
     } else if (is_no_cone_mode) {
         // 无锥桶巡航状态：使用固定控制策略
         handleNoConeMode();
     } else {
         // 正常循迹状态：使用完整的锥桶检测和路径规划逻辑
         handleNormalTrackingMode(laser);
     }
     
     // ===== 输出限制和发布 =====
     // 角度限制：确保转向角度在有效范围内（按下游硬件/接口定义）
     if (twist.angular.z > 180)
         twist.angular.z = 180;                                    // 最大右转角度限制
     if (twist.angular.z < 0)
         twist.angular.z = 0;                                      // 最大左转角度限制
         
     pub_->publish(twist);                                         // 发布速度控制指令
 }
 
 /**
  * @brief 主函数：程序入口点
  * @details 初始化ROS2系统，创建节点，进入事件循环
  * @param argc 命令行参数数量
  * @param argv 命令行参数数组
  * @return 程序退出码
  */
 int main(int argc, char *argv[])
 {
     pid.Init();                                                   // 初始化PID控制器参数
     rclcpp::init(argc, argv);                                     // 初始化ROS2系统
     auto node = std::make_shared<PubAndSub>();                    // 创建节点实例
     rclcpp::spin(node);                                           // 进入事件循环，处理回调
     rclcpp::shutdown();                                           // 关闭ROS2系统
     return 0;                                                     // 程序正常退出
 }