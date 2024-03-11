#include "ros/ros.h"
#include <nav_msgs/Odometry.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <pid_lon_mpc_lat/vehicle_simulator.h>
#include <pid_lon_mpc_lat/trajectory_processor.h>
# include <pid_lon_mpc_lat/lateral_mpc_controller.h>
# include <pid_lon_mpc_lat/longitudinal_pid_controller.h>

using namespace std;
/*
    ** 节点名称：vehicle_controller

    ** 节点输入（模拟传感器测量）:
    笛卡尔坐标系车辆位置坐标{x, y};
    车辆横摆角yaw;
    车速v;
    横摆角速度r;
    横向速度vy；

    ** 节点输出（模拟控制量传输）：
    前轮转角delta;
    轮上驱动力矩T；

    ** 上游节点：
    vehicle_simulator
    ** 下游节点：
    vehicle_simulator

    ** 简介：
    本节点为车辆控制器，根据传感器/上游传来的状态量与目标轨迹，计算前轮转角与轮上驱动力矩，使车辆跟踪已知轨迹；
    假设车辆为前驱前轮转向车辆；
    根据题目要求，本节点横纵向解耦控制，纵向利用PID，横向利用线性时变MPC；
    主要包含三部分：轨迹处理器，纵向控制器，横向控制器
*/

class VehicleController{
public:
    VehicleState vehicle_state_;
    ControlCmd control_cmd_;
    VehicleParam vehicle_param_;

    // 用于处理轨迹
    TrajectoryProcessor* trajectory_processor_;
    // 用于纵向控制
    LongitudinalPidController* pid_controller_;
    // 用于横向控制
    LateralMpcController* mpc_controller_;

    // 判定此时是高速动力学模型[1]还是低速运动学模型[0]
    int lateral_flag = 0;

    double e_y = 0.0;
    double e_yaw = 0.0;
    double e_v = 0.0;

    /*
        重载构造函数
        输入：节点
    */
    VehicleController(ros::NodeHandle& nh);

    /*
        析构函数
    */
    ~VehicleController();

    /*
        回调函数
        输入：nav_msgs::OdometryConstPtr
        输出：无
    */
    void stateCallback(const nav_msgs::OdometryConstPtr &msgs);
    /*
        发送控制指令
    */
    void publishCmd();
private:
    ros::Publisher cmd_pub;
    ros::Subscriber state_sub;
};