#include "ros/ros.h"
#include <nav_msgs/Odometry.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>

using namespace std;
#ifndef VEHICLE_SIMULATOR_H
#define VEHICLE_SIMULATOR_H
/*
    ** 节点名称：vehicle_simulator

    ** 节点输入（模拟执行器输入）:
    前轮转角delta;
    轮上驱动力矩T；

    ** 节点输出（模拟传感器测量）：
    笛卡尔坐标系车辆位置坐标{x, y};
    车辆横摆角yaw;
    车速v;
    横摆角速度r;
    横向速度vy；

    ** 上游节点：
    vehicle_controller
    ** 下游节点：
    vehicle_controller

    ** 简介：
    本节点为车辆仿真器，用于模拟真实车辆，验证控制算法；
    假设车辆为前驱前轮转向车辆；
    本节点车辆模型采用横纵向耦合动力学模型，轮胎模型采用魔术公式，参数为捏造。
*/
// 车辆状态量
struct VehicleState{
    double x = 0.0;
    double y = 0.0;
    double yaw = 0.0;
    double vx = 0.0;
    double vy = 0.0;
    double r = 0.0;
};
// 车辆控制量
struct ControlCmd{
    double T = 0.0;
    double delta = 0.0;
};
struct VehicleParam{
    // 质心距前轴距离
    double l_f = 1.165;
    // 质心距后轴距离
    double l_r = 1.165;
    // 车辆质量
    double m = 1140.0;
    // 车辆横摆惯量
    double I_z = 2849.5;
    // 车辆轴距
    double l = 2.33;
    // 重力系数
    double g = 9.8;
    // 前轴魔术公式参数
    double B_f = 12.6167;
    double C_f = 1.3;
    double D_f = 4748.0;
    double E_f = -1.5;
    // 后轴魔术公式
    double B_r = 13.7677;
    double C_r = 1.3;
    double D_r = 4748.0;
    double E_r = -1.5;
    // 轮胎半径
    double r_wheel = 0.298;
};

class Vehicle {
    public: 
        VehicleState sim_vehicle_state_;
        ControlCmd sim_control_cmd_;
        VehicleParam sim_vehicle_param_;

        /*
            构造函数
            输入：无输入
        */
        Vehicle(){
            sim_vehicle_state_.x = 0.0;
            sim_vehicle_state_.y = 0.0;
            sim_vehicle_state_.yaw = 0.0;
            sim_vehicle_state_.vx = 0.0;
            sim_vehicle_state_.vy = 0.0;
            sim_vehicle_state_.r = 0.0;
            sim_control_cmd_.delta = 0.0;
            sim_control_cmd_.T = 0.0;
        }

        /*
            重载构造函数
            输入：各车辆状态初值
        */
        Vehicle(double x_init, double y_init, double yaw_init, double v_init, double vy_init, double r_init);
};

class VehicleSimulator{
public:
    Vehicle* vehicle_;
    vector<vector<double>> target_path_;

    /*
        VehicleSimulator构造函数
        输入：各状态初值，
    */
    VehicleSimulator(double x_init, double y_init, double yaw_init, double vx_init, double vy_init, double r_init, ros::NodeHandle& nh);

    /*
        析构函数
    */
   virtual ~VehicleSimulator();
   
    /*
        输入：ackermann_msgs::AckermannDriveStamped
        输出：void
    */
    void vehicleModelCallback(const ackermann_msgs::AckermannDriveStampedConstPtr &msgs);

    /*
        publisher发送数据
        输入：无
        输出：无
    */
    void publishData();
private:
    ros::Publisher state_pub;
    ros::Publisher path_pub;
    ros::Subscriber cmd_sub;

    nav_msgs::Path real_path_data;
};

#endif