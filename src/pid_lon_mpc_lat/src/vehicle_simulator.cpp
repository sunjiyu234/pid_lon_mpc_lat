#include <ros/ros.h>
#include <pid_lon_mpc_lat/vehicle_simulator.h>
#include <string>
#include <pid_lon_mpc_lat/read_csv.h>
#include <vector>
#include <math.h>
#include <tf/tf.h>
#include <fstream>
using namespace std;
double dt = 0.02;

Vehicle::Vehicle(double x_init, double y_init, double yaw_init, double vx_init, double vy_init, double r_init){
    sim_vehicle_state_.x = x_init;
    sim_vehicle_state_.y = y_init;
    sim_vehicle_state_.yaw = yaw_init;
    sim_vehicle_state_.vx = vx_init;
    sim_vehicle_state_.vy = vy_init;
    sim_vehicle_state_.r = r_init;
    sim_control_cmd_.delta = 0.0;
    sim_control_cmd_.T = 0.0;
}

VehicleSimulator::VehicleSimulator(double x_init, double y_init, double yaw_init, double vx_init, double vy_init, double r_init, ros::NodeHandle& nh){
    vehicle_ = new Vehicle(x_init, y_init, yaw_init, vx_init, vy_init, r_init);
    state_pub = nh.advertise<nav_msgs::Odometry>("/vehicle_state", 10);
    cmd_sub = nh.subscribe("/control_cmd", 10, &VehicleSimulator::vehicleModelCallback, this);
    path_pub = nh.advertise<nav_msgs::Path>("/real_path", 10);
}

VehicleSimulator::~VehicleSimulator(){
    delete vehicle_;
}

void VehicleSimulator::vehicleModelCallback(const ackermann_msgs::AckermannDriveStampedConstPtr &msgs){
    // 获取当前控制量
    vehicle_->sim_control_cmd_.T = msgs->drive.acceleration;
    vehicle_->sim_control_cmd_.delta = msgs->drive.steering_angle;
    if (vehicle_->sim_control_cmd_.T < -3000.0){
            return;
    }
    // 为方便写代码，取简化名字
    double cur_delta = vehicle_->sim_control_cmd_.delta;
    double cur_T = vehicle_->sim_control_cmd_.T;
    double cur_x = vehicle_->sim_vehicle_state_.x;
    double cur_y = vehicle_->sim_vehicle_state_.y;
    double cur_yaw = vehicle_->sim_vehicle_state_.yaw;
    double cur_vx =  vehicle_->sim_vehicle_state_.vx;
    double cur_vy = vehicle_->sim_vehicle_state_.vy;
    double cur_v = sqrt(cur_vx * cur_vx + cur_vy * cur_vy);
    double cur_r = vehicle_->sim_vehicle_state_.r;
    // 车速低于0.5m/s 运动学模型更新车辆状态
    // 车速高于0.5m/s 动力学模型更新车辆状态
    if (abs(vehicle_->sim_vehicle_state_.vx) < 0.5){
        vehicle_->sim_vehicle_state_.x = cur_x +(cur_vx * cos(cur_yaw)) * dt;
        vehicle_->sim_vehicle_state_.y = cur_y +(cur_vx * sin(cur_yaw)) * dt;
        vehicle_->sim_vehicle_state_.yaw = cur_yaw + (cur_vx * tan(cur_delta) / vehicle_->sim_vehicle_param_.l) * dt;
        vehicle_->sim_vehicle_state_.vx = cur_vx + (cur_T * 2.0) / vehicle_->sim_vehicle_param_.m * dt;
    } else{
        // 质心侧偏角
        double beta = asin(cur_vy /cur_v);
        vehicle_->sim_vehicle_state_.x = cur_x +(cur_v * cos(cur_yaw + beta)) * dt;
        vehicle_->sim_vehicle_state_.y = cur_y +(cur_v * sin(cur_yaw + beta)) * dt;
        vehicle_->sim_vehicle_state_.yaw = cur_yaw + (cur_r) * dt;
        // 计算前轴纵向力
        double Fxf = cur_T * 2.0 / vehicle_->sim_vehicle_param_.r_wheel;
        // 计算前后轮侧偏角
        double alpha_f = -(atan((cur_v * sin(beta) + vehicle_->sim_vehicle_param_.l_f * cur_r) / (cur_v * cos(beta))) - cur_delta);
        double alpha_r = -(atan((cur_v * sin(beta) - vehicle_->sim_vehicle_param_.l_r * cur_r) / (cur_v * cos(beta))));
        // 计算前后横向力
        double Fyf = vehicle_->sim_vehicle_param_.D_f * sin(vehicle_->sim_vehicle_param_.C_f * atan(vehicle_->sim_vehicle_param_.B_f * alpha_f - vehicle_->sim_vehicle_param_.E_f * (vehicle_->sim_vehicle_param_.B_f * alpha_f - atan(vehicle_->sim_vehicle_param_.B_f * alpha_f))));
        double Fyr = vehicle_->sim_vehicle_param_.D_r *sin(vehicle_->sim_vehicle_param_.C_r * atan(vehicle_->sim_vehicle_param_.B_r * alpha_r - vehicle_->sim_vehicle_param_.E_r * (vehicle_->sim_vehicle_param_.B_r * alpha_r - atan(vehicle_->sim_vehicle_param_.B_r * alpha_r))));
        // 计算状态变化率
        double dot_v = (Fxf * cos(cur_delta - beta) - Fyf * sin(cur_delta - beta) + Fyr * sin(beta)) / vehicle_->sim_vehicle_param_.m;
        double dot_r = (vehicle_->sim_vehicle_param_.l_f * (Fxf * sin(cur_delta) + Fyf * cos(cur_delta)) - vehicle_->sim_vehicle_param_.l_r * Fyr) / vehicle_->sim_vehicle_param_.I_z;
        double dot_beta = -cur_r + (Fxf * sin(cur_delta - beta) + Fyf * cos(cur_delta -  beta) + Fyr * cos(beta)) / (vehicle_->sim_vehicle_param_.m * cur_v);
        cur_v = cur_v + dot_v * dt;
        beta = beta + dot_beta * dt;
        vehicle_->sim_vehicle_state_.r = cur_r + dot_r * dt;
        vehicle_->sim_vehicle_state_.vx = cur_v * cos(beta);
        vehicle_->sim_vehicle_state_.vy = cur_v * sin(beta);
        // 限制横摆角范围[-Pi到Pi]
        nav_msgs::Odometry state_data;
        state_data.pose.pose.orientation = tf::createQuaternionMsgFromYaw(vehicle_->sim_vehicle_state_.yaw);
        vehicle_->sim_vehicle_state_.yaw = tf::getYaw(state_data.pose.pose.orientation);
        ROS_INFO("Finish state update!");
    } 
}

void VehicleSimulator::publishData(){
    // 定义要发送的topic格式
    // 发送车辆状态到controller
    nav_msgs::Odometry state_data;
    state_data.header.stamp = ros::Time::now();
    state_data.header.frame_id = "map";
    state_data.pose.pose.position.x = vehicle_->sim_vehicle_state_.x;
    state_data.pose.pose.position.y = vehicle_->sim_vehicle_state_.y;
    state_data.pose.pose.orientation = tf::createQuaternionMsgFromYaw(vehicle_->sim_vehicle_state_.yaw);
    state_data.twist.twist.linear.x = vehicle_->sim_vehicle_state_.vx;
    state_data.twist.twist.linear.y = vehicle_->sim_vehicle_state_.vy;
    state_data.twist.twist.angular.z = vehicle_->sim_vehicle_state_.r;
    state_pub.publish(state_data);
    ROS_INFO("State publish success !");

    // 发送车辆实时路径到rviz可视化
    real_path_data.header.stamp = ros::Time::now();
    real_path_data.header.frame_id = "map";
    geometry_msgs::PoseStamped real_single_point;
    real_single_point.header.stamp = ros::Time::now();
    real_single_point.header.frame_id = "map";
    real_single_point.pose.position.x = vehicle_->sim_vehicle_state_.x;
    real_single_point.pose.position.y = vehicle_->sim_vehicle_state_.y;
    real_path_data.poses.push_back(real_single_point);
    path_pub.publish(real_path_data);
}

int main(int argc, char **argv){
    ros::init(argc, argv, "vehicle_simulator");
    ros::NodeHandle nh;
    
    string waypoint_file;
    ofstream outfile;
    outfile.open("/home/sun234/pid_lon_mpc_lat/src/pid_lon_mpc_lat/data/U_road.txt");

    ros::Publisher target_path_pub = nh.advertise<nav_msgs::Path> ("/target_path", 10);

    // 读取目标路径点集的起点坐标与航向角
    nh.getParam("waypoint_file", waypoint_file);
    CSVReader file_reader(waypoint_file, ",");
    vector<vector<string>> target_road_string = file_reader.readData();
    ROS_INFO("Get waypoint file !");
    double x_init = stof(target_road_string.at(0).at(0));
    double x_next = stof(target_road_string.at(1).at(0));
    double y_init = stof(target_road_string.at(0).at(1));
    double y_next = stof(target_road_string.at(1).at(1));
    double yaw_init = atan((y_next - y_init) / (x_next - x_init));

    VehicleSimulator vehicle_sim(x_init, y_init, yaw_init, 0.0, 0.0, 0.0, nh);

    // 发送目标路径到rviz可视化
    nav_msgs::Path target_path_data;
    target_path_data.header.stamp = ros::Time::now();
    target_path_data.header.frame_id = "map";
    geometry_msgs::PoseStamped target_single_point;
    target_single_point.header.stamp = ros::Time::now();
    target_single_point.header.frame_id = "map";
    for (int row = 0; row < target_road_string.size(); row++){
        target_single_point.pose.position.x = stof(target_road_string.at(row).at(0));
        target_single_point.pose.position.y = stof(target_road_string.at(row).at(1));
        target_path_data.poses.push_back(target_single_point);
    }



    ROS_INFO("Finish initialization !");

    // 定义循环频率为50Hz
    ros::Rate rate(50);

    // 开始循环
    while (ros::ok()){
        ros::spinOnce();
        target_path_pub.publish(target_path_data);
        // 发送状态
        vehicle_sim.publishData();
        outfile << vehicle_sim.vehicle_->sim_vehicle_state_.vx << endl;
        if (vehicle_sim.vehicle_->sim_control_cmd_.T < -3000.0){
            break;
        }
        rate.sleep();
    }
    outfile.close();
}