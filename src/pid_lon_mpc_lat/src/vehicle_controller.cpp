#include <ros/ros.h>
#include <pid_lon_mpc_lat/read_csv.h>
#include <pid_lon_mpc_lat/vehicle_controller.h>
#include <tf/tf.h>
#include <iostream>
#include <time.h>
#include <fstream>
using namespace std;

double dt = 0.05;
 

VehicleController::VehicleController(ros::NodeHandle& nh){
    trajectory_processor_ = new TrajectoryProcessor();
    pid_controller_ = new LongitudinalPidController();
    mpc_controller_ = new LateralMpcController();
    cmd_pub = nh.advertise<ackermann_msgs::AckermannDriveStamped>("/control_cmd", 10);
    state_sub = nh.subscribe("/vehicle_state", 10, &VehicleController::stateCallback, this);
}

VehicleController::~VehicleController(){
    delete trajectory_processor_;
    delete pid_controller_;
    delete mpc_controller_;
}

void VehicleController::stateCallback(const nav_msgs::OdometryConstPtr &msgs){
    // 获取当前车辆状态
    vehicle_state_.x = msgs->pose.pose.position.x;
    vehicle_state_.y = msgs->pose.pose.position.y;
    vehicle_state_.yaw = tf::getYaw(msgs->pose.pose.orientation);
    vehicle_state_.vx = msgs->twist.twist.linear.x;
    vehicle_state_.vy = msgs->twist.twist.linear.y;
    vehicle_state_.r = msgs->twist.twist.angular.z;
}

void VehicleController::publishCmd(){
    ackermann_msgs::AckermannDriveStamped cmd_data;
    cmd_data.drive.acceleration = control_cmd_.T;
    cmd_data.drive.steering_angle = control_cmd_.delta;
    cmd_pub.publish(cmd_data);
}

int main(int argc, char **argv){
    ros::init(argc, argv, "vehicle_controller");
    ros::NodeHandle nh;

    string waypoint_file;

    // 初始化控制器
    VehicleController vehicle_controller(nh);

    // 打开文件，存储车速偏差与ey，eyaw偏差
    ofstream outfile;
    outfile.open("/home/sun234/pid_lon_mpc_lat/src/pid_lon_mpc_lat/data/U_road_error.txt");

    // 读取目标路径点集
    nh.getParam("waypoint_file", waypoint_file);
    CSVReader file_reader(waypoint_file, ",");
    vector<vector<string>> target_road_string = file_reader.readData();
    vector<double>  node_x_lst;
    vector<double>  node_y_lst;
    vector<double>  node_v_lst;
    for (int row = 0; row < target_road_string.size(); row++){
        node_x_lst.push_back(stof(target_road_string[row][0]));
        node_y_lst.push_back(stof(target_road_string[row][1]));
        node_v_lst.push_back(stof(target_road_string[row][2]));
    }

    // 初步加载轨迹点，生成曲线
    vehicle_controller.trajectory_processor_->loadTrajectory(node_x_lst, node_y_lst, node_v_lst);
    vector<NodeStamped> track;
    // 定义循环频率为50Hz
    ros::Rate rate(50);

    // 开始循环
    while (ros::ok()){
        // 开始计时
        double time_start = clock();
        ros::spinOnce();

        // 判断是否到终点
        if (abs(sqrt(pow(vehicle_controller.vehicle_state_.x - node_x_lst[node_x_lst.size() - 1], 2) + pow(vehicle_controller.vehicle_state_.y - node_y_lst[node_y_lst.size() - 1], 2))) < 0.3)
        {
            vehicle_controller.control_cmd_.T = -3001.0;
            vehicle_controller.control_cmd_.delta = 0.0;
            cout <<"goal"<<endl;
            vehicle_controller.publishCmd();
            break;
        }

        // 如果车速小于0.001，补到0.001
        if (abs(vehicle_controller.vehicle_state_.vx) < 0.001){
            ROS_INFO("Now vx is 0.0m/s !");
            vehicle_controller.vehicle_state_.vx += 0.001;
        }

        // 获取当前状态下对应的预测时域目标轨迹
        vehicle_controller.trajectory_processor_->getTrajectory(vehicle_controller.vehicle_state_.x, vehicle_controller.vehicle_state_.y, vehicle_controller.mpc_controller_->Np_, dt, &track);

        // 计算横向偏差，横摆角偏差，车速偏差
        vehicle_controller.e_y = vehicle_controller.trajectory_processor_->cur_error_y;
        vehicle_controller.e_yaw = vehicle_controller.vehicle_state_.yaw - track[0].yaw;
        if (vehicle_controller.e_yaw > M_PI){
            vehicle_controller.e_yaw -= 2.0 * M_PI;
        } else if (vehicle_controller.e_yaw < -M_PI){
            vehicle_controller.e_yaw += 2.0 * M_PI;
        }
        vehicle_controller.e_v = track[0].vx - vehicle_controller.vehicle_state_.vx;

        
        // 调用纵向PID控制器
        vehicle_controller.control_cmd_.T += vehicle_controller.pid_controller_->solveCmd(vehicle_controller.e_v);
        vehicle_controller.control_cmd_.T = max(-2500.0, min(2500.0, vehicle_controller.control_cmd_.T));

        // 调用横向MPC控制器
        if (abs(vehicle_controller.vehicle_state_.vx) < 0.5){
            // 运动学模型
            ROS_INFO("Now is kinematics !");
            vehicle_controller.control_cmd_.delta = atan(track[0].curvature * vehicle_controller.vehicle_param_.l);
            vehicle_controller.mpc_controller_->first_run_ = true;
        } else{
            // 动力学模型
            ROS_INFO("Now is dynamic !");
            vehicle_controller.mpc_controller_->solveMpc(vehicle_controller.vehicle_state_.r, vehicle_controller.vehicle_state_.vy, vehicle_controller.e_yaw,
                vehicle_controller.e_y, vehicle_controller.control_cmd_.delta, track, vehicle_controller.vehicle_param_);
            if (vehicle_controller.mpc_controller_->solver_status_){
                vehicle_controller.control_cmd_.delta = vehicle_controller.mpc_controller_->QPSolution_(vehicle_controller.mpc_controller_->Np_ * vehicle_controller.mpc_controller_->nx);
                vehicle_controller.mpc_controller_->first_run_ = false;
            }
        }

        vehicle_controller.control_cmd_.delta = max(-30.0 / 180.0 * M_PI, min(30.0 / 180.0 * M_PI, vehicle_controller.control_cmd_.delta));

        // 发送控制指令
        vehicle_controller.publishCmd();

        // 结束计时
        double time_end = clock();
        ROS_INFO("Time Use = %f", (time_end - time_start) / CLOCKS_PER_SEC);

        outfile << vehicle_controller.e_v << "," << vehicle_controller.e_yaw <<  "," << vehicle_controller.e_y  << "," << (time_end - time_start) / CLOCKS_PER_SEC << endl;

        rate.sleep();
    }
    outfile.close();

}