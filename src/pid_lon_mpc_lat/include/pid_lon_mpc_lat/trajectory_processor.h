#include <pid_lon_mpc_lat/spline.h>
#include <vector>
using namespace std;
#ifndef TRAJECTORY_PROCESSOR_H
#define TRAJECTORY_PROCESSOR_H

struct NodeStamped {
    double x, y, yaw;
    double vx;
    double curvature;
};

/*
    ** 类名称：TrajectoryProcessor

    ** 简介：
    本类用于处理输入的轨迹点集合，查找匹配点，实时计算横向偏差与横摆角偏差，计算预测时域内序列
*/
class TrajectoryProcessor {
private:
    // 构建轨迹点的曲线
    tk::spline spline_x_;
    tk::spline spline_y_;
    tk::spline spline_v_;
    tk::spline spline_s_;
public:
    TrajectoryProcessor();
    vector<double> cur_curvature;
    double  spline_max_i_; 
    double cur_error_y;
    double cur_min_i_ = 0.01;
    double final_min_i_;
    
    /*
        加载轨迹点集
        输入：x,y,v的序列
    */
    bool loadTrajectory(vector<double> node_x_lst, vector<double> node_y_lst, vector<double> node_v_lst);
    
    /*
        查找匹配点, 计算预测时域内序列
        输入：当前车辆位置x,y, 预测时域长度，时间步长
    */
    void getTrajectory(double x, double y, int n, double dt, vector<NodeStamped> *path);
    
    /*
        查找匹配点
        输入：当前车辆位置x,y
        输出：spline中的最近点
    */
   double findNearestPoint(double x, double y);

    /*
        查找预瞄距离的匹配点
        输入：预测时域初始点，距离
        输出：对应匹配点索引
    */
   double findDistancePoint(double i_start, double l_i, double eps = 1e-8);
};

#endif