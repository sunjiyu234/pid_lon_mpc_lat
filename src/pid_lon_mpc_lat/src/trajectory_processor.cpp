# include <pid_lon_mpc_lat/trajectory_processor.h>
# include <ros/ros.h>
# include <iostream>
# include <fstream>

using namespace std;

TrajectoryProcessor::TrajectoryProcessor(){
}

bool TrajectoryProcessor::loadTrajectory(vector<double> node_x_lst, vector<double> node_y_lst, vector<double> node_v_lst){
    /*
    ** 判断轨迹点集是否符合要求
    * 各维度的点的数量是否相等
    * 点的数量是否大于0
    */ 
    if (node_x_lst.size() != node_y_lst.size() || node_x_lst.size() != node_v_lst.size()){
        ROS_INFO("Wrong !!! Node number is not same !");
        return false;
    }
    if (node_x_lst.size() <= 0){
        ROS_INFO("Wrong !!! Node number < 0 !");
        return false;
    }
    // 构建索引序列与里程序列
    vector<double> node_i_lst;
    vector<double> node_s_lst;
    double cur_i = 0.0;
    for (int i = 0; i < node_x_lst.size(); i++){
        node_i_lst.push_back(cur_i);
        cur_i += 1.0;
        if (i == 0){
            node_s_lst.push_back(0.0);
        } else{
            node_s_lst.push_back(node_s_lst[i - 1] + sqrt((node_x_lst[i] - node_x_lst[i - 1]) * (node_x_lst[i] - node_x_lst[i - 1]) + (node_y_lst[i] - node_y_lst[i - 1]) * (node_y_lst[i] - node_y_lst[i - 1])));
        }
    }
    spline_max_i_ = node_x_lst.size() - 1;
    final_min_i_ = spline_max_i_;

    // 生成点的曲线
    spline_x_.set_points(node_i_lst, node_x_lst);
    spline_y_.set_points(node_i_lst, node_y_lst);
    spline_v_.set_points(node_i_lst, node_v_lst);
    spline_s_.set_points(node_i_lst, node_s_lst);


    // 以0.01为space，计算各点的曲率
    for (double i = 0.01; i < spline_max_i_; i = i + 0.01){
        double kr_i;
        // 计算曲率
        kr_i = (spline_x_.deriv(1, i) * spline_y_.deriv(2, i) - spline_x_.deriv(2, i) * spline_y_.deriv(1, i)) / pow((spline_x_.deriv(1, i) * spline_x_.deriv(1, i) + spline_y_.deriv(1, i) * spline_y_.deriv(1, i)), 1.5);
        // 通过横摆角变化判断曲率为正/负
        double yaw_dert_i = atan2(spline_y_(i + 0.01) - spline_y_(i), spline_x_(i + 0.01) - spline_x_(i)) - atan2(spline_y_(i) - spline_y_(i - 0.01), spline_x_(i) - spline_x_(i - 0.01));
        if (yaw_dert_i > M_PI){
            yaw_dert_i -= 2 * M_PI;
        } else if (yaw_dert_i < -M_PI){
            yaw_dert_i += 2 * M_PI;
        }
        if (yaw_dert_i < 0.0){
            kr_i = -kr_i;
        }
        // 判断曲率是否满足要求，若不满足，则沿用上一点曲率
        if (abs(kr_i) > 0.1){
            cur_curvature.push_back(cur_curvature[cur_curvature.size() - 1]);
        } else{
            cur_curvature.push_back(kr_i);
        }
    }
    cur_curvature.push_back(cur_curvature[cur_curvature.size() - 1]);
    ROS_INFO("Finish load trajectory");
}

void TrajectoryProcessor::getTrajectory(double x, double y, int n, double dt, vector<NodeStamped> *path){
    // 清空现有预测时域内轨迹
    path->clear();

    // 找当前xy的匹配点，返回i，作为预测时域的起始点
    double i_start = findNearestPoint(x, y);
    path->push_back(NodeStamped({
        spline_x_(i_start), spline_y_(i_start), atan2(spline_y_.deriv(1, i_start), spline_x_.deriv(1, i_start)),
        spline_v_(i_start),
        cur_curvature.at(static_cast<int>(i_start / 0.01) - 1)
    }));

    // 从起始点开始，寻找预测时域内各点的匹配点
    double l_i = 0.0;
    double last_i = i_start;
    for (int i = 0; i < n - 1; i++){
        l_i = spline_v_(last_i) * dt;
        double new_i = findDistancePoint(last_i, l_i);
        if (i == n - 2){
            final_min_i_ = new_i;
        }
        path->push_back(NodeStamped({
            spline_x_(new_i), spline_y_(new_i), atan2(spline_y_.deriv(1, new_i), spline_x_.deriv(1, new_i)),
            spline_v_(new_i),
            cur_curvature.at(static_cast<int>(new_i / 0.01) - 1)
        }));
        last_i = new_i;
    }
}

double TrajectoryProcessor::findNearestPoint(double x, double y){
    double min_dist = 1.0 / 0.0;
    double min_i = cur_min_i_;
    // 从上一初始匹配点开始遍历所有点，寻找最近i
    for (double i = cur_min_i_; i < final_min_i_; i = i + 0.01){
        double cur_dist = sqrt((x - spline_x_(i)) * (x - spline_x_(i)) + (y - spline_y_(i)) * (y - spline_y_(i)));
        if (cur_dist < min_dist){
            min_dist = cur_dist;
            min_i = i;
        }
    }
    cur_error_y = min_dist;
    // 若当前点在目标轨迹右侧，error_y为负数
    if ((y - spline_y_(min_i)) * cos(atan2(spline_y_.deriv(1, min_i), spline_x_.deriv(1, min_i))) - (x - spline_x_(min_i))  * sin(atan2(spline_y_.deriv(1, min_i), spline_x_.deriv(1, min_i))) < 0){
        cur_error_y = - cur_error_y;
    }
    cur_min_i_ = min_i;
    return min_i;
}

double TrajectoryProcessor::findDistancePoint(double i_start, double l_i, double eps){
    // 二分法找对应匹配点
    double l_index = i_start;
    double r_index = spline_max_i_;
    while ((r_index - l_index) > eps){
        double m_index = (l_index + r_index) / 2.0;
        if (spline_s_(m_index) - spline_s_(i_start) <= l_i){
            l_index = m_index;
        } else {
            r_index = m_index;
        }
    }
    return (l_index + r_index) / 2.0;
}

