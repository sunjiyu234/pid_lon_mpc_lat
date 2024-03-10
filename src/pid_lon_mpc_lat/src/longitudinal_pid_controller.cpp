#include <pid_lon_mpc_lat/longitudinal_pid_controller.h>
using namespace std;

LongitudinalPidController::LongitudinalPidController(){
}

double LongitudinalPidController::solveCmd(double error_v){
    // 计算纵向控制量T的变化量
    double dert_cmd_T;
    dert_cmd_T = pid_kp_ * (error_v - last_error_) + pid_ki_ * (error_v) + pid_kd_ * (error_v - 2 * last_error_ + lastlast_error_);
    lastlast_error_ = last_error_;
    last_error_ = error_v;
    return dert_cmd_T;
}
