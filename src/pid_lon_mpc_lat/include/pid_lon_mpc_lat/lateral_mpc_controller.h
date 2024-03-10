#include <Eigen/Sparse>
#include "OsqpEigen/OsqpEigen.h"
#include <pid_lon_mpc_lat/trajectory_processor.h>
#include <pid_lon_mpc_lat/vehicle_simulator.h>
#include <unsupported/Eigen/MatrixFunctions>

using namespace Eigen;

/*
    ** 类名称：LateralMpcController

    ** 简介：
    本类用于求解MPC问题得到横向控制量。
*/
class LateralMpcController{
public:
    LateralMpcController();
    
    // 预测时域和控制时域和步长
    int Np_ = 20;
    int Nc_ = 10;
    double Ts_ = 0.05;
    // 目标函数惩罚权重
    double w_r_ = 0.0;
    double w_vy_ = 0.5;
    double w_eyaw_ = 1.0;
    double w_ey_ = 2.0;
    double w_delta_ = 5000.0;
    double w_s_ey_ = 500.0;
    double w_s_eyaw_ = 500.0;
    // 状态量约束
    double max_ey_ = 0.05;
    double max_eyaw_ = 5.0 / 180.0 * M_PI;
    // 控制量约束
    double max_delta_ = 30.0 / 180.0 * M_PI;
    double max_dert_delta_ = 100.0 / 180 * M_PI;


    // 最优值
    double primal_value_;

    // 最优解
    VectorXd QPSolution_;

    // 控制量与状态量维度
    static const int nx = 4;
    static const int nu = 1;

    // 判断是否为初次调用
    bool first_run_ = true;

    // 求解器求解成功与否flag
    bool solver_status_ = true;

    /*
        定义MPC优化问题，并求解得到横向控制量
        输入: 当前状态量，控制量，目标轨迹
    */
    void solveMpc(double cur_r, double cur_vy, double cur_eyaw, double cur_ey, double  cur_delta, vector<NodeStamped> track, VehicleParam vehicle_model);

    /*
        在参考点处对非线性动力学模型线性化
        输入: 系统矩阵，控制矩阵，常数矩阵，参考点，当前控制量，预测时域内当前点索引
    */
    void getLinearDynamics(Matrix<double, nx, nx> &Ad, Matrix<double, nx, nu>& Bd, Matrix<double, nx, 1>& hd,
        vector<double> reference_state, double ref_delta, int i, VehicleParam vehicle_model, vector<NodeStamped> track);
};