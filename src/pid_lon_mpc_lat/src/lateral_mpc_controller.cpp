#include <pid_lon_mpc_lat/lateral_mpc_controller.h>\
#include <ros/ros.h>

using namespace Eigen;
using namespace std;

LateralMpcController::LateralMpcController(){
}

void LateralMpcController::getLinearDynamics(Matrix<double, nx, nx> &Ad, Matrix<double, nx, nu>& Bd, Matrix<double, nx, 1>& hd,
    vector<double> reference_state, double ref_delta, int i, VehicleParam vehicle_model, vector<NodeStamped> track){
        // 提取参考点状态
        double r_ref = reference_state[0];
        double vy_ref = reference_state[1];
        double eyaw_ref = reference_state[2];
        double ey_ref = reference_state[3];

        // 定义线性化后连续动力学模型系统矩阵等
        VectorXd state_ref_dot(4), hc(4);
        Matrix<double, nx, nx> Ac, M12;
        Matrix<double, nx, nu> Bc;
        Matrix<double, nx, 1> state_ref;
        Matrix<double, nu, 1> control_ref;
        state_ref  << r_ref, vy_ref, eyaw_ref, ey_ref;
        control_ref << ref_delta;

        // 计算f(xref)
        double alpha_f = -(atan((vy_ref + r_ref * vehicle_model.l_f) / track[i - 1].vx) - ref_delta);
        double alpha_r = -atan((vy_ref - r_ref * vehicle_model.l_r) / track[i - 1].vx);
        double Fyf = vehicle_model.D_f*sin(vehicle_model.C_f*atan(vehicle_model.B_f*alpha_f-vehicle_model.E_f*(vehicle_model.B_f*alpha_f -atan(vehicle_model.B_f*alpha_f))));
        double Fyr = vehicle_model.D_r*sin(vehicle_model.C_r*atan(vehicle_model.B_r*alpha_r-vehicle_model.E_r*(vehicle_model.B_r*alpha_r -atan(vehicle_model.B_r*alpha_r))));
        state_ref_dot(0) = (vehicle_model.l_f * Fyf * cos(ref_delta) - vehicle_model.l_r * Fyr) / vehicle_model.I_z;
        state_ref_dot(1) = (Fyf * cos(ref_delta) + Fyr) / vehicle_model.m - track[i -1].vx * r_ref;
        state_ref_dot(2) = r_ref - track[i - 1].curvature * (track[i -1].vx * cos(eyaw_ref)- vy_ref * sin(eyaw_ref)) / (1 - ey_ref * track[i - 1].curvature);
        state_ref_dot(3) = vy_ref + track[i -1].vx * eyaw_ref;

        // 计算偏导数
        // 利用MATLAB的Jacobi函数计算得到的结果，复制过来
        // dertf / dertx
        double t2 = std::cos(ref_delta);
        double t3 = std::cos(eyaw_ref);
        double t4 = std::sin(eyaw_ref);
        double t5 = vehicle_model.l_f * r_ref;
        double t6 = vehicle_model.l_r * r_ref;
        double t8 = pow(vehicle_model.B_f, 2.0);
        double t9 = pow(vehicle_model.B_r, 2.0);
        double t10 = 1.0 /vehicle_model.I_z;
        double t11 = 1.0/vehicle_model.m;
        double t14 = -t6;
        double t15 = t5 + vy_ref;
        double t19 = pow((t6-vy_ref),2.0);
        double t17 = pow(t15,2.0);
        double t18 = t14+vy_ref;
        double t7 = ey_ref * track[i - 1].curvature;
        double t12 = 1.0 / track[i - 1].vx;
        double t13 = pow(t12,2.0);
        double t26 = t13 *t19;
        double t33 = t26+1.0;
        double t38 = 1.0 /t33;
        double t24 = t13 *t17;
        double t27 = t24+1.0;
        double t35 = 1.0 /t27;
        double t16 = t7-1.0;
        double t20 = 1.0 /t16;
        double t21 = t12 *t15;
        double t22 = atan(t21);
        double t28 = -t22;
        double t32 = ref_delta+t28;
        double t36 = pow(t32,2.0);
        double t43 = t8 *t36;
        double t46 = t43+1.0;
        double t53 = 1.0 /t46;
        double t37 = vehicle_model.B_f *t32;
        double t39 = atan(t37);
        double t41 = -t37;
        double t57 = t39+t41;
        double t58 = -vehicle_model.E_f *(t37-t39);
        double t74 = t37+t58;
        double t75 = atan(t74);
        double t77 = vehicle_model.C_f *t75;
        double t78 = cos(t77);
        double t76 = pow(t74,2.0);
        double t79 = t76+1.0;
        double t80 = 1.0 /t79;
        double t23 = -t12 *(t6-vy_ref);
        double t25 = atan(t23);
        double t29 = vehicle_model.B_r *t25;
        double t31 = atan(t29);
        double t34 = -t29;
        double t55 = t31+t34;
        double t56 = -vehicle_model.E_r *(t29-t31);
        double t64 = t29+t56;
        double t67 = atan(t64);
        double t70 = vehicle_model.C_r *t67;
        double t72 = cos(t70);
        double t69 = pow(t64,2.0);
        double t71 = t69+1.0;
        double t73 = 1.0 /t71;
        double t30 = pow(t25,2.0);
        double t40 = t9 *t30;
        double t42 = t40+1.0;
        double t47 = 1.0 /t42;
        double t49 = vehicle_model.B_r *t12 *t38;
        double t51 = vehicle_model.l_r *t49;
        double t54 = -t51;
        double t66 = t47 *t54;
        double t61 = t47 *t51;
        double t85 = t51+t66;
        double t88 = vehicle_model.E_r *t85;
        double t92 = t54+t88;
        double t52 = -t49;
        double t62 = t47 *t52;
        double t59 = t47 *t49;
        double t82 = t49+t62;
        double t86 = vehicle_model.E_r *t82;
        double t90 = t52+t86;
        double t44 = vehicle_model.B_f *t12 *t35;
        double t45 = vehicle_model.l_f *t44;
        double t50 = -t45;
        double t68 = t50 *t53;
        double t63 = t45 *t53;
        double t83 = t45+t68;
        double t87 = vehicle_model.E_f *t83;
        double t91 = t50+t87;
        double t48 = -t44;
        double t65 = t48 *t53;
        double t60 = t44 *t53;
        double t81 = t44+t65;
        double t84 = vehicle_model.E_f *t81;
        double t89 = t48+t84;

        // dertf / dertu
        double tu2 = std::cos(ref_delta);
        double tu3 = std::sin(ref_delta);
        double tu4 = vehicle_model.l_f * r_ref;
        double tu5 = pow(vehicle_model.B_f,2.0);
        double tu7 = tu4+vy_ref;
        double tu6 = 1.0 / track[i - 1].vx;
        double tu8 = tu6 *tu7;
        double tu9 = atan(tu8);
        double tu10 = -tu9;
        double tu11 = ref_delta +tu10;
        double tu12 = pow(tu11,2.0);
        double tu13 = vehicle_model.B_f *tu11;
        double tu14 = atan(tu13);
        double tu15 = -tu13;
        double tu16 = tu5 *tu12;
        double tu17 = tu16+1.0;
        double tu25 = tu14+tu15;
        double tu26 = -vehicle_model.E_f *(tu13-tu14);
        double tu18 = 1.0 /tu17;
        double tu27 = tu13+tu26;
        double tu19 = vehicle_model.B_f *tu18;
        double tu28 = atan(tu27);
        double tu29 = pow(tu27,2.0);
        double tu20 = -tu19;
        double tu30 = vehicle_model.C_f *tu28;
        double tu33 = tu29+1.0;
        double tu21 = vehicle_model.B_f+tu20;
        double tu31 = cos(tu30);
        double tu32 = sin(tu30);
        double tu34 = 1.0 /tu33;
        double tu22 = vehicle_model.E_f *tu21;
        double tu23 = -tu22;
        double tu24 = vehicle_model.B_f+tu23;

        // dertf / dertx雅可比矩阵各项,4*4矩阵
        double ax11 = -t10 *(vehicle_model.C_r *vehicle_model.D_r *vehicle_model.l_r *t72 *t73 *(t51-t88)+vehicle_model.C_f *vehicle_model.D_f *vehicle_model.l_f *t2 *t78 *t80 *(t45-t87));
        double ax12 = t10 *(vehicle_model.C_r *vehicle_model.D_r *vehicle_model.l_r *t72 *t73 *(t49-t86)-vehicle_model.C_f *vehicle_model.D_f *vehicle_model.l_f *t2 *t78 *t80 *(t44-t84));
        double ax13 = 0.0;
        double ax14 = 0.0;
        double ax21 = -track[i - 1].vx+t11 *(vehicle_model.C_r *vehicle_model.D_r *t72 *t73 *(t51-t88)-vehicle_model.C_f *vehicle_model.D_f *t2 *t78 *t80 *(t45-t87));
        double ax22 = -t11 *(vehicle_model.C_r *vehicle_model.D_r *t72 *t73 *(t49-t86)+vehicle_model.C_f *vehicle_model.D_f *t2 *t78 *t80 *(t44-t84));
        double ax23 = 0.0;
        double ax24 = 0.0;
        double ax31 = 1.0;
        double ax32 = -track[i - 1].curvature *t4 *t20;
        double ax33 = -track[i - 1].curvature *t20 *(t4 *track[i - 1].vx +t3 *vy_ref);
        double ax34 = -pow(track[i - 1].curvature, 2) *pow(t20, 2) *(t3 *track[i - 1].vx- t4 *vy_ref);
        double ax41 = 0.0;
        double ax42 = 1.0;
        double ax43 = track[i - 1].vx;
        double ax44 = 0.0;

        //dertf / dertu雅可比矩阵各项，
        double au11 = -(vehicle_model.D_f *vehicle_model.l_f *tu3 *tu32- vehicle_model.C_f *vehicle_model.D_f *vehicle_model.l_f *tu2 *tu24 *tu31 *tu34) /vehicle_model.I_z;
        double au21 = -(vehicle_model.D_f *tu3 *tu32-vehicle_model.C_f *vehicle_model.D_f *tu2 *tu24 *tu31 *tu34) /vehicle_model.m;
        double au31 = 0.0;
        double au41 = 0.0;

        Ac << ax11, ax12, ax13, ax14,
                  ax21, ax22, ax23, ax24,
                  ax31, ax32, ax33, ax34,
                  ax41, ax42, ax43, ax44;
        Bc << au11, au21, au31, au41;

        // 零阶保持器离散化
        Matrix<double,nx+nx,nx+nx> aux, M;
        aux.setZero();
        aux.block<nx,nx>(0,0) << Ac;
        aux.block<nx,nx>(0, nx) << Matrix<double,nx,nx>::Identity();
        M = (aux*Ts_).exp();
        M12 = M.block<nx,nx>(0,nx);
        hc = state_ref_dot - (Ac * state_ref + Bc * control_ref);

        Ad = (Ac * Ts_).exp();
        Bd = M12 * Bc;
        hd = M12 * hc;
}

void LateralMpcController::solveMpc(double cur_r, double cur_vy, double cur_eyaw, double cur_ey, double  cur_delta, vector<NodeStamped> track, VehicleParam vehicle_model){
    // 定义目标函数矩阵P的维度 (4个状态量[r, vy, eyaw, ey]，1个控制量，2个松弛量)
    SparseMatrix<double> HessianMatrix(4 * Np_ + 1 * Nc_ + 2 * Np_, 4 * Np_ + 1 * Nc_ + 2 * Np_);
    
    // 定义约束矩阵A的维度（4Np个动力学方程等式约束，2Nc个控制量不等式约束，2Np个状态量不等式约束）
    SparseMatrix<double> ConstraintMatrix(4 * Np_ + 2 * Nc_ + 2 * Np_, 4 * Np_ + 1 * Nc_ + 2 * Np_);

    // 定义目标函数q向量的维度
    VectorXd gradient(4 * Np_ + 1 * Nc_ + 2 * Np_);

    // 定义约束上下限制向量l,u维度
    VectorXd lower(4 * Np_ + 2 * Nc_ + 2 * Np_);
    VectorXd upper(4 * Np_ + 2 * Nc_ + 2 * Np_);

    gradient.setZero();
    lower.setZero();
    upper.setZero();

    // 定义模型线性化后系统矩阵，控制矩阵，常数项， 初值
    Matrix<double, 4, 4> Ad;
    Matrix<double, 4, 1> Bd;
    Matrix<double, 4, 1> x0, hd;
    vector<double> reference_state;
    double ref_delta;

    x0 << cur_r, cur_vy, cur_eyaw, cur_ey;

    // 初始状态约束
    for (int row = 0; row < nx; row++) {
        ConstraintMatrix.insert(row, row) = -1.0;
    }
    lower.head(nx) = -x0;
    upper.head(nx) = -x0;

    // 计算预测时域内各点线性化矩阵
    for (int i = 1; i < Np_; i++){
        // 利用上一帧的预测时域内状态构建参考点
        if (first_run_){
            reference_state = {cur_r, cur_vy, cur_eyaw, cur_ey};
            ref_delta = cur_delta;
        } else{
            if (i == Np_ - 1){
                reference_state = {QPSolution_((i - 1) * nx), QPSolution_((i - 1) * nx + 1), QPSolution_((i - 1) * nx + 2),  QPSolution_((i - 1) * nx + 3)};
            } else{
                reference_state = {QPSolution_(i * nx), QPSolution_(i * nx + 1), QPSolution_(i * nx + 2),  QPSolution_(i * nx + 3)};
            }
            if (i < Nc_ - 1){
                ref_delta = QPSolution_(Np_ * nx + i * nu);
            } else{
                ref_delta = QPSolution_(Np_ * nx + (Nc_ - 2) * nu);
            }
        }
        getLinearDynamics(Ad, Bd, hd, reference_state, ref_delta, i, vehicle_model, track);
        
        // 构建动力学方程等式约束 Adx(k) + Bd(u(k)) - x(k + 1) = - hd
        for (int row = 0; row < nx; row++){
            for (int col = 0; col < nx; col++){
                ConstraintMatrix.insert(i * nx + row, (i - 1) * nx + col) = Ad(row, col);
            }
        }

        if (i < Nc_){
            for (int row = 0; row < nx; row++){
                for (int col = 0; col < nu; col++){
                    ConstraintMatrix.insert(i * nx + row, Np_ * nx + (i - 1) * nu + col) = Bd(row, col);
                }
            }
        }else{
            for (int row = 0; row < nx; row++){
                for (int col = 0; col < nu; col++){
                    ConstraintMatrix.insert(i * nx + row, Np_ * nx + (Nc_ - 1) * nu + col) = Bd(row, col);
                }
            }
        }

        lower.segment<nx>(i * nx) = -hd;
        upper.segment<nx>(i * nx) = -hd;
        for (int row = 0; row < nx; row++) {
            ConstraintMatrix.insert(i*nx+row, i*nx+row) = -1.0;
        }
    }

    // 构建控制量不等式约束 -max_delta_ < delta < max_delta_
    for (int i = 0; i < Nc_; i++){
        ConstraintMatrix.insert(nx * Np_ + i, Np_ * nx + i * nu) = 1.0;
        lower(nx * Np_ + i) = -max_delta_;
        upper(nx * Np_ + i) = max_delta_;
    }

    // 构建控制量变化量不等式约束 -max_dert_delta_ < delta(k + 1) -  delta(k)< max_dert_delta_
    ConstraintMatrix.insert(nx * Np_ + Nc_, Np_ * nx) = 1.0;
    lower(nx * Np_ + Nc_) = cur_delta - max_dert_delta_ * Ts_;
    upper(nx * Np_ + Nc_) = cur_delta + max_dert_delta_ * Ts_;
    for (int i = 1; i < Nc_; i++){
        ConstraintMatrix.insert(nx * Np_ + Nc_ + i, Np_ * nx + i * nu) = 1.0;
        ConstraintMatrix.insert(nx * Np_ + Nc_ + i, Np_ * nx + (i - 1) * nu) = -1.0;
        lower(nx * Np_ + Nc_ + i) = - max_dert_delta_ * Ts_;
        upper(nx * Np_ + Nc_ + i) =  max_dert_delta_ * Ts_;
    }

    // 构建状态量eyaw与ey的软约束
    // -max_eyaw - s_eyaw < eyaw < max_eyaw - s_eyaw
    // -max_ey - s_ey < ey < max_ey - s_ey
    for (int i = 0; i < Np_; i++){
        ConstraintMatrix.insert(nx * Np_ + Nc_ * 2 + i * 2, i * nx + 2) = 1.0;   // eyaw
        ConstraintMatrix.insert(nx * Np_ + Nc_ * 2 + i * 2, Np_ * nx + Nc_ * nu + i * 2) = 1.0;    // eyaw的松弛项
        lower(nx * Np_ + Nc_ * 2 + i * 2) = -max_eyaw_;
        upper(nx * Np_ + Nc_ * 2 + i * 2) = max_eyaw_;

        ConstraintMatrix.insert(nx * Np_ + Nc_ * 2 + i * 2 + 1, i * nx + 3) = 1.0;   // ey
        ConstraintMatrix.insert(nx * Np_ + Nc_ * 2 + i * 2 + 1, Np_ * nx + Nc_ * nu + i * 2 + 1) = 1.0;    // ey的松弛项
        lower(nx * Np_ + Nc_ * 2 + i * 2 + 1) = -max_ey_;
        upper(nx * Np_ + Nc_ * 2 + i * 2 + 1) = max_ey_;
    }

    // 构建目标函数矩阵P
    // 状态量惩罚项
    for (int i = 0; i < Np_; i++){
        HessianMatrix.insert(i * nx, i * nx) = w_r_;
        HessianMatrix.insert(i * nx + 1, i * nx + 1) = w_vy_;
        HessianMatrix.insert(i * nx + 2, i * nx + 2) = w_eyaw_;
        HessianMatrix.insert(i * nx + 3, i * nx + 3) = w_ey_;
    }
    // 控制量惩罚项
    for (int i = 0; i < Nc_; i++){
        HessianMatrix.insert(Np_ * nx + i, Np_ * nx + i) = w_delta_;
    }
    // 松弛量惩罚项
    for (int i = 0; i < Np_; i++){
        HessianMatrix.insert(Np_ * nx + Nc_ * nu + i * 2, Np_ * nx + Nc_ * nu + i * 2) = w_s_eyaw_;
        HessianMatrix.insert(Np_ * nx + Nc_ * nu + i * 2 + 1, Np_ * nx + Nc_ * nu + i * 2 + 1) = w_s_ey_;
    }

    // 保证P矩阵对称正定
    SparseMatrix<double> sparse_I(4 * Np_ + 1 * Nc_ + 2 * Np_, 4 * Np_ + 1 * Nc_ + 2 * Np_);
    sparse_I.setIdentity();
    HessianMatrix = HessianMatrix + 1e-7 * sparse_I;

    // 构建OSQP求解器
    OsqpEigen::Solver solver;
    solver.settings()->setWarmStart(true);
    solver.data()->setNumberOfVariables(4 * Np_ + 1 * Nc_ + 2 * Np_);
    solver.data()->setNumberOfConstraints(4 * Np_ + 2 * Nc_ + 2 * Np_);

    // 优化问题代入模板
    if (!solver.data()->setHessianMatrix(HessianMatrix)) throw "fail set Hessian";
    if (!solver.data()->setGradient(gradient)){throw "fail to set gradient";}
    if (!solver.data()->setLinearConstraintsMatrix(ConstraintMatrix)) throw"fail to set constraint matrix";
    if (!solver.data()->setLowerBound(lower)){throw "fail to set lower bound";}
    if (!solver.data()->setUpperBound(upper)){throw "fail to set upper bound";}

    if (!solver.initSolver()){ cout<< "fail to initialize solver"<<endl;}
    
    // 求解优化问题
    if (!solver.solve()){
        ROS_INFO("Solve Failed !!!!");
        first_run_ = true;
        solver_status_ = false;
        return;
    }
    QPSolution_ = solver.getSolution();
    primal_value_ =  solver.getObjValue();
    solver.clearSolver();
}