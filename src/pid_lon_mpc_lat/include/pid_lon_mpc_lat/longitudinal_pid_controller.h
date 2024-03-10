using namespace std;

/*
    ** 类名称：LongitudinalPidController

    ** 简介：
    本类用于根据车速跟踪偏差计算轮上驱动力矩，采用增量式PID控制，计算量小
*/
class LongitudinalPidController{
    public:
        LongitudinalPidController();
        double solveCmd(double error_v);
    private:
        double pid_kp_ = 3500.0;
        double pid_ki_ = 500.0;
        double pid_kd_ = 0.0;
        double last_error_ = 0.0;
        double lastlast_error_ = 0.0;
};