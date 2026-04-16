#include "mpc.h"
#include <cmath>

namespace CB{

    MPC::MPC(){
        solver_initialized_ = false;
    }

    MPC::~MPC(){

    }

    bool MPC::dlqr(
        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> &A,
        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> &B,
        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> &Q,
        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> &R,
        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> &K,
        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> &P,
        int n
    ){
        // Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> P;
        P = Q;

        // 两轮差速模型
        Eigen::Matrix<double, 3, 3> AT = A.transpose();
        Eigen::Matrix<double, 2, 3> BT = B.transpose();

        for(int i=0; i<100; ++i){
            K = (B.transpose() * P * B + R).inverse() * (B.transpose() * P * A);

            Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> P_next = Q + K.transpose() * R * K + (A - B * K).transpose() * P * (A - B * K);

            P = P_next;

            // std::cout << "K: \n" << K  << std::endl;
        }

        Eigen::MatrixXd eigen_values_real = (A - B * K).eigenvalues().real();
        for (int i = 0; i < eigen_values_real.size(); i++)
        {
            if (eigen_values_real(i) > 1.001)
            {
                std::cout << " dlqr iterator failed" << std::endl;
                return false;
            }
        }
        return true;
    }

    bool MPC::run_controller(
        const std::vector<double> & param_q,
        const std::vector<double> & param_r,
        const geometry_msgs::Pose2D & robot_pose,
        const geometry_msgs::Twist & robot_twist,
        const TrajPoint & target_point, 
        const std::vector<TrajPoint> & target_traj,
        double & cmd_vel_v, 
        double & cmd_vel_w
    ){

        int n = 3; // state size
        int m = 20; // prediction step
        double dt = 1.0 / control_rate_;
        bool lqr_kpvalid = false;

        // reference point
        double xd = target_point.path_point.x;
        double yd = target_point.path_point.y;
        double yawd = target_point.path_point.yaw;
        double vd = target_point.v;
        // state error
        double ex = robot_pose.x() - target_point.path_point.x;
        double ey = robot_pose.y() - target_point.path_point.y;
        double eyaw = normalize_angle(robot_pose.theta() - target_point.path_point.yaw);
        // current state
        double v = robot_twist.linear().x();
        
        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> A;
        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> B;
        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> M;
        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> C;
        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> Q;
        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> Q_block;
        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> R;
        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> R_block;
        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> K;
        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> P;
        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> H;
        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> f;
        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> init_state_err;
        A.resize(3, 3);
        B.resize(3, 2);
        M.resize(3 * m, 3);
        C.resize(3 * m, 2 * m);
        C.setZero();
        Q.resize(3 * m, 3 * m);
        Q.setZero();
        Q_block.resize(3, 3);
        Q_block.setZero();
        R.resize(2 * m, 2 * m);
        R.setZero();
        R_block.resize(2, 2);
        R_block.setZero();
        K.resize(2, 3);
        P.resize(3, 3);
        H.resize(2 * m, 2 * m);
        f.resize(1, 2 * m);
        init_state_err.resize(3, 1);

        Q_block.diagonal() = Eigen::Map<const Eigen::Matrix<double, 3, 1>>(param_q.data());
        R_block.diagonal() = Eigen::Map<const Eigen::Matrix<double, 2, 1>>(param_r.data());
        A << 1, 0, -1.0 * dt * vd * sin(yawd),
             0, 1, dt * vd * cos(yawd),
             0, 0, 1;
        B << dt * cos(yawd), 0,
             dt * sin(yawd), 0,
             0, dt;
        // lqr_kpvalid = dlqr(A, B, Q_block, R_block, K, P, 3);
        init_state_err << ex, ey, eyaw;
        std::cout << "init_state_err: " << init_state_err.transpose() << " target yaw: " << yawd << " v: " << vd << std::endl;
        
        M.block<3, 3>(0, 0) = A;
        for (int i = 1; i < m; ++i) {
            M.block<3, 3>(3 * i, 0).noalias() = M.block<3, 3>(3 * (i - 1), 0) * A;
        }
        // std::cout << "M:\n" << M << std::endl;

        for (int i = 0; i < m; ++i) {
            C.block<3, 2>(3 * i, 2 * i) = B;
            for (int j = i + 1; j < m; ++j) {
                C.block<3, 2>(3 * j, 2 * i).noalias() = A * C.block<3, 2>(3 * (j - 1), 2 * i);
            }
        }
        // std::cout << "C:\n" << C << std::endl;

        for(int i=0; i<m-1; ++i){
            Q.block<3, 3>(i*3, i*3) = Q_block;
        }
        // Q.block<3, 3>(m*3-3, m*3-3) = P;
        Q.block<3, 3>(m*3-3, m*3-3) = Q_block;
        // std::cout << "Q:\n" << Q << std::endl;

        for(int i=0; i<m; ++i){
            R.block<2, 2>(i*2, i*2) = R_block;
        }
        // std::cout << "R:\n" << R << std::endl;

        H = C.transpose() * Q * C + R; // 二次项系数矩阵
        f = 2.0 * init_state_err.transpose() * M.transpose() * Q * C; // 线性项系数矩阵

        /*-------------------------------- solve qp-----------------------------------------*/
        OsqpEigen::Solver solver;
        solver.settings()->setVerbosity(false); // 打印输出信息
        // solver_.settings()->setWarmStart(true); // 热启动
        solver.data()->setNumberOfVariables(m * 2); // 变量个数
        solver.data()->setNumberOfConstraints(m * 2); // 约束个数
        solver.settings()->setMaxIteration(2000);  // 迭代次数限制
        solver.settings()->setAbsoluteTolerance(1e-6);  // 容差
        solver.settings()->setRelativeTolerance(1e-6);
        // solver.settings()->setRho(1e-3);  // 调整rho参数
        // solver.settings()->setAlpha(1.6);  // 调整alpha参数（超松弛）

        // loss function(软约束)
        Eigen::SparseMatrix<double> A2(2 * m, 2 * m);
        for(int i=0; i<H.rows(); ++i){
            for(int j=0; j<H.cols(); ++j){
                if(std::fabs(H(i, j) - 0.0) > 1e-5){
                    A2.insert(i, j) = H(i, j);
                }
            }
        }
        Eigen::VectorXd q = Eigen::VectorXd::Zero(2 * m);
        for(int i=0; i<2*m; ++i){
            q(i) = f(0, i);
        }
        if (!solver.data()->setHessianMatrix(A2)) {
            std::cerr << "设置 Hessian 矩阵失败!" << std::endl;
            return false;
        }
        solver.data()->setGradient(q); // 设置梯度向量 q:Zero(2 * m);
        for(int i=0; i<2*m; ++i){
            q(i) = f(0, i);
        }

        // 约束(硬约束)
        Eigen::SparseMatrix<double> B2(2*m, 2*m);
        Eigen::VectorXd lowerBound(m * 2);
        Eigen::VectorXd upperBound(m * 2);
        int m2 = 5;
        for(int i=0; i< 2*m2; i+=2){
            lowerBound(i) = min_vel_;
            upperBound(i) = max_vel_;
            lowerBound(i + 1) = min_w_;
            upperBound(i + 1) = max_w_;
        }
        for(int i=2*m2; i<2*m; i+=2){
            lowerBound(i) = vd;
            upperBound(i) = vd;
            lowerBound(i + 1) = min_w_;
            upperBound(i + 1) = max_w_;
        }
        for(int i=0; i< 2*m; ++i){
            B2.insert(i, i) = 1;
        }
        solver.data()->setLinearConstraintsMatrix(B2);
        solver.data()->setLowerBound(lowerBound);
        solver.data()->setUpperBound(upperBound);

        // 求解
        if (solver.initSolver() && solver.solve()) {
            Eigen::VectorXd solution = solver.getSolution();
            cmd_vel_v = solution(0);
            cmd_vel_w = solution(1);
            std::cout << " solution v: " << cmd_vel_v << " w: " << cmd_vel_w << std::endl;
        }
        else{
            std::cout << " osqp solve failed " << std::endl;
            return false;
        }
        return true;
    }

    bool MPC::mpc_delay_controller(
        const std::vector<double> & param_q,
        const std::vector<double> & param_r,
        const geometry_msgs::Pose2D & robot_pose,
        const geometry_msgs::Twist & robot_twist,
        const TrajPoint & target_point, 
        const std::vector<TrajPoint> & target_traj,
        double & cmd_vel_v, 
        double & cmd_vel_w
    ){

        int n = 5; // state size
        int m = 20; // prediction step
        double dt = 1.0 / control_rate_;
        bool lqr_kpvalid = false;

        // reference point
        double xd = target_point.path_point.x;
        double yd = target_point.path_point.y;
        double yawd = target_point.path_point.yaw;
        double vd = target_point.v;
        // state error
        double ex = robot_pose.x() - target_point.path_point.x;
        double ey = robot_pose.y() - target_point.path_point.y;
        double eyaw = normalize_angle(robot_pose.theta() - target_point.path_point.yaw);
        double ev = robot_twist.linear().x() - target_point.v;
        double ew = robot_twist.angular().z() - target_point.w;
        // current state
        double v = robot_twist.linear().x();

        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> A;
        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> B;
        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> M;
        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> C;
        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> Q;
        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> Q_block;
        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> R;
        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> R_block;
        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> K;
        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> P;
        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> H;
        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> f;
        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> init_state_err;
        A.resize(5, 5);
        B.resize(5, 2);
        M.resize(n * m, n);
        C.resize(n * m, 2 * m);
        C.setZero();
        Q.resize(n * m, n * m);
        Q.setZero();
        Q_block.resize(n, n);
        Q_block.setZero();
        R.resize(2 * m, 2 * m);
        R.setZero();
        R_block.resize(2, 2);
        R_block.setZero();
        // K.resize(2, 3);
        // P.resize(3, 3);
        H.resize(2 * m, 2 * m);
        f.resize(1, 2 * m);
        init_state_err.resize(5, 1);

        Q_block.diagonal() = Eigen::Map<const Eigen::Matrix<double, 5, 1>>(param_q.data());
        R_block.diagonal() = Eigen::Map<const Eigen::Matrix<double, 2, 1>>(param_r.data());
        A << 1, 0, -1.0 * dt * vd * sin(yawd), dt * cos(yawd), 0,
             0, 1, dt * vd * cos(yawd), dt * sin(yawd), 0,
             0, 0, 1, 0, dt,
             0, 0, 0, 1 - dt / tau_v_, 0,
             0, 0, 0, 0, 1 - dt / tau_w_;
        B << 0, 0,
             0, 0,
             0, 0,
             dt / tau_v_, 0,
             0, dt / tau_w_;
        // lqr_kpvalid = dlqr(A, B, Q_block, R_block, K, P, 3);
        init_state_err << ex, ey, eyaw, ev, ew;
        std::cout << "init_state_err: " << init_state_err.transpose() << " target yaw: " << yawd << " v: " << vd << std::endl;
        
        M.block<5, 5>(0, 0) = A;
        for (int i = 1; i < m; ++i) {
            M.block<5, 5>(5 * i, 0).noalias() = M.block<5, 5>(5 * (i - 1), 0) * A;
        }
        // std::cout << "M:\n" << M << std::endl;

        for (int i = 0; i < m; ++i) {
            C.block<5, 2>(5 * i, 2 * i) = B;
            for (int j = i + 1; j < m; ++j) {
                C.block<5, 2>(5 * j, 2 * i).noalias() = A * C.block<5, 2>(5 * (j - 1), 2 * i);
            }
        }
        // std::cout << "C:\n" << C << std::endl;

        for(int i=0; i<m-1; ++i){
            Q.block<5, 5>(i*5, i*5) = Q_block;
        }
        // Q.block<5, 5>(m*5-5, m*5-5) = P;
        Q.block<5, 5>(m*5-5, m*5-5) = Q_block;
        // std::cout << "Q:\n" << Q << std::endl;

        for(int i=0; i<m; ++i){
            R.block<2, 2>(i*2, i*2) = R_block;
        }
        // std::cout << "R:\n" << R << std::endl;

        H = C.transpose() * Q * C + R; // 二次项系数矩阵
        f = 2.0 * init_state_err.transpose() * M.transpose() * Q * C; // 线性项系数矩阵

        /*-------------------------------- solve qp-----------------------------------------*/
        OsqpEigen::Solver solver;
        solver.settings()->setVerbosity(false); // 打印输出信息
        // solver_.settings()->setWarmStart(true); // 热启动
        solver.data()->setNumberOfVariables(m * 2); // 变量个数
        solver.data()->setNumberOfConstraints(m * 2); // 约束个数
        solver.settings()->setMaxIteration(2000);  // 迭代次数限制
        solver.settings()->setAbsoluteTolerance(1e-6);  // 容差
        solver.settings()->setRelativeTolerance(1e-6);
        // solver.settings()->setRho(1e-3);  // 调整rho参数
        // solver.settings()->setAlpha(1.6);  // 调整alpha参数（超松弛）

        // loss function(软约束)
        Eigen::SparseMatrix<double> A2(2 * m, 2 * m);
        for(int i=0; i<H.rows(); ++i){
            for(int j=0; j<H.cols(); ++j){
                if(std::fabs(H(i, j) - 0.0) > 1e-5){
                    A2.insert(i, j) = H(i, j);
                }
            }
        }
        Eigen::VectorXd q = Eigen::VectorXd::Zero(2 * m);
        for(int i=0; i<2*m; ++i){
            q(i) = f(0, i);
        }
        if (!solver.data()->setHessianMatrix(A2)) {
            std::cerr << "设置 Hessian 矩阵失败!" << std::endl;
            return false;
        }
        solver.data()->setGradient(q); // 设置梯度向量 q:Zero(2 * m);
        for(int i=0; i<2*m; ++i){
            q(i) = f(0, i);
        }

        // 约束(硬约束)
        Eigen::SparseMatrix<double> B2(2*m, 2*m);
        Eigen::VectorXd lowerBound(m * 2);
        Eigen::VectorXd upperBound(m * 2);
        int m2 = 5;
        for(int i=0; i< 2*m2; i+=2){
            lowerBound(i) = min_vel_;
            upperBound(i) = max_vel_;
            lowerBound(i + 1) = min_w_;
            upperBound(i + 1) = max_w_;
        }
        for(int i=2*m2; i<2*m; i+=2){
            lowerBound(i) = vd;
            upperBound(i) = vd;
            lowerBound(i + 1) = min_w_;
            upperBound(i + 1) = max_w_;
        }
        for(int i=0; i< 2*m; ++i){
            B2.insert(i, i) = 1;
        }
        solver.data()->setLinearConstraintsMatrix(B2);
        solver.data()->setLowerBound(lowerBound);
        solver.data()->setUpperBound(upperBound);

        // 求解
        if (solver.initSolver() && solver.solve()) {
            Eigen::VectorXd solution = solver.getSolution();
            cmd_vel_v = solution(0);
            cmd_vel_w = solution(1);
            std::cout << " solution v: " << cmd_vel_v << " w: " << cmd_vel_w << std::endl;
        }
        else{
            std::cout << " osqp solve failed " << std::endl;
            return false;
        }

        return true;
    }
}


