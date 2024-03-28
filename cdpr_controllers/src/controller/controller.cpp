#include "cdpr_controllers/controller/controller.hpp"
#include <fstream>
#include <iostream>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <cmath>
#include <vector>

using namespace cdpr_controller::math;
using namespace cdpr_controller::trajectory;
using namespace cdpr_controller::qp_solver;
using namespace Eigen;
using namespace pinocchio;
using namespace std;
using namespace cdpr_controller::SE3_task;
using namespace cdpr_controller::platform_task;
using namespace cdpr_controller::torque_task;
using namespace cdpr_controller::arm_task;


namespace RobotController{
    HuskyFrankaWrapper::HuskyFrankaWrapper(const bool & issimulation, ros::NodeHandle & node, pinocchio::Model & model)
    : issimulation_(issimulation), n_node_(node), model_(model)
    {
        arm_joint_ = 7; platform_ = 6; cable_ = 8;

        //for publish
        m_pub.arm_acc.resize(arm_joint_);
        m_pub.arm_torque.resize(arm_joint_);
        m_pub.platform_acc.resize(platform_);
        m_pub.tension.resize(cable_);
        m_pub.wholebody_acc.resize(arm_joint_ + cable_);
        m_pub.A_matrix.setZero(13,15);
        m_pub.B_matrix.setZero(13,15);
        m_pub.torque_mode = true;


        //for plot
        for_plot.se3_pose.resize(6);
        for_plot.desired_se3_pose.resize(6);
        for_plot.franka_q.resize(arm_joint_);
        for_plot.platform_pose.resize(platform_);

        m_k = 0;

        //goal
        time_ = 0.;
        m_armjoint_goal.resize(arm_joint_);

        //gain setting(SE3)
        m_Kp_se3.resize(6);
        m_Kd_se3.resize(6);
        m_Kp_se3 << 3500,3500,3500,1000,1000,1000;
        for(int i=0; i<6; i++){
            m_Kd_se3(i) = 2*sqrt(m_Kp_se3(i));
        }

        //gain setting(joint_posture)        
        m_Kp = 100;
        m_Kd = 2;

        // For Singularity
        m_U_matrix.resize(6,6);
        m_singular = false;

    }

    void HuskyFrankaWrapper::initialize(){

        // Robot Data
        robot_ = std::make_shared<CDPR>(n_node_);        
        Data data(model_);
        m_data = data;
        nv_ = model_.nv; //nv_ = 13;
        nq_ = model_.nq; //nq_ = 14; platform orientation -> quaternion

        //Trajectory Pointer
        joint_traj_ = std::make_shared<JointCubicTrajectory>("Cubic_Trajectory", 7);
        se3_traj_ = std::make_shared<SE3CubicTrajectory>("SE3_Trajectory");

        //HQP_SOLVER
        int ns = 6;
        hqp_solver_ = std::make_shared<cdpr_controller::HQP_solver::cdpr_hqp>(arm_joint_+cable_, ns);
        robot_->getJointId(model_, joint_id_);

        //initialize
        m_q.setZero(14);
        m_v.setZero(13);
        J_.setZero(6,nq_);
        
        m_q_init.setZero(14);
        m_v_init.setZero(13);

    }
    
    //gravity compensation
    void HuskyFrankaWrapper::init_gravity_compensation(){
        robot_->platform_position(m_init_platform_pose);
    }

    void HuskyFrankaWrapper::gravity_compensation(){
        Eigen::MatrixXd W(6,8),W_inv(8,6);
        Eigen::VectorXd nle_franka(arm_joint_), nle_platform(platform_),nle_cable(cable_), pd_error(6),current_vel(arm_joint_ + platform_);
        SE3 platform_pose;

        robot_->compute_global_W(W);
        robot_->get_W_inv(W_inv,W);
        robot_->get_franka_G(m_data,nle_franka);
        robot_->get_platform_G(m_data, nle_platform);

        nle_cable = W_inv * nle_platform;
        m_pub.arm_torque = nle_franka;
        m_pub.tension = nle_cable;

        robot_ -> platform_position(platform_pose);
        robot_ -> get_all_vel(current_vel);
        pd_error = cdpr_controller::math::get_error_6d(platform_pose, m_init_platform_pose);
        m_pub.platform_acc = m_Kp_se3.cwiseProduct(pd_error) - m_Kd_se3.cwiseProduct(current_vel.head(6));

    }

    //Wholebody Control
    void HuskyFrankaWrapper::init_se3_compute(ros::Time stime, ros::Duration dur, bool & rel){
        m_pose_init = robot_->position(m_data,model_.getJointId(ee_id));
        m_q_init = m_q;

        //Setting Traj
        se3_traj_ -> SetStartTime(stime);
        se3_traj_ -> SetDuration(dur);
        se3_traj_ -> SetInitSample(m_pose_init);
        if (rel){
            m_wTep.translation() = m_pose_init.translation() + m_wTep.translation();
            m_wTep.rotation() = m_pose_init.rotation() * m_wTep.rotation();
        }
        se3_traj_ -> SetGoalSample(m_wTep);

        //Setting Initial Condition
        robot_->platform_position(m_init_platform_pose);
        m_duration = dur;
        m_stime = stime;
        end_task_ = false;
        m_pub.torque_mode = true;
        m_singular = false;

        // Task Pointer - Platform
        platform_static_task_ = std::make_shared<Platform_Static_Task>("Platform_Static_Task", m_init_platform_pose, m_pub.torque_mode, arm_joint_ + cable_, 6);
        platform_parallel_task_ = std::make_shared<Platform_Parallel_Task>("Platform_Parallel_Task", m_init_platform_pose, m_pub.torque_mode, arm_joint_ + cable_, 6);

        // Task Pointer - Wholebody
        se3_goal_task_ = std::make_shared<SE3_Goal_Task>("SE3_Goal_Task", arm_joint_ + cable_, 6);

        // Task_Pointer - Varialbe - Torque
        simple_torque_task_ = std::make_shared<SimpleTorqueTask>("Test_Simple_Torque", arm_joint_ + cable_, 6);

        // Task Pointer - Minimize Task
        torque_min_task_ = std::make_shared<Torque_Minimize_Task>("Torque_Minimize_Task",arm_joint_+cable_,6);
        
        // Task Pointer - Joint Posture Task
        joint_posture_task_ = std::make_shared<Joint_Posture_Task>("Joint_Posture_Task", arm_joint_ + cable_, arm_joint_);
        
        // Task Pointer - Goal + Singularity Avoidance Task
        SE3_Singularity_Avoidance_task_ = std::make_shared<Singularity_Avoidance_Task>("SE3_Singularity_Avoid_Task", arm_joint_ + cable_);

        hqp_solver_->Remove_Task();

    }

    void HuskyFrankaWrapper::wholebody_compute(ros::Time ctime)
    {
        // Init
        m_singular = false;

        // Declare Property
        Eigen::MatrixXd A_matrix(platform_ + arm_joint_,cable_ + arm_joint_), W(platform_ ,cable_),B_matrix(platform_ + arm_joint_,cable_ + arm_joint_), platform_inv_J(platform_, cable_), M_tilda(cable_ + arm_joint_,cable_ + arm_joint_), mass(platform_ + arm_joint_,platform_ + arm_joint_);
        SE3 platform_pose;
        Eigen::VectorXd joint_vel(platform_ + arm_joint_), G_tilda(cable_ + arm_joint_), gravity(platform_ + arm_joint_);

        //Compute Property
        robot_->jointJacobianWorld(m_data, model_, joint_id_, J_);
        robot_->compute_global_W(W);
        robot_->get_platform_inv_J(platform_inv_J, W);
        robot_->get_A(A_matrix, platform_inv_J);
        robot_->get_All_M(m_data, mass);
        robot_->get_wholebody_G(m_data, gravity);
        B_matrix.setZero();
        B_matrix.topLeftCorner(6,8) = W;
        B_matrix.bottomRightCorner(7,7).setIdentity();
        M_tilda = pseudoinv(B_matrix) * mass * A_matrix;
        G_tilda = pseudoinv(B_matrix) * gravity;

        // Singular Value Decomposition
        JacobiSVD<MatrixXd> SVD_J_arm(J_.topRightCorner(6,arm_joint_),ComputeThinU | ComputeThinV);
        m_U_matrix = SVD_J_arm.matrixU();
        m_singular_value = SVD_J_arm.singularValues();

        //current vel
        Eigen::VectorXd v_current(6);
        robot_ -> get_all_vel(joint_vel);
        robot_ -> getVelvec(m_data,joint_id_,v_current);
        v_current.head(3) = m_pose_current.rotation() * v_current.head(3);
        v_current.tail(3) = m_pose_current.rotation() * v_current.tail(3);

        //get trajectory
        se3_traj_ -> SetCurrentTime(ctime);
        m_pose_cubic = se3_traj_ -> computeNext();

        //get pose error
        m_pose_current = robot_->position(m_data,model_.getJointId(ee_id));
        robot_ -> platform_position(platform_pose);
        Eigen::VectorXd err = cdpr_controller::math::get_error_6d(m_pose_current,m_pose_cubic);

        // Declare Task
        Eigen::MatrixXd SE3_Kinematic_goal_A(6,cable_ + arm_joint_ + 6), Platform_Static_A_Task(6,cable_ + arm_joint_ + 6), Platform_Parallel_A_Task(6,cable_ + arm_joint_ + 6), Kinematic_torque_min_Q(cable_ + arm_joint_, cable_ + arm_joint_), SE3_Dynamic_goal_A(6,cable_ + arm_joint_ + 6), Min_Arm_Posture_A_Task(arm_joint_, arm_joint_ + cable_ + arm_joint_), Singularity_Avoidance_Task_A_1, Singularity_Avoidance_Task_A_2;
        Eigen::VectorXd SE3_Kinematic_goal_B(6), Platform_Static_B_Task(6), Platform_Parallel_B_Task(6), Kinematic_torque_min_C(cable_ + arm_joint_), SE3_Dynamic_goal_B(6), Min_Arm_Posture_B_Task(arm_joint_), Singularity_Avoidance_Task_B_1, Singularity_Avoidance_Task_B_2;

        //SE3 Goal Task
        se3_goal_task_ -> Set_Gain(m_Kp_se3, m_Kd_se3);
        se3_goal_task_ -> Set_Current_State(J_ * A_matrix, m_pose_current, v_current);
        se3_goal_task_ -> Set_Goal_State(m_pose_cubic);
        SE3_Kinematic_goal_A = se3_goal_task_ -> Get_A_Equality();
        SE3_Kinematic_goal_B = se3_goal_task_ -> Get_B_Equality();
    
        //Platform Static Task
        platform_static_task_ -> Set_Gain(m_Kp_se3, m_Kd_se3);
        platform_static_task_ -> Set_Current_State(M_tilda, W.transpose(), platform_pose, joint_vel.head(6));
        Platform_Static_A_Task = platform_static_task_ -> Get_A_Equality();
        Platform_Static_B_Task = platform_static_task_ -> Get_B_Equality();

        //Platform Parallel Task 
        platform_parallel_task_ -> Set_Gain(m_Kp_se3, m_Kd_se3);
        platform_parallel_task_ -> Set_Current_State(M_tilda, W.transpose(), platform_pose, joint_vel.head(6));
        Platform_Parallel_A_Task = platform_parallel_task_ -> Get_A_Equality();
        Platform_Parallel_B_Task = platform_parallel_task_ -> Get_B_Equality();

        // Simple Torque Task
        simple_torque_task_ -> Set_Gain(m_Kp_se3, m_Kd_se3);
        simple_torque_task_ -> Set_Current_State(M_tilda, J_ * A_matrix, m_pose_current, v_current);
        simple_torque_task_ -> Set_Goal_State(m_pose_cubic);
        SE3_Dynamic_goal_A = simple_torque_task_ -> Get_A_Equality();
        SE3_Dynamic_goal_B = simple_torque_task_ -> Get_B_Equality();

        // Joint Posture Task
        joint_posture_task_ -> Set_Gain(1000, sqrt(1000));
        joint_posture_task_ -> Set_Current_State(M_tilda, m_q.tail(7), m_v.tail(7));
        joint_posture_task_ -> Set_Goal_State(m_q.tail(7));
        Min_Arm_Posture_A_Task = joint_posture_task_ -> Get_A_Equality();
        Min_Arm_Posture_B_Task = joint_posture_task_ -> Get_B_Equality();

        // Test Singularity Aovid Task
        SE3_Singularity_Avoidance_task_ -> Set_Gain(m_Kp_se3, m_Kd_se3);
        SE3_Singularity_Avoidance_task_ -> Set_Current_State(J_ * A_matrix, M_tilda, m_U_matrix, m_pose_current, v_current);
        SE3_Singularity_Avoidance_task_ -> Set_Singularity(m_singular_value);
        SE3_Singularity_Avoidance_task_ -> Set_Goal_State(m_pose_cubic);
        Singularity_Avoidance_Task_A_1 =  SE3_Singularity_Avoidance_task_ -> Get_Non_Singular_Task_A();
        Singularity_Avoidance_Task_B_1 =  SE3_Singularity_Avoidance_task_ -> Get_Non_Singular_Task_B();
        Singularity_Avoidance_Task_A_2 =  SE3_Singularity_Avoidance_task_ -> Get_Singular_Task_A();
        Singularity_Avoidance_Task_B_2 =  SE3_Singularity_Avoidance_task_ -> Get_Singular_Task_B(); 
        
        
        // minimize itself Task
        Eigen::MatrixXd Q_Min_variabel_norm(15,15);
        Eigen::VectorXd C_Min_variabel_norm(15);
        Q_Min_variabel_norm.setZero();
        Q_Min_variabel_norm.setIdentity();
        C_Min_variabel_norm.setZero();

        //Torque minimze Task
        torque_min_task_ -> Set_Current_State(M_tilda, G_tilda);
        Kinematic_torque_min_Q = torque_min_task_ -> Get_Q();
        Kinematic_torque_min_C = torque_min_task_ -> Get_C();

        // Test manip Task
        // Eigen::MatrixXd Q_m_test(15,15);
        // Q_m_test.setIdentity();
        // Q_m_test *= 1e-5;
        // Eigen::VectorXd C_m_test(15);
        // C_m_test.setZero();
        // C_m_test.tail(7) = -getJacobM(model_, m_q, m_v, "sdf", arm_joint_);

        //Test - 나중에 Inequality 태스크 짤 때 알아서 계산되게 만들기
        // hqp_solver_->get_tilda(M_tilda,G_tilda);
        // hqp_solver_->Set_Current_State(m_q.tail(7),m_v.tail(7));
        
        if(!end_task_){
            if(!m_pub.torque_mode){
                hqp_solver_ -> Add_Task(SE3_Kinematic_goal_A,SE3_Kinematic_goal_B, 1);
                hqp_solver_ -> Add_Task(Platform_Static_A_Task, Platform_Static_B_Task, 2);
            }
            else{
                hqp_solver_ -> Add_Task(Singularity_Avoidance_Task_A_1, Singularity_Avoidance_Task_B_1, 1);
                hqp_solver_ -> Add_Task(Singularity_Avoidance_Task_A_2, Singularity_Avoidance_Task_B_2, 2);
                hqp_solver_ -> Add_Task(Platform_Static_A_Task, Platform_Static_B_Task,3);
                hqp_solver_ -> Add_Min_Task(Q_Min_variabel_norm, C_Min_variabel_norm, 4);

            }
            end_task_ = true;
        }
        
        else{
            if(!m_pub.torque_mode){
                hqp_solver_ -> Update_Task(SE3_Kinematic_goal_A,SE3_Kinematic_goal_B,1);
                hqp_solver_ -> Update_Task(Platform_Static_A_Task, Platform_Static_B_Task, 2);
            }
            else{
                hqp_solver_ -> Update_Task(Singularity_Avoidance_Task_A_1, Singularity_Avoidance_Task_B_1, 1);
                hqp_solver_ -> Update_Task(Singularity_Avoidance_Task_A_2, Singularity_Avoidance_Task_B_2, 2);
                hqp_solver_ -> Update_Task(Platform_Static_A_Task, Platform_Static_B_Task, 3);
                hqp_solver_ -> Update_Min_Task(Q_Min_variabel_norm, C_Min_variabel_norm, 4);
            }
        }


        // Publish
        if(!m_pub.torque_mode)
        {
            m_pub.tension = (pseudoinv(B_matrix) * (mass * A_matrix * hqp_solver_ ->solve() + gravity)).head(8);
            m_pub.arm_torque = (pseudoinv(B_matrix) * (mass * A_matrix * hqp_solver_ ->solve() + gravity)).tail(7);
        }
        
        else{
            m_pub.tension = (hqp_solver_ -> solve() + G_tilda).head(8);
            m_pub.arm_torque = (hqp_solver_ -> solve() + G_tilda).tail(7);
        }

        //for plot
        for_plot.se3_pose.head(3) = m_pose_current.translation();
        for_plot.se3_pose.tail(3) = robot_->rotationMatrixToEulerAngles(m_pose_current.rotation());

        for(int i=0; i<arm_joint_; i++){
            for_plot.franka_q(i) = m_q(i+platform_);
        }

        for(int i=0; i<arm_joint_; i++){
            for_plot.platform_pose(i) = m_q(i);
        }
        
        for_plot.desired_se3_pose.head(3) = m_pose_cubic.translation();
        for_plot.desired_se3_pose.tail(3) = robot_->rotationMatrixToEulerAngles(m_pose_cubic.rotation());

        HuskyFrankaWrapper::pose_plot();

    }


    void HuskyFrankaWrapper::platform_se3_compute(ros::Time ctime)
    {
        Eigen::MatrixXd A_matrix(13,15),W(6,8),B_matrix(13,15),platform_inv_J(6,8);
        B_matrix.setZero();
        SE3 platform_pose;
        Eigen::VectorXd joint_vel(platform_ + arm_joint_);

        robot_->jointJacobianWorld(m_data, model_, joint_id_, J_);
        robot_->compute_global_W(W);
        robot_->get_platform_inv_J(platform_inv_J,W);
        robot_->get_A(A_matrix,platform_inv_J);
        B_matrix.topLeftCorner(6,8) = W;
        B_matrix.bottomRightCorner(7,7).setIdentity();

        //get trajectory
        se3_traj_ -> SetCurrentTime(ctime);
        m_pose_cubic = se3_traj_ -> computeNext();

        //get pose error
        m_pose_current = robot_->position(m_data,model_.getJointId(ee_id));
        robot_ -> platform_position(platform_pose);
        Eigen::VectorXd err = cdpr_controller::math::get_error_6d(m_pose_current,m_pose_cubic);

        //current vel
        Eigen::VectorXd v_current(6);
        robot_ -> get_all_vel(joint_vel);
        robot_ -> getVelvec(m_data,joint_id_,v_current);
        v_current.head(3) = m_pose_current.rotation() * v_current.head(3);
        v_current.tail(3) = m_pose_current.rotation() * v_current.tail(3);

        Eigen::VectorXd pd_control(3);
        pd_control = (m_Kp_se3.cwiseProduct(cdpr_controller::math::get_error_6d(platform_pose,m_init_platform_pose))- m_Kd_se3.cwiseProduct(joint_vel.head(6))).tail(3);
        
        cdpr_qp_solver.qp_setting(err ,v_current, J_, platform_inv_J, A_matrix, pd_control);
        cdpr_qp_solver.gain_setting(m_Kp_se3,m_Kd_se3);

        m_pub.wholebody_acc = cdpr_qp_solver.solve();
        m_pub.A_matrix = A_matrix;
        m_pub.B_matrix = B_matrix;

    }

    void HuskyFrankaWrapper::arm_se3_compute(ros::Time ctime)
    {
        Eigen::MatrixXd A_matrix(13,15),W(6,8),B_matrix(13,15),platform_inv_J(6,8);
        B_matrix.setZero();
        SE3 platform_pose;
        Eigen::VectorXd joint_vel(platform_ + arm_joint_);

        robot_->jointJacobianWorld(m_data, model_, joint_id_, J_);
        robot_->compute_global_W(W);
        robot_->get_platform_inv_J(platform_inv_J,W);
        robot_->get_A(A_matrix,platform_inv_J);
        B_matrix.topLeftCorner(6,8) = W;
        B_matrix.bottomRightCorner(7,7).setIdentity();

        //get trajectory
        se3_traj_ -> SetCurrentTime(ctime);
        m_pose_cubic = se3_traj_ -> computeNext();

        //get pose error
        m_pose_current = robot_->position(m_data,model_.getJointId(ee_id));
        robot_ -> platform_position(platform_pose);
        Eigen::VectorXd err = cdpr_controller::math::get_error_6d(m_pose_current,m_pose_cubic);

        //current vel
        Eigen::VectorXd v_current(6);
        robot_ -> get_all_vel(joint_vel);
        robot_ -> getVelvec(m_data,joint_id_,v_current);
        v_current.head(3) = m_pose_current.rotation() * v_current.head(3);
        v_current.tail(3) = m_pose_current.rotation() * v_current.tail(3);

        Eigen::VectorXd pd_control(3);
        pd_control = (m_Kp_se3.cwiseProduct(cdpr_controller::math::get_error_6d(platform_pose,m_init_platform_pose))- m_Kd_se3.cwiseProduct(joint_vel.head(6))).tail(3);
        
        cdpr_qp_solver.qp_setting(err ,v_current, J_, platform_inv_J, A_matrix, pd_control);
        cdpr_qp_solver.gain_setting(m_Kp_se3,m_Kd_se3);

        m_pub.wholebody_acc = cdpr_qp_solver.solve();
        m_pub.A_matrix = A_matrix;
        m_pub.B_matrix = B_matrix;

    }

    void HuskyFrankaWrapper::init_arm_posture_compute(ros::Time stime, ros::Duration duration){
        //Joint Posture Trajectory
        joint_traj_->SetStartTime(stime);
        joint_traj_->SetDuration(duration);
        joint_traj_->SetInitSample(m_q.tail(arm_joint_));
        joint_traj_->SetGoalSample(m_armjoint_goal);

        //Platform Static Control
        robot_->platform_position(m_init_platform_pose);

    }

    void HuskyFrankaWrapper::arm_posture_compute(ros::Time ctime){
        Eigen::VectorXd joint_cubic(arm_joint_), pd_error(6), current_vel(arm_joint_ + platform_), gravity(platform_+arm_joint_);
        Eigen::MatrixXd W(6,8), B_matrix(13,15), A_matrix(13,15), platform_inv_J(6,8), mass(arm_joint_ + cable_,arm_joint_ + cable_);
        SE3 platform_pose;

        robot_->compute_global_W(W);
        robot_->get_platform_inv_J(platform_inv_J,W);
        robot_->get_A(A_matrix,platform_inv_J);
        robot_->get_All_M(m_data, mass);
        robot_->get_wholebody_G(m_data,gravity);

        B_matrix.topLeftCorner(6,8) = W;
        B_matrix.bottomRightCorner(7,7).setIdentity();


        joint_traj_->SetCurrentTime(ctime);
        joint_cubic = joint_traj_->computeNext();
        
        //Arm Control
        m_pub.arm_acc = (m_Kp * (joint_cubic - m_q.tail(arm_joint_)) - m_Kd * m_v.tail(arm_joint_));

        //Platform Static Control
        robot_ -> platform_position(platform_pose);
        robot_ -> get_all_vel(current_vel);
        pd_error = cdpr_controller::math::get_error_6d(platform_pose, m_init_platform_pose);
        m_pub.platform_acc = m_Kp_se3.cwiseProduct(pd_error) - m_Kd_se3.cwiseProduct(current_vel.head(6));

        //Get Wholebody Acc
        m_pub.wholebody_acc.head(8) = W.transpose() * m_pub.platform_acc;
        m_pub.wholebody_acc.tail(7) = m_pub.arm_acc;
        m_pub.tension = (pseudoinv(B_matrix) * (mass * A_matrix * m_pub.wholebody_acc + gravity)).head(8);
        m_pub.arm_torque = (pseudoinv(B_matrix) * (mass * A_matrix * m_pub.wholebody_acc + gravity)).tail(7);
    }

    void HuskyFrankaWrapper::joint_update(const Eigen::VectorXd& q, const Eigen::VectorXd& v){        
        m_q = q;
        m_v = v;
    }

    void HuskyFrankaWrapper::compute_all_terms(){
        pinocchio::computeAllTerms(model_, m_data, m_q, m_v);
    }

    void HuskyFrankaWrapper::end_signal(){
        end_task_ = true;
    }

    void HuskyFrankaWrapper::pose_plot(){
        std::string filepath = "/home/chan/rci_cdpr_ws/src/cdpr_controllers/txt/se3_pose.txt";
        std::string filepath_for_des = "/home/chan/rci_cdpr_ws/src/cdpr_controllers/txt/desired_se3_pose.txt";
        // std::string filepath_input = "/home/chan/rci_cdpr_ws/src/cdpr_controllers/txt/task_3_manip_.txt";
        ofstream fout(filepath,std::ios::app);
        for(int i =0; i<6; i++){
            fout << for_plot.se3_pose(i) << '\t';
            }
        fout << "\n";
        fout.close();

        ofstream fout_desired(filepath_for_des,std::ios::app);
        for(int i =0; i<6; i++){
                fout_desired<< for_plot.desired_se3_pose(i) << "\t";
        }
        fout_desired << "\n";
        fout_desired.close();

        // ofstream fout_input(filepath_input,std::ios::app);
        // fout_input<<m_k<<"\n";
        // fout_input.close();


        // std::string filepath = "/home/chan/rci_cdpr_ws/src/cdpr_controllers/txt/franka_q_real.txt";
        // ofstream fout(filepath,std::ios::app);
        // for(int i =0; i<7; i++){
        //     fout << for_plot.franka_q(i) << '\t';
        // }
        // fout << "\n";
        // fout.close();
    }

}