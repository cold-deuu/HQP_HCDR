#include "cdpr_controllers/simulator/simulator.hpp"
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include "pinocchio/fwd.hpp"
#include <iostream>
#include <cmath>

using namespace std;
using namespace pinocchio;
using namespace Eigen;

int main(int argc, char ** argv){
    ros::init(argc, argv, "cdpr_control");
    ros::NodeHandle nh;

    // Robot Wrapper
    robot_ = std::make_shared<CDPR>(nh);        

    std::string urdf_filename = "/home/chan/rci_cdpr_ws/src/cdpr/sdf/uppanda.urdf";
    pinocchio::Model model;
    pinocchio::urdf::buildModel(urdf_filename, JointModelFreeFlyer(),model);
    pinocchio::Model::Index joint_id;
    pinocchio::Data data(model);
    
    int nq = model.nq;
    int nv = model.nv;

    q_.setZero(14); v_.setZero(13);
    Eigen::VectorXd franka_torque(7),tension(8), wholebody_acc(15), nonlineareffect(13);
    Eigen::MatrixXd mass(13,13), A_matrix(13,15), B_matrix(13,15), platform_W(6,8), platform_W_inv(8,6);
    ros::Rate loop_rate(1000);



    //controller
    ctrl_ = std::make_shared<RobotController::HuskyFrankaWrapper>(true, nh, model);
    ctrl_->initialize();

    wholebody_action_server_ = std::make_unique<SE3ActionServer>("cdpr_controllers/wholebody_control",nh,ctrl_);
    joint_posture_action_server_ = std::make_unique<JointPostureActionServer>("cdpr_controllers/arm_joint_control",nh,ctrl_);

    int i = 0;
    while(ros::ok()){
        pinocchio::computeAllTerms(model,data,q_,v_);
        if(robot_->ok()){
            get_joint_state();
            ctrl_->joint_update(q_,v_);
            ctrl_->compute_all_terms();
            
            robot_->get_All_M(data, mass);
            robot_->get_wholebody_G(data,nonlineareffect);
            robot_->compute_global_W(platform_W);
            robot_->get_W_inv(platform_W_inv, platform_W);

            wholebody_action_server_->compute(ros::Time::now());
            joint_posture_action_server_->compute(ros::Time::now());

                
            if (!wholebody_action_server_->isrunning() && !joint_posture_action_server_->isrunning())
            {
                if(i==0)
                    ctrl_->init_gravity_compensation();
                ctrl_->gravity_compensation();
                franka_torque = ctrl_->get_pub().arm_torque;
                tension = platform_W_inv * mass.block<6,6>(0,0) * ctrl_->get_pub().platform_acc + ctrl_->get_pub().tension;
                i+=1;
            }
            
            else{
                i=0;
                tension = ctrl_->get_pub().tension;
                franka_torque = ctrl_->get_pub().arm_torque;
            }

            robot_->sendTensions(tension);
            robot_->send_torque(franka_torque);     
        }
        
        ros::spinOnce();
        loop_rate.sleep();
    }

}

void get_joint_state()
{
    robot_->get_all_pose(q_);
    robot_->get_all_vel(v_);
}
