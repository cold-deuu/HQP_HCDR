#ifndef __husky_franka_ctrl__
#define __husky_franka_ctrl__

//Pinocchio Header
#include <pinocchio/fwd.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp> 
#include <Eigen/Dense>
#include <Eigen/QR>

//ROS Header
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Float32.h"
#include "sensor_msgs/JointState.h"
#include "geometry_msgs/Transform.h"
#include "geometry_msgs/Pose2D.h"
#include "std_msgs/Int16.h"
#include "tf/transform_datatypes.h"

//CDPR Controller
#include <cdpr/cdpr.h>
#include <rci_cdpr_controller/trajectory/joint_cubic.hpp>
#include <rci_cdpr_controller/trajectory/SE3_cubic.hpp>
#include <rci_cdpr_controller/math/math.hpp>
#include <rci_cdpr_controller/solver/qp_solver_mm.hpp>
#include <rci_cdpr_controller/solver/HQP_solver.hpp>
#include <rci_cdpr_controller/task/platform_static_task.hpp>
#include <rci_cdpr_controller/task/platform_parallel_task.hpp>
#include <rci_cdpr_controller/task/SE3_Goal_task.hpp>
#include <rci_cdpr_controller/task/Torque_minimize_task.hpp>
#include <rci_cdpr_controller/task/Simple_Torque_Task.hpp>
#include <rci_cdpr_controller/task/JointLimitAvoidanceTask.hpp>
#include <rci_cdpr_controller/task/Joint_Posture_Task.hpp>
#include <rci_cdpr_controller/task/Singularity_Avoidance_Task.hpp>



using namespace std;
using namespace Eigen;
using namespace pinocchio;
using namespace cdpr_controller::math;
using namespace cdpr_controller::trajectory;
using namespace cdpr_controller::qp_solver;
using namespace cdpr_controller::SE3_task;
using namespace cdpr_controller::platform_task;
using namespace cdpr_controller::torque_task;
using namespace cdpr_controller::joint_limit_task;
using namespace cdpr_controller::arm_task;


typedef struct Pub_state {   
    Eigen::VectorXd arm_acc;
    Eigen::VectorXd platform_acc;
    Eigen::VectorXd wholebody_acc;
    Eigen::VectorXd tension;
    Eigen::VectorXd arm_torque;
    Eigen::MatrixXd A_matrix;
    Eigen::MatrixXd B_matrix;
    bool torque_mode;
    bool wholebody;
    double pose_err;
} pub_state;   

typedef struct Plot_state{
    Eigen::VectorXd se3_pose;
    Eigen::VectorXd franka_q;
    Eigen::VectorXd desired_se3_pose;
    Eigen::VectorXd platform_pose;
} plot_state;


namespace RobotController{
    class HuskyFrankaWrapper{
        public: 
            HuskyFrankaWrapper(const bool & issimulation, ros::NodeHandle & node, pinocchio::Model & model);
            ~HuskyFrankaWrapper(){};

            void initialize();
            void compute_all_terms();

            // Gravity Compensation
            void init_gravity_compensation();
            void gravity_compensation();

            // Get Arm Joint goal
            void init_arm_posture_compute(ros::Time stime, ros::Duration duration);
            void arm_posture_compute(ros::Time ctime);

            // Get SE3 goal
            void init_se3_compute(ros::Time stime, ros::Duration dur, bool & rel);
            void wholebody_compute(ros::Time ctime);
            void platform_se3_compute(ros::Time ctime);
            void arm_se3_compute(ros::Time ctime);

            // get target
            void get_se3_task(SE3 &goal){
                m_wTep = goal;
            }
            void get_joint_posture_goal(VectorXd &goal){m_armjoint_goal = goal;}

            // For plot
            void pose_plot();

            void end_signal();
            void joint_update(const Eigen::VectorXd& q, const Eigen::VectorXd& v);
            bool simulation(){
                return issimulation_;
            }
            Pub_state & get_pub(){
                return m_pub;
            }

        private:
            bool issimulation_, mode_change_, franka_gripper_;
            std::string robot_node_;
            
            int arm_joint_, platform_, cable_;

            double m_k;

            Pub_state m_pub;
            Plot_state for_plot;

            double time_;
            ros::Time stime_;
            int array_cnt_;
            int na_, nv_, nq_;
            bool isfinished_;

            std::shared_ptr<CDPR> robot_;
            std::shared_ptr<JointCubicTrajectory> joint_traj_;
            std::shared_ptr<SE3CubicTrajectory> se3_traj_;

            std::shared_ptr<Platform_Static_Task> platform_static_task_;
            std::shared_ptr<Platform_Parallel_Task> platform_parallel_task_;
            std::shared_ptr<SE3_Goal_Task> se3_goal_task_;
            std::shared_ptr<Torque_Minimize_Task> torque_min_task_;
            std::shared_ptr<SimpleTorqueTask> simple_torque_task_;
            std::shared_ptr<JointLimit_Avoidance_Task> joint_limit_avoidance_task_;
            std::shared_ptr<Joint_Posture_Task> joint_posture_task_;
            std::shared_ptr<Singularity_Avoidance_Task> SE3_Singularity_Avoidance_task_;

            
            std::shared_ptr<cdpr_controller::HQP_solver::cdpr_hqp> hqp_solver_; 


            pinocchio::Model model_;
            pinocchio::Data m_data;


            //solver
            cdpr_controller::qp_solver::cdpr_hqp cdpr_qp_solver;
            
            //init
            Eigen::VectorXd m_q_init,m_v_init;

            //goal
            SE3 m_wTep;
            Eigen::VectorXd m_armjoint_goal;
            ros::Duration m_dur;

            //Constraint
            Eigen::MatrixXd m_qlim;
            Eigen::VectorXd m_qdlim;

            //State
            Eigen::VectorXd m_q,m_v;
            Eigen::MatrixXd J_;
            Eigen::VectorXd m_v_current;
            SE3 m_pose_current;

            //Model
            pinocchio::Model::Index joint_id_;


            //for compute
            SE3 m_pose_cubic,m_pose_init, m_init_platform_pose;
            MatrixXd m_J_qp;
            VectorXd m_q_qp, m_base_q;
            
            //gain
            double m_Kp,m_Kd;
            Eigen::VectorXd m_Kp_se3, m_Kd_se3;

            //Sim time
            ros::Time m_stime;
            ros::Duration m_duration;

            bool end_task_ = false;
            string ee_id = "panda_joint7";
            ros::NodeHandle n_node_;


            // Singularity Avoidance
            Eigen::MatrixXd m_U_matrix;
            Eigen::VectorXd m_singular_value;
            bool m_singular;
            
    };
}
#endif


