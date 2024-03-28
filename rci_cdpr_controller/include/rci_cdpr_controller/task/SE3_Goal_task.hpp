#ifndef __rci_SE3_Goal_task_hpp__
#define __rci_SE3_Goal_task_hpp__

#include <iostream>
#include <cmath>
#include <fstream>
#include <vector>
#include <algorithm>
#include "rci_cdpr_controller/math/math.hpp"
// #include <pinocchio/multibody/data.hpp>
// #include <pinocchio/multibody/model.hpp>
// #include <pinocchio/spatial/fwd.hpp>
// #include <pinocchio/spatial/se3.hpp>

using namespace Eigen;
using namespace pinocchio;
using namespace std;
using namespace cdpr_controller::math;

namespace cdpr_controller{
    namespace SE3_task{
        class SE3_Goal_Task{
            public:
                SE3_Goal_Task(const std::string & Task_Name, int variables, int slacks);
                ~SE3_Goal_Task(){};

                // Setting
                void Set_Gain(Eigen::VectorXd &Kp, Eigen::VectorXd &Kd);
                void Set_Current_State(Eigen::MatrixXd Jacob_Tilda, pinocchio::SE3 & current_position, Eigen::VectorXd & current_twist);
                void Set_Goal_State(pinocchio::SE3 & target_pose);

                // Task
                Eigen::MatrixXd Get_A_Equality();
                Eigen::VectorXd Get_B_Equality();

            private:
                //Initial Setting
                int m_variables, m_slacks;  
                pinocchio::SE3 m_goal_pose;
            
                // Current State
                Eigen::MatrixXd m_Jacob_tilda;
                Eigen::VectorXd m_current_twist;
                pinocchio::SE3 m_current_pose;

                // Set Goal
                pinocchio::SE3 m_target_pose;

                //Gain Vector(6D)
                Eigen::VectorXd m_Kp,m_Kd;

                //Task
                Eigen::MatrixXd m_A_equality;
                Eigen::VectorXd m_B_equality;
                


        };
    }
}

#endif