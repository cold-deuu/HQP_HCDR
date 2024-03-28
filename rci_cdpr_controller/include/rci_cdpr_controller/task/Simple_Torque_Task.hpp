#ifndef __rci_SE3_Simple_Torque_task_hpp__
#define __rci_SE3_Simple_Torque_task_hpp__

#include <iostream>
#include <cmath>
#include <fstream>
#include <vector>
#include <algorithm>
#include "rci_cdpr_controller/math/math.hpp"

namespace cdpr_controller
{
    namespace torque_task{
        class SimpleTorqueTask{
            public:
                SimpleTorqueTask(const std::string & task_name, int variables, int slacks);
                ~SimpleTorqueTask(){};

                void Set_Gain(Eigen::VectorXd &Kp, Eigen::VectorXd &Kd);
                void Set_Current_State(Eigen::MatrixXd & mass_tilda, Eigen::MatrixXd jacobian, pinocchio::SE3 & current_position, Eigen::VectorXd current_twist);
                void Set_Goal_State(pinocchio::SE3 & target_pose);
                Eigen::MatrixXd Get_A_Equality();
                Eigen::VectorXd Get_B_Equality();




            private:

                // Initialize
                int m_variables, m_slacks;


                // Current States
                Eigen::MatrixXd m_jacobian, m_mass_tilda;
                Eigen::VectorXd m_current_twist;
                pinocchio::SE3 m_current_pose;


                // Goal States
                pinocchio::SE3 m_target_pose;

                //Gain
                Eigen::VectorXd m_Kp, m_Kd;

                //Task 
                Eigen::MatrixXd m_Task_A_Eq;
                Eigen::VectorXd m_Task_B_Eq;

                //Singularity
                bool m_singularity;
                Eigen::MatrixXd m_U_matrix;

        
        };
    }
}






#endif