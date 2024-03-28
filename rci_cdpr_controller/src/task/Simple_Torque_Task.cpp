#include "rci_cdpr_controller/task/Simple_Torque_Task.hpp"


using namespace Eigen;
using namespace pinocchio;
using namespace std;
using namespace cdpr_controller::math;

namespace cdpr_controller{
    namespace torque_task{
        SimpleTorqueTask::SimpleTorqueTask(const std::string & task_name, int variables, int slacks)
        : m_variables(variables), m_slacks(slacks)
        {
            m_Kp.resize(6);
            m_Kd.resize(6);
            m_current_twist.resize(6);

            m_mass_tilda.setZero(m_variables, m_variables);
            m_jacobian.setZero(6,m_variables);
            m_current_twist.setZero(6);
            



            m_Task_A_Eq.setZero(6,m_variables + m_slacks);
            m_Task_B_Eq.setZero(6);

            m_U_matrix.resize(6,6);
            
        }

        void SimpleTorqueTask::Set_Gain(Eigen::VectorXd & Kp, Eigen::VectorXd & Kd)
        {
            m_Kp = Kp;
            m_Kd = Kd;
        }

        void SimpleTorqueTask::Set_Current_State(Eigen::MatrixXd & mass_tilda, Eigen::MatrixXd jacobian, pinocchio::SE3 & current_position, Eigen::VectorXd current_twist)
        {
            m_mass_tilda = mass_tilda;
            m_jacobian = jacobian;
            m_current_twist = current_twist;
            m_current_pose = current_position;

        }


        void SimpleTorqueTask::Set_Goal_State(pinocchio::SE3 & target_pose)
        {
            m_target_pose = target_pose;
        }

        Eigen::MatrixXd SimpleTorqueTask::Get_A_Equality()
        {  
            m_Task_A_Eq.setZero();
            m_Task_A_Eq.topLeftCorner(6,m_variables) = m_jacobian * pseudoinv(m_mass_tilda);  
            m_Task_A_Eq.topRightCorner(m_slacks, m_slacks).setIdentity();
    
            return m_Task_A_Eq;
        }


        Eigen::VectorXd SimpleTorqueTask::Get_B_Equality()
        {
            Eigen::VectorXd err_6d(6);
            err_6d = get_error_6d(m_current_pose, m_target_pose);
            m_Task_B_Eq = m_Kp.cwiseProduct(err_6d) - m_Kd.cwiseProduct(m_current_twist);

            return m_Task_B_Eq;
        }

    }
}