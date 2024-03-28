#include "rci_cdpr_controller/task/platform_static_task.hpp"
// #include "rci_cdpr_controller/math/math.hpp"

using namespace pinocchio;
using namespace Eigen;
using namespace std;
using namespace cdpr_controller::math;

namespace cdpr_controller
{
    namespace platform_task
    {
        Platform_Static_Task::Platform_Static_Task(const std::string & task_name, pinocchio::SE3 &platform_position, bool use_torque, int variables, int slacks)
        : m_platform_pose(platform_position), m_use_torque(use_torque), m_variables(variables), m_slacks(slacks)
        {
            m_Kp.resize(6);
            m_Kd.resize(6);
            m_A_equality.setZero(6,m_variables + m_slacks);
            m_B_equality.setZero(6);
            m_platform_jacob.setZero(8,6);
            m_current_twist.setZero(6);
            m_Mass_tilda.setZero(m_variables, m_variables);
        }

        void Platform_Static_Task::Set_Gain(Eigen::VectorXd &Kp, Eigen::VectorXd &Kd)
        {
            m_Kp = Kp;
            m_Kd = Kd;
        }
        
        void Platform_Static_Task::Set_Current_State(Eigen::MatrixXd &mass_tilda, Eigen::MatrixXd platform_jacobian, pinocchio::SE3 & current_position, Eigen::VectorXd current_twist)
        {
            m_current_position = current_position;
            m_current_twist = current_twist;
            m_platform_jacob = platform_jacobian;
            m_Mass_tilda = mass_tilda;
        }

        Eigen::MatrixXd Platform_Static_Task::Get_A_Equality(){
            Eigen::MatrixXd a_tmp(6,8);
            a_tmp.setZero();
            a_tmp = m_platform_jacob.completeOrthogonalDecomposition().pseudoInverse();

            m_A_equality.setZero();
            m_A_equality.topLeftCorner(6,8) = a_tmp;
            Eigen::MatrixXd abc(15,15), mass_tmp(8,8);

            abc.setZero();
            mass_tmp = m_Mass_tilda.topLeftCorner(8,8);
            mass_tmp = mass_tmp.completeOrthogonalDecomposition().pseudoInverse();
            // cout<<"asdasd\n"<<mass_tmp<<endl;
            abc.topLeftCorner(8,8) = mass_tmp;
        


            if (m_use_torque)
                m_A_equality.topLeftCorner(6,15) = m_A_equality.topLeftCorner(6,15) * abc;
            
            m_A_equality.topRightCorner(6,6).setIdentity();
         
            return m_A_equality;
        }

        Eigen::VectorXd Platform_Static_Task::Get_B_Equality(){
            Eigen::VectorXd platform_err = get_error_6d(m_current_position, m_platform_pose);
            m_B_equality = m_Kp.cwiseProduct(platform_err) - m_Kd.cwiseProduct(m_current_twist);
            return m_B_equality;
        }

    }
}
