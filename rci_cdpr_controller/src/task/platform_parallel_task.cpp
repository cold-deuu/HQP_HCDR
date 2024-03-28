#include "rci_cdpr_controller/task/platform_parallel_task.hpp"
// #include "rci_cdpr_controller/math/math.hpp"

using namespace pinocchio;
using namespace Eigen;
using namespace std;
using namespace cdpr_controller::math;

namespace cdpr_controller
{
    namespace platform_task
    {
        Platform_Parallel_Task::Platform_Parallel_Task(const std::string & task_name, pinocchio::SE3 &platform_position,  bool use_torque, int variables, int slacks)
        : m_platform_pose(platform_position), m_variables(variables), m_slacks(slacks), m_use_torque(use_torque)
        {
            m_Kp.resize(3);
            m_Kd.resize(3);

            m_A_equality.setZero(3,m_variables + 3);
            m_B_equality.setZero(3);

            m_Mass_tilda.setZero(m_variables, m_variables);

        }

        void Platform_Parallel_Task::Set_Gain(Eigen::VectorXd &Kp, Eigen::VectorXd &Kd)
        {
            m_Kp = Kp.tail(3);
            m_Kd = Kd.tail(3);
        }

        void Platform_Parallel_Task::Set_Current_State(Eigen::MatrixXd & mass_tilda, Eigen::MatrixXd platform_jacobian, pinocchio::SE3 & current_position, Eigen::VectorXd current_twist)
        {
            m_platform_jacob = platform_jacobian;
            m_current_position = current_position;
            m_current_twist = current_twist;
            m_Mass_tilda = mass_tilda;
        }

        Eigen::MatrixXd Platform_Parallel_Task::Get_A_Equality(){
            m_A_equality.setZero();
            if (!m_use_torque){
                m_A_equality.bottomLeftCorner(3,8) = pseudoinv(m_platform_jacob).bottomLeftCorner(3,8);
                m_A_equality.topRightCorner(6,6).setIdentity();
            }

            else{
                Eigen::MatrixXd tmp(m_variables, m_variables), tmp_jacob(3,m_variables);
                tmp.setZero();
                tmp.topLeftCorner(8,8) = m_Mass_tilda.topLeftCorner(8,8);
                tmp = tmp.completeOrthogonalDecomposition().pseudoInverse();
                // tmp.topLeftCorner(8,8) = pseudoinv(m_Mass_tilda);
                tmp_jacob = pseudoinv(m_platform_jacob).bottomLeftCorner(3,8);

                m_A_equality.topLeftCorner(3,m_variables) =  tmp_jacob * tmp;
                m_A_equality.bottomRightCorner(3,3).setIdentity();
            }
            return m_A_equality;
        }

        Eigen::VectorXd Platform_Parallel_Task::Get_B_Equality(){
            m_B_equality.setZero();
            Eigen::VectorXd platform_err(3);
            platform_err = cdpr_controller::math::get_error_6d(m_current_position, m_platform_pose).tail(3);
            m_B_equality = m_Kp.cwiseProduct(platform_err) - m_Kd.cwiseProduct(m_current_twist.tail(3));

            return m_B_equality;
        }

    }
}
