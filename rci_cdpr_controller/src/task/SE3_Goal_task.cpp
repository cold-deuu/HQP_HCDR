#include "rci_cdpr_controller/task/SE3_Goal_task.hpp"
// #include "rci_cdpr_controller/math/math.hpp"


using namespace cdpr_controller::math;

namespace cdpr_controller{
    namespace SE3_task{
        SE3_Goal_Task::SE3_Goal_Task(const std::string & Task_Name, int variables, int slacks)
        : m_variables(variables), m_slacks(slacks)
        {
            m_Kp.resize(6);
            m_Kd.resize(6);
            m_current_twist.setZero(6);

            m_A_equality.setZero(6,m_variables + m_slacks);
            m_B_equality.setZero(6);

        }

        void SE3_Goal_Task::Set_Gain(Eigen::VectorXd &Kp, Eigen::VectorXd &Kd)
        {
            m_Kp = Kp;
            m_Kd = Kd;
        }
        
        void SE3_Goal_Task::Set_Current_State(Eigen::MatrixXd Jacob_Tilda, pinocchio::SE3 & current_position, Eigen::VectorXd & current_twist)
        {
            m_Jacob_tilda = Jacob_Tilda;
            m_current_pose = current_position;
            m_current_twist = current_twist;
        }

        void SE3_Goal_Task::Set_Goal_State(pinocchio::SE3 & target_pose){
            m_target_pose = target_pose;
        }



        Eigen::MatrixXd SE3_Goal_Task::Get_A_Equality()
        {
            m_A_equality.topLeftCorner(6,15) = m_Jacob_tilda;
            m_A_equality.topRightCorner(6,6).setIdentity();
            
            return m_A_equality;
        }
        
        Eigen::VectorXd SE3_Goal_Task::Get_B_Equality()
        {
            Eigen::VectorXd goal_err(6);
            goal_err = get_error_6d(m_current_pose, m_target_pose);
            m_B_equality = m_Kp.cwiseProduct(goal_err) - m_Kd.cwiseProduct(m_current_twist);
            return m_B_equality;
        }

    }
}