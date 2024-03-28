#include <Eigen/QR>    
#include <Eigen/Core>
#include <Eigen/Dense>
#include <string>
#include <vector>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/spatial/fwd.hpp>
#include "eiquadprog/eiquadprog.hpp"

namespace cdpr_controller{
    namespace qp_solver{
        class cdpr_hqp{
            public:
                cdpr_hqp();
                Eigen::VectorXd m_err_6d,m_v_current,m_x,m_pd_control;
                int m_n;

                Eigen::MatrixXd m_J, m_J_cdpr;
                Eigen::MatrixXd m_Q,m_Aeq,m_Aineq;
                Eigen::VectorXd m_C,m_Beq,m_Bineq,m_qdlim;
                Eigen::VectorXd m_Kp, m_Kd, m_Task_B_equality;
                Eigen::MatrixXd m_Task_A_equality;

                Eigen::VectorXi m_activeSet;
                size_t m_activeSetSize;
                void initialize();
                void add_Task(Eigen::MatrixXd);
                void qp_setting(Eigen::VectorXd &error_6d, Eigen::VectorXd &v_current, Eigen::MatrixXd &jacob, Eigen::MatrixXd &platform_inv_J, Eigen::MatrixXd &A_mat, Eigen::VectorXd &pd_term);
                void gain_setting(Eigen::VectorXd & kp_gain, Eigen::VectorXd & kd_gain);
                void set_Q();
                void set_C();
                void set_eq();
                void set_ineq();
                Eigen::VectorXd solve();
                
        };
    }
}