#ifndef __rci_cdpr_qp_solver__
#define __rci_cdpr_qp_solver__

#include "rci_cdpr_controller/solver/qp_solver_mm.hpp"
#include "eiquadprog/eiquadprog.hpp"
#include <iostream>
#include <Eigen/Dense>
#include <Eigen/Core>
#include <ros/ros.h>
#include <cmath>
#include <fstream>
#include <vector>
#include <algorithm>
using namespace Eigen;
using namespace std;

# define M_PI           3.14159265358979323846


namespace cdpr_controller
{
    namespace qp_solver{
        
        cdpr_hqp::cdpr_hqp()
        {
            m_n = 15;
            m_err_6d.resize(6);
            m_v_current.resize(6);
            m_Kp.resize(6);
            m_Kd.resize(6);
            m_J.resize(6,15);
            m_pd_control.resize(3);
            m_J_cdpr.resize(6,8);
        }


        void cdpr_hqp::qp_setting(Eigen::VectorXd &error_6d,Eigen::VectorXd &v_current, Eigen::MatrixXd &jacob, Eigen::MatrixXd &platform_inv_J, Eigen::MatrixXd &A_mat, Eigen::VectorXd &pd_term){
            m_Aeq.setZero(9,m_n);
            m_Beq.setZero(9);
            m_Q.setZero(m_n,m_n);
            m_C.setZero(m_n);
            m_Aineq.setZero(0,m_n);
            m_Bineq.setZero(0);
            m_x.setZero(m_n);
            m_activeSet.setZero(0);

            m_err_6d.resize(6);
            m_err_6d = error_6d;

            m_v_current = v_current;
            m_J = jacob * A_mat;
            m_J_cdpr = platform_inv_J;
            m_pd_control = pd_term;

        }

        void cdpr_hqp::gain_setting(Eigen::VectorXd & kp_gain, Eigen::VectorXd & kd_gain)
        {
            m_Kp = kp_gain;
            m_Kd = kd_gain;
        }

        void cdpr_hqp::set_Q(){
            m_Q.setIdentity();
        }

        void cdpr_hqp::set_C(){
            m_C.setZero();
        }

        void cdpr_hqp::set_eq(){

            m_Aeq.topLeftCorner(6,m_n) = m_J;
            m_Aeq.bottomLeftCorner(3,8) = m_J_cdpr.bottomLeftCorner(3,8);
            // m_Beq = -(m_Kp.cwiseProduct(m_err_6d) - m_Kd.cwiseProduct(m_v_current));
            // m_Beq = m_Kp.cwiseProduct(m_err_6d);
            m_Beq.head(6) = -(m_Kp.cwiseProduct(m_err_6d) - m_Kd.cwiseProduct(m_v_current));
            m_Beq.tail(3) = -m_pd_control;
        }

        void cdpr_hqp::set_ineq(){
        }

        Eigen::VectorXd cdpr_hqp::solve(){
            cdpr_hqp::set_Q();
            cdpr_hqp::set_C();
            cdpr_hqp::set_eq();
            // ROS_INFO_STREAM(m_Aeq);
            // ROS_INFO_STREAM(m_Beq);
            // cdpr_hqp::set_ineq();
            
            double out = eiquadprog::solvers:: 
                         solve_quadprog(m_Q, m_C, m_Aeq.transpose(), m_Beq,
                                        m_Aineq.transpose(), m_Bineq,
                                        m_x, m_activeSet, m_activeSetSize);
            
            return m_x;
        }
    }
}

#endif