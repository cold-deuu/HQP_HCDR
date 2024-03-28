#include "rci_cdpr_controller/solver/HQP_solver.hpp"
#include "eiquadprog/eiquadprog.hpp"
#include <iostream>
#include <cmath>
#include <fstream>
#include <vector>
#include <algorithm>
using namespace Eigen;
using namespace std;

# define M_PI           3.14159265358979323846


namespace cdpr_controller
{
    namespace HQP_solver{
        
        cdpr_hqp::cdpr_hqp(int variables, int slacks)
        : m_variables(variables), m_slacks(slacks)
        {
            m_Task_Num = 0;
            m_rows = 0;
            m_cost_trigger = false;
            m_Q.setZero(m_variables+m_slacks, m_variables+m_slacks);
            m_C.setZero(m_variables+m_slacks);        
            m_x.resize(m_variables + m_slacks);
            m_slack_vector.resize(m_slacks);
            m_Task_A_equality.setZero(0,m_variables + m_slacks);
            m_Task_B_equality.setZero(0);
            m_A_eq_dummy.clear();
            m_B_eq_dummy.clear();
            m_Q_dummy.clear();
            m_C_dummy.clear();

            //for in eq test
            m_M_tilda.resize(15,15);
            m_G_tilda.resize(15);
            m_q_current.resize(7);
            m_v_current.resize(7);
            m_qlim.resize(2,7);

            m_joint_upper_bound.resize(7);
            m_joint_lower_bound.resize(7);

            m_Kp = 4000;
            m_Kd = sqrt(4000);
            m_buffer = 10 * M_PI/180;

            m_qlim << -2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8793,
                        2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8793;

        }

        void cdpr_hqp::Add_Task(Eigen::MatrixXd &A_Eq, Eigen::VectorXd &B_Eq, int Task_Level){
            if(A_Eq.rows() != B_Eq.rows())
                cout<<Task_Level<<" Level Task Did Not applied"<<endl;
            else if(A_Eq.cols() != (m_variables+m_slacks))
                cout<<Task_Level<<" Level Task Can Not Use HQP(No Slack)"<<A_Eq.cols()<<"and"<<m_variables+m_slacks<<endl;
            else if(A_Eq.rows() == B_Eq.rows()){
                
                m_A_eq_dummy[Task_Level] = A_Eq;
                m_B_eq_dummy[Task_Level] = B_Eq;

                if(m_Task_Num<Task_Level)
                    m_Task_Num = Task_Level;
                // m_Task_Num += 1;
            }
        }
        
        void cdpr_hqp::Add_Min_Task(Eigen::MatrixXd &Q, Eigen::VectorXd & C, int Task_Level){
            // if(Q.rows() != C.size())
            //     cout<<Task_Level<<" Level Task Did Not applied"<<endl;
            // else if(Q.cols() != (m_variables+m_slacks))
            //     cout<<Task_Level<<" Level Task Can Not Use HQP(No Slack)"<<Q.cols()<<"and"<<m_variables+m_slacks<<endl;

            // else if(Q.rows() == C.size()){
            m_Q_dummy[Task_Level] = Q;
            m_C_dummy[Task_Level] = C;

            if(m_Task_Num<Task_Level)
                m_Task_Num = Task_Level;
            // }
        }

        void cdpr_hqp::Update_Task(Eigen::MatrixXd &A_Eq, Eigen::VectorXd &B_Eq, int Task_Level){
            if(A_Eq.rows() != B_Eq.rows())
                cout<<Task_Level<<" Level Task Did Not applied"<<endl;
            else if(A_Eq.cols() != (m_variables+m_slacks))
                cout<<Task_Level<<" Level Task Can Not Use HQP(No Slack)"<<A_Eq.cols()<<"and"<<m_variables+m_slacks<<endl;
            else if(A_Eq.rows() == B_Eq.rows()){
                m_A_eq_dummy.erase(Task_Level);
                m_B_eq_dummy.erase(Task_Level);

                m_A_eq_dummy[Task_Level] = A_Eq;
                m_B_eq_dummy[Task_Level] = B_Eq;

            }
        }

        void cdpr_hqp::Update_Min_Task(Eigen::MatrixXd &Q, Eigen::VectorXd &C, int Task_Level){
            // if(Q.rows() != C.size())
            //     cout<<Task_Level<<" Level Task Did Not applied"<<endl;
            m_Q_dummy.erase(Task_Level);
            m_C_dummy.erase(Task_Level);
            m_Q_dummy[Task_Level] = Q;
            m_C_dummy[Task_Level] = C;
            // }
        }


        void cdpr_hqp::Remove_Task(){
            m_A_eq_dummy.clear();
            m_B_eq_dummy.clear();
            m_Task_Num = 0;
            m_rows = 0;
            m_x.resize(m_variables + m_slacks);
            m_slack_vector.resize(m_slacks);
            m_Task_A_equality.setZero(0,m_variables + m_slacks);
            m_Task_B_equality.setZero(0);
            m_Q_dummy.clear();
            m_C_dummy.clear();
        }

        void cdpr_hqp::set_Q(){
            m_Q.setZero(m_variables+m_slacks, m_variables+m_slacks);
            m_Q.setIdentity();
            m_Q.topLeftCorner(m_variables,m_variables) *= 1e-5;
        }

        void cdpr_hqp::get_tilda(Eigen::MatrixXd & mass_tilda, Eigen::VectorXd & g_tilda){
            m_M_tilda = mass_tilda;
            m_G_tilda = g_tilda;
        }

        void cdpr_hqp::set_C(){
            m_C.setZero(m_variables+m_slacks);
        }

        void cdpr_hqp::Set_Current_State(Eigen::VectorXd q_current, Eigen::VectorXd v_current){
            m_q_current = q_current;
            m_v_current = v_current;
        }

        void cdpr_hqp::set_ineq(){
            m_Aineq.setZero(0,m_variables+m_slacks);
            m_Bineq.setZero(0);
        }

        Eigen::VectorXd cdpr_hqp::solve(){
            m_Task_A_equality.setZero(0,m_variables+m_slacks);
            m_Task_B_equality.setZero(0);
            m_rows = 0;
            m_slack_vector.setZero(m_slacks);

            for (int i = 0; i<m_Task_Num; i++){
                // History (Temp)
                Eigen::MatrixXd A_Eq_history(m_rows,m_variables + m_slacks), A_dummy_tmp, Q_tmp;
                Eigen::VectorXd B_Eq_history(m_rows), B_dummy_tmp, C_tmp;
                A_Eq_history = m_Task_A_equality;
                B_Eq_history = m_Task_B_equality;  
                
                if(i!=0){
                    A_Eq_history.bottomRightCorner(m_slacks,m_slacks).setZero(); 
                    B_Eq_history.tail(m_slacks) -= m_slack_vector;
                }
                    
                int dummy_rows, dummy_cols, history_rows, history_cols;

                history_rows = A_Eq_history.rows();
                history_cols = A_Eq_history.cols();

                auto A_iter = m_A_eq_dummy.find(i+1);
                if( A_iter != m_A_eq_dummy.end())
                {
                    A_dummy_tmp = A_iter -> second;
                    dummy_rows = A_dummy_tmp.rows();
                    dummy_cols = A_dummy_tmp.cols();

                    m_rows += dummy_rows;
                    m_Task_A_equality.resize(m_rows, m_variables + m_slacks);
                    m_Task_A_equality.topLeftCorner(history_rows,history_cols) = A_Eq_history;
                    m_Task_A_equality.bottomLeftCorner(dummy_rows,dummy_cols) = A_dummy_tmp;
                }
                else
                    A_dummy_tmp.resize(0,21);

                auto B_iter = m_B_eq_dummy.find(i+1);
                if( B_iter != m_B_eq_dummy.end() )
                {
                    B_dummy_tmp = B_iter -> second;
                    m_Task_B_equality.resize(m_rows);
                    m_Task_B_equality.head(history_rows) = B_Eq_history;
                    m_Task_B_equality.tail(dummy_rows) = B_dummy_tmp;
                }
                else
                    B_dummy_tmp.resize(0); 

                auto Q_iter = m_Q_dummy.find(i+1);
                if( Q_iter == m_Q_dummy.end() )
                {
                    Q_tmp.setZero(m_variables + m_slacks, m_variables + m_slacks);
                    Q_tmp.setIdentity();
                    Q_tmp.topLeftCorner(m_variables, m_variables) *= 1e-5;
                }

                else{
                    Q_tmp.setZero(m_variables + m_slacks, m_variables + m_slacks);
                    Q_tmp.setIdentity();
                    Q_tmp.topLeftCorner(m_variables, m_variables) = Q_iter -> second;
                    Q_tmp.bottomRightCorner(m_slacks, m_slacks) *= 1e+5;
                }

                auto C_iter = m_C_dummy.find(i+1);
                if( C_iter == m_C_dummy.end() )
                {
                    C_tmp.setZero(m_variables + m_slacks);
                }

                else
                {
                    C_tmp.setZero(m_variables + m_slacks);
                    C_tmp.head(m_variables) = C_iter -> second;
                }

                cdpr_hqp::set_ineq();

                // m_Aineq.setZero(30,21);
                // m_Bineq.setZero(30);

                // m_Aineq.block<8,8>(0,0).setIdentity();
                // m_Aineq.block<8,8>(8,0).setIdentity();
                // m_Aineq.block<7,7>(16,8).setIdentity();
                // m_Aineq.block<7,7>(23,8).setIdentity();
                // m_Aineq.block<7,7>(16,8) *= -1;

                // m_Bineq.head(8) << 50, 50, 50, 50, 50, 50, 50, 50;
                // m_Bineq.segment(8,8) << 1e+4, 1e+4, 1e+4, 1e+4, 1e+4, 1e+4, 1e+4, 1e+4;

                // for(int j=0; j<8; j++){
                //     m_Bineq(j) -= m_G_tilda(j);
                //     m_Bineq(j+8) -= m_G_tilda(j);
                // }
                // m_Bineq.head(8) *= -1;
                // m_Aineq.block(8,0,8,8) *= -1;
    

                // for (int j=0; j<7; j++){
                //     if((m_qlim(0,j)+m_buffer)>m_q_current(j)){
                //         m_joint_lower_bound(j) = m_Kp * ((m_qlim(0,j)+m_buffer)-m_q_current(j)) - m_Kd * m_v_current(j);
                //         if(m_joint_lower_bound(i) < -200)
                //             m_joint_lower_bound(i) = -200;
                //         m_joint_upper_bound(j) = 200;
                //         cout<<"check"<<endl;
                //     }
                //     else if ((m_qlim(1,j)-m_buffer)<m_q_current(j)){
                //         m_joint_upper_bound(j) = m_Kp * ((m_qlim(1,j)-m_buffer)-m_q_current(j)) - m_Kd * m_v_current(j);
                //         if(m_joint_upper_bound(i) > 200)
                //             m_joint_upper_bound(i) = 200;
                //         m_joint_lower_bound(j) = -200;
                //         cout<<"dsf"<<endl;
                //     }
                //     else{
                //         m_joint_upper_bound(j) = 200;
                //         m_joint_lower_bound(j) = -200;
                //     }
                // }

                // m_joint_upper_bound = m_M_tilda.bottomRightCorner(7,7) * m_joint_upper_bound;
                // m_joint_lower_bound = m_M_tilda.bottomRightCorner(7,7) * m_joint_lower_bound;
                
                // m_Bineq.segment(16,7) = m_joint_upper_bound;
                // m_Bineq.tail(7) = -m_joint_lower_bound;
                



                // cout << m_Bineq.transpose() << endl;


                // for(int j=0; j<7; j++){
                //     if(m_q_current(j)-m_qlim(0,j) <= 0.3){
                //         m_Aineq(j+16,j+8) = 1;
                //         m_Bineq(j+16) = -100 * (5*M_PI/180 - (m_q_current(j) - m_qlim(0,j)));
                //         cout<<"check"<<j+1<<endl;
                //     }
                //     else if(m_qlim(1,j)-m_q_current(j) <=0.3){
                //         m_Aineq(j+16,j+8) = -1;
                //         m_Bineq(j+16) = 100 * ((m_qlim(1,j) - m_q_current(j))-5*M_PI/180);
                //         cout<<"check2"<<j+1<<endl;
                //     }
                // }

                // m_Bineq.tail(7) = m_M_tilda.bottomRightCorner(7,7) * m_Bineq.tail(7);



                // if(i==0){
                    cout<<"A\n"<<m_Task_A_equality<<endl;
                    cout<<"B\n"<<m_Task_B_equality.transpose()<<endl;

                // }



                double out = eiquadprog::solvers:: 
                    solve_quadprog(Q_tmp, C_tmp, m_Task_A_equality.transpose(), -m_Task_B_equality,
                                m_Aineq.transpose(), m_Bineq,
                                m_x, m_activeSet, m_activeSetSize);

                
                m_slack_vector = m_x.tail(m_slacks);

                // 
                cout<<i+1<<" Priority Task\n"<<m_slack_vector.transpose()<<endl;
                
                if(i==0){
                    std::string filepath = "/home/chan/rci_cdpr_ws/src/cdpr_controllers/txt/1stSlack.txt";
                    ofstream fout(filepath,std::ios::app);
                    for(int j =0; j<6; j++){
                        fout << m_x(j+15) << '\t';
                    }
                    fout << "\n";
                    fout.close();
                }
            }
            return m_x.head(m_variables);
        }
    }
}
