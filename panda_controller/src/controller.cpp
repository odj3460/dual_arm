#include "panda_controller/panda_controller.h"

PandaController::PandaController(ros::NodeHandle &nh, DataContainer &dc) : dc_(dc)
{
    dc_.sim_mode_ = "position";
    
    right_arm_.id_ = "right_arm";
    left_arm_.id_ = "left_arm";
    robots_.push_back(&right_arm_);
    robots_.push_back(&left_arm_);

    // RBDL
    std::string urdf_name = ros::package::getPath("panda_description") + "/robots/panda_arm_dj.urdf";
    std::cout<<"Robot Model name: " << urdf_name <<std::endl;
    RigidBodyDynamics::Addons::URDFReadFromFile(urdf_name.c_str(), &right_arm_.robot_model_, false, false);
    RigidBodyDynamics::Addons::URDFReadFromFile(urdf_name.c_str(), &left_arm_.robot_model_, false, false);

    init_keyboard();

    txt_right_.open("/home/odong/dual_ws/src/data_right.txt", std::ios::out); 
    txt_left_.open("/home/odong/dual_ws/src/data_left.txt", std::ios::out); 

    if (!txt_right_.is_open())
    {
        std::cout << "Unable to open right txt file" << std::endl;
        return;
    }
    if (!txt_left_.is_open())
    {
        std::cout << "Unable to open left txt file" << std::endl;
        return;
    }

}

PandaController::~PandaController()
{
    // txt_right_.close();
    // txt_left_.close();
}

void PandaController::compute()
{
    
    ros::Rate r(2000);
    while(ros::ok())
    {
        if (!dc_.is_first_callback)
        {
            if (!is_init_)
            {
                for (auto &robot: robots_)
                {
                    robot->q_.resize(DOF);
                    robot->q_.setZero();
                    robot->q_dot_.resize(DOF);
                    robot->q_dot_.setZero();
                    robot->q_dot_zero_.resize(DOF);
                    robot->q_dot_zero_.setZero();
                    robot->effort_.resize(DOF);
                    robot->effort_.setZero();
                    robot->control_input_.resize(DOF);
                    robot->control_input_.setZero();
                    robot->grasp_input_.resize(2);
                    robot->grasp_input_.setZero();

                    robot->q_desired.resize(DOF);
                    robot->q_desired.setZero();
                    robot->q_init_.resize(DOF);
                    robot->q_init_.setZero();

                    robot->j_temp_.resize(6, DOF);
                    robot->j_temp_.setZero();
                    robot->j_.resize(6, DOF);
                    robot->j_.setZero();

                    robot->A_.resize(DOF, DOF);
                    robot->A_.setZero();
                    robot->g_.resize(DOF);
                    robot->g_.setZero();
                }

                is_init_ = true;
            } 

            m_dc_.lock();
            right_arm_.ft_before_.head(3) = right_arm_.ft_.head(3);
            left_arm_.ft_before_.head(3) = left_arm_.ft_.head(3);
            right_arm_.ft_before_.tail(3) = right_arm_.ft_.tail(3);
            left_arm_.ft_before_.tail(3) = left_arm_.ft_.tail(3);
            right_arm_.x_dot_before_ = right_arm_.x_dot_;
            left_arm_.x_dot_before_ = left_arm_.x_dot_;
            sim_time_ = dc_.sim_time_;
            right_arm_.q_ = dc_.q_.head(DOF);
            left_arm_.q_ = dc_.q_.segment(DOF+2,DOF);
            right_arm_.q_dot_ = dc_.q_dot_.head(DOF);
            left_arm_.q_dot_ = dc_.q_dot_.segment(DOF+2,DOF);
            right_arm_.effort_ = dc_.effort_.head(DOF);
            left_arm_.effort_ = dc_.effort_.segment(DOF+2,DOF);
            right_arm_.ft_.head(3) = dc_.force_.head(3);
            right_arm_.ft_.tail(3) = dc_.torque_.head(3);
            left_arm_.ft_.head(3) = dc_.force_.tail(3);
            left_arm_.ft_.tail(3) = dc_.torque_.tail(3);
            m_dc_.unlock();


            updateKinematicsDynamics();
            computeControlInput();

            if (_kbhit()) {
                int ch = _getch();
                _putch(ch);
                mode_ = ch;

                for (auto &robot: robots_)
                {
                    robot->x_init_.translation() = robot->x_.translation();
                    robot->x_init_.linear() = robot->x_.linear();
                    robot->ft_fix_.head(3) = robot->ft_.head(3);
                    robot->ft_fix_.tail(3) = robot->ft_.tail(3);
                }
                control_start_time_ = sim_time_;

                std::cout << "Mode Changed to: ";   // i: 105, r: 114, m: 109, s: 115, f:102, h: 104
                switch(mode_)
                {
                    case(104):
                        std::cout << "Home Pose" << std::endl;
                        break;
                    case(105):
                        std::cout<< "Init Pose" << std::endl;
                        break;
                    case(102):
                        std::cout << "Force Control" << std::endl;
                        break;
                    case(115):
                        std::cout << "Stop" << std::endl;
                        break;
                    case(113):
                        std::cout << "Move Set 1" << std::endl;
                        break;
                    case(119):
                        std::cout << "Move Set 2" << std::endl;
                        break;
                    case(101):
                        std::cout << "Move Set 3" << std::endl;
                        break;
                    case(114):
                        std::cout << "Move Set 4" << std::endl;
                        break;
                    case(116):
                        std::cout << "Move Set 5" << std::endl;
                        break;

                    case(122):
                        std::cout << "Move Scene 1" << std::endl;
                        break;
                    case(120):
                        std::cout << "Move Scene 2" << std::endl;
                        break;
                    case(99):
                        std::cout << "Move Scene 3" << std::endl;
                        break;
                    case(118):
                        std::cout << "Move Scene 4" << std::endl;
                        break;
                        
                }


            }

        }
    
        txt_right_ << sim_time_ << " " << control_start_time_<< " " << right_arm_.x_.translation().transpose() << " " << right_arm_.ft_.head(3).transpose() << " " << right_arm_.ft_fix_.head(3).transpose() << " " << right_arm_.x_desired_.transpose()<< std::endl;
        txt_left_ << sim_time_ << " " << control_start_time_<< " " << left_arm_.x_.translation().transpose() << " " << left_arm_.ft_.head(3).transpose() << " " << left_arm_.ft_fix_.head(3).transpose() << " " << left_arm_.x_desired_.transpose()<< std::endl;

        r.sleep();
    }
    close_keyboard();
}

void PandaController::updateKinematicsDynamics()
{
    for (auto &robot: robots_)
    {
        static const int BODY_ID = robot->robot_model_.GetBodyId("panda_link8");

        // Kinematoics
        robot->x_.translation().setZero();
        robot->x_.linear().setZero();
        robot->x_.translation() = RigidBodyDynamics::CalcBodyToBaseCoordinates(robot->robot_model_, robot->q_, BODY_ID, Eigen::Vector3d(0.0, 0.0, 0.0), true);
        robot->x_.linear() = RigidBodyDynamics::CalcBodyWorldOrientation(robot->robot_model_, robot->q_, BODY_ID, true).transpose();

        robot->j_temp_.setZero();
        RigidBodyDynamics::CalcPointJacobian6D(robot->robot_model_, robot->q_, BODY_ID, Eigen::Vector3d(0.0, 0.0, 0.0), robot->j_temp_, true);
        robot->j_.setZero();
        for (int i = 0; i<2; i++)
        {
            robot->j_.block<3, 7>(i * 3, 0) = robot->j_temp_.block<3, 7>(3 - i * 3, 0);
        }    

        robot->x_dot_ = robot->j_ * robot->q_dot_;

        RigidBodyDynamics::NonlinearEffects(robot->robot_model_, robot->q_, robot->q_dot_zero_, robot->g_);
    }
}

void PandaController::computeControlInput()
{
    
    
    Eigen::Matrix6d Kp_;
    Kp_.setZero();
    for (int i = 0; i < 3; i++)
    {
        Kp_(i,i) = 4800;    //2000  //2800
        Kp_(i+3, i+3) = 3800;     //400     //1800
    } 

    if (mode_ == MODE_HOME)
    {
        
        for (auto &robot: robots_)
        {
            if (robot->id_ == "right_arm")
            {
                Eigen::Isometry3d x_target_;
                x_target_.translation() << 0.604, 0.36, 0.51;
                x_target_.linear() << 0.7071 , 0.7071, 0, 0.7071, -0.7071, 0, 0 ,0, -1; 
                double duration_time = 10.0;

                Eigen::MatrixXd j_inverse_;
                j_inverse_.resize(7,6);
                j_inverse_ = robot->j_.transpose()*((robot->j_*robot->j_.transpose()).inverse());

                Eigen::Isometry3d x_cubic_;
                Eigen::Vector6d x_cubic_dot_;
                for (int i = 0; i < 3; i++)
                {
                    x_cubic_.translation()(i) = cubic(sim_time_, control_start_time_, control_start_time_ + duration_time, robot->x_init_.translation()(i), x_target_.translation()(i), 0.0, 0.0);
                    x_cubic_dot_(i) = cubicDot(sim_time_, control_start_time_, control_start_time_ + duration_time, robot->x_init_.translation()(i), x_target_.translation()(i), 0.0, 0.0);
                }

                Eigen::Vector3d rotation_cubic_dot;
                x_cubic_.linear() = rotationCubic(sim_time_, control_start_time_, control_start_time_ + duration_time, robot->x_init_.linear(), x_target_.linear());
                rotation_cubic_dot = rotationCubicDot(sim_time_, control_start_time_, control_start_time_ + duration_time, robot->x_init_.linear(), x_target_.linear());

                for (int i = 0; i < 3; i++)
                {
                    x_cubic_dot_(i + 3) = rotation_cubic_dot(i);
                }

                Eigen::Vector6d x_error_;
                x_error_ << x_cubic_.translation() - robot->x_.translation(), -getPhi(robot->x_.linear(), x_cubic_.linear());

                Eigen::Vector7d q_desired_;
                q_desired_ = robot->q_ + j_inverse_ * (x_cubic_dot_+ Kp_* x_error_)/2000.0;

                for (int i = 0; i < DOF; i++)
                {
                    robot->control_input_(i) = q_desired_(i);
                }
                robot->grasp_input_ << 0.2, 0.2;
            }
            else if (robot->id_ == "left_arm")
            {
                Eigen::Isometry3d x_target_;
                x_target_.translation() << 0.6, 0.32, 0.336;
                x_target_.linear() << 0.7071 , 0.7071, 0, 0.7071, -0.7071, 0, 0 ,0, -1; 
                double duration_time = 10.0;

                Eigen::MatrixXd j_inverse_;
                j_inverse_.resize(7,6);
                j_inverse_ = robot->j_.transpose()*((robot->j_*robot->j_.transpose()).inverse());

                Eigen::Isometry3d x_cubic_;
                Eigen::Vector6d x_cubic_dot_;
                for (int i = 0; i < 3; i++)
                {
                    x_cubic_.translation()(i) = cubic(sim_time_, control_start_time_, control_start_time_ + duration_time, robot->x_init_.translation()(i), x_target_.translation()(i), 0.0, 0.0);
                    x_cubic_dot_(i) = cubicDot(sim_time_, control_start_time_, control_start_time_ + duration_time, robot->x_init_.translation()(i), x_target_.translation()(i), 0.0, 0.0);
                }

                x_cubic_.linear() = rotationCubic(sim_time_, control_start_time_, control_start_time_ + duration_time, robot->x_init_.linear(), x_target_.linear());
                x_cubic_dot_.tail(3) = rotationCubicDot(sim_time_, control_start_time_, control_start_time_ + duration_time, robot->x_init_.linear(), x_target_.linear());

                Eigen::Vector6d x_error_;
                x_error_ << x_cubic_.translation() - robot->x_.translation(), -getPhi(robot->x_.linear(), x_cubic_.linear());

                Eigen::Vector7d q_desired_;
                q_desired_ = robot->q_ + j_inverse_ * (x_cubic_dot_ + Kp_* x_error_)/2000.0;
                
                for (int i = 0; i < DOF; i++)
                {
                    robot->control_input_(i) = q_desired_(i);
                }
                robot->grasp_input_ << 0.2, 0.2;
            }
        }
    }
    else if (mode_ == MODE_INIT)
    {
        for (auto &robot: robots_)
        {
            if (robot->id_ == "right_arm")
            {
                Eigen::Isometry3d x_target_;
                x_target_.translation() << 0.404, 0.02, 0.71;
                x_target_.linear() << 0.7071 , 0.7071, 0, 0.7071, -0.7071, 0, 0 ,0, -1; 
                double duration_time = 10.0;

                Eigen::MatrixXd j_inverse_;
                j_inverse_.resize(7,6);
                j_inverse_ = robot->j_.transpose()*((robot->j_*robot->j_.transpose()).inverse());

                Eigen::Isometry3d x_cubic_;
                Eigen::Vector6d x_cubic_dot_;
                for (int i = 0; i < 3; i++)
                {
                    x_cubic_.translation()(i) = cubic(sim_time_, control_start_time_, control_start_time_ + duration_time, robot->x_init_.translation()(i), x_target_.translation()(i), 0.0, 0.0);
                    x_cubic_dot_(i) = cubicDot(sim_time_, control_start_time_, control_start_time_ + duration_time, robot->x_init_.translation()(i), x_target_.translation()(i), 0.0, 0.0);
                }

                Eigen::Vector3d rotation_cubic_dot;
                x_cubic_.linear() = rotationCubic(sim_time_, control_start_time_, control_start_time_ + duration_time, robot->x_init_.linear(), x_target_.linear());
                rotation_cubic_dot = rotationCubicDot(sim_time_, control_start_time_, control_start_time_ + duration_time, robot->x_init_.linear(), x_target_.linear());

                for (int i = 0; i < 3; i++)
                {
                    x_cubic_dot_(i + 3) = rotation_cubic_dot(i);
                }

                // Eigen::Matrix6d Kp_;
                // Kp_.setZero();
                // for (int i = 0; i < 3; i++)
                // {
                //     Kp_(i,i) = 1600;    //2000
                //     Kp_(i+3, i+3) = 300;     //400
                // } 

                Eigen::Vector6d x_error_;
                x_error_ << x_cubic_.translation() - robot->x_.translation(), -getPhi(robot->x_.linear(), x_cubic_.linear());

                Eigen::Vector7d q_desired_;
                q_desired_ = robot->q_ + j_inverse_ * (x_cubic_dot_ + Kp_* x_error_)/2000.0;
                
                for (int i = 0; i < DOF; i++)
                {
                    robot->control_input_(i) = q_desired_(i);
                }
                robot->grasp_input_ << 0.2, 0.2;
            }
            else if (robot->id_ == "left_arm")
            {
                Eigen::Isometry3d x_target_;
                x_target_.translation() << 0.404, -0.02, 0.71;
                x_target_.linear() << 0.7071 , 0.7071, 0, 0.7071, -0.7071, 0, 0 ,0, -1; 
                double duration_time = 10.0;

                Eigen::MatrixXd j_inverse_;
                j_inverse_.resize(7,6);
                j_inverse_ = robot->j_.transpose()*((robot->j_*robot->j_.transpose()).inverse());

                Eigen::Isometry3d x_cubic_;
                Eigen::Vector6d x_cubic_dot_;
                for (int i = 0; i < 3; i++)
                {
                    x_cubic_.translation()(i) = cubic(sim_time_, control_start_time_, control_start_time_ + duration_time, robot->x_init_.translation()(i), x_target_.translation()(i), 0.0, 0.0);
                    x_cubic_dot_(i) = cubicDot(sim_time_, control_start_time_, control_start_time_ + duration_time, robot->x_init_.translation()(i), x_target_.translation()(i), 0.0, 0.0);
                }

                x_cubic_.linear() = rotationCubic(sim_time_, control_start_time_, control_start_time_ + duration_time, robot->x_init_.linear(), x_target_.linear());
                x_cubic_dot_.tail(3) = rotationCubicDot(sim_time_, control_start_time_, control_start_time_ + duration_time, robot->x_init_.linear(), x_target_.linear());

                // Eigen::Matrix6d Kp_;
                // Kp_.setZero();
                // for (int i = 0; i < 3; i++)
                // {
                //     Kp_(i,i) = 1600;    //2000
                //     Kp_(i+3, i+3) = 300;     //400
                // } 

                Eigen::Vector6d x_error_;
                x_error_ << x_cubic_.translation() - robot->x_.translation(), -getPhi(robot->x_.linear(), x_cubic_.linear());

                Eigen::Vector7d q_desired_;
                q_desired_ = robot->q_ + j_inverse_ * (x_cubic_dot_ + Kp_* x_error_)/2000.0;
                
                for (int i = 0; i < DOF; i++)
                {
                    robot->control_input_(i) = q_desired_(i);
                }
                robot->grasp_input_ << 0.2, 0.2;
            }
        }
    }
    else if (mode_ == MODE_1)
    {
        for (auto &robot: robots_)
        {
            if (robot->id_ == "right_arm")
            {
                Eigen::Isometry3d x_target_;
                x_target_.translation() << 0.600, 0.02, 0.51;
                x_target_.linear() << 0.7071 , -0.7071, 0, -0.7071, -0.7071, 0, 0 ,0, -1;   // 1, 0, 0, 0, -1, 0, 0, 0, -1;
                //  0.7071 , -0.7071, 0, -0.7071, -0.7071, 0, 0 ,0, -1;  (45 degree)
                //  0.7071 , 0.7071, 0, 0.7071, -0.7071, 0, 0 ,0, -1;    (-45 degree)
                double duration_time = 5.0;

                Eigen::MatrixXd j_inverse_;
                j_inverse_.resize(7,6);
                j_inverse_ = robot->j_.transpose()*((robot->j_*robot->j_.transpose()).inverse());

                Eigen::Isometry3d x_cubic_;
                Eigen::Vector6d x_cubic_dot_;
                for (int i = 0; i < 3; i++)
                {
                    x_cubic_.translation()(i) = cubic(sim_time_, control_start_time_, control_start_time_ + duration_time, robot->x_init_.translation()(i), x_target_.translation()(i), 0.0, 0.0);
                    x_cubic_dot_(i) = cubicDot(sim_time_, control_start_time_, control_start_time_ + duration_time, robot->x_init_.translation()(i), x_target_.translation()(i), 0.0, 0.0);
                }

                Eigen::Vector3d rotation_cubic_dot;
                x_cubic_.linear() = rotationCubic(sim_time_, control_start_time_, control_start_time_ + duration_time, robot->x_init_.linear(), x_target_.linear());
                rotation_cubic_dot = rotationCubicDot(sim_time_, control_start_time_, control_start_time_ + duration_time, robot->x_init_.linear(), x_target_.linear());

                for (int i = 0; i < 3; i++)
                {
                    x_cubic_dot_(i + 3) = rotation_cubic_dot(i);
                }

                // Eigen::Matrix6d Kp_;
                // Kp_.setZero();
                // for (int i = 0; i < 3; i++)
                // {
                //     Kp_(i,i) = 1200;    //2000
                //     Kp_(i+3, i+3) = 200;     //400
                // } 

                // xdot = J qdot
                // xdot = J *J^-1 * x_cubic_dot+ KP*x_error_
                // xcubic_dot -xdot +Kp xerr = 0
                // xerr_dot + Kp xerr = 0
                Eigen::Vector6d x_error_;
                x_error_ << x_cubic_.translation() - robot->x_.translation(), -getPhi(robot->x_.linear(), x_cubic_.linear());

                Eigen::Vector7d q_desired_;
                q_desired_ = robot->q_ + j_inverse_ * (x_cubic_dot_ + Kp_* x_error_)/2000.0;
                
                for (int i = 0; i < DOF; i++)
                {
                    robot->control_input_(i) = q_desired_(i);
                }
                robot->grasp_input_ << 0.2, 0.2;
            }
            else if (robot->id_ == "left_arm")
            {
                Eigen::Isometry3d x_target_;
                x_target_.translation() << 0.600, -0.02, 0.51;
                x_target_.linear() << 0.7071 , -0.7071, 0, -0.7071, -0.7071, 0, 0 ,0, -1;
                double duration_time = 5.0;

                Eigen::MatrixXd j_inverse_;
                j_inverse_.resize(7,6);
                j_inverse_ = robot->j_.transpose()*((robot->j_*robot->j_.transpose()).inverse());

                Eigen::Isometry3d x_cubic_;
                Eigen::Vector6d x_cubic_dot_;
                for (int i = 0; i < 3; i++)
                {
                    x_cubic_.translation()(i) = cubic(sim_time_, control_start_time_, control_start_time_ + duration_time, robot->x_init_.translation()(i), x_target_.translation()(i), 0.0, 0.0);
                    x_cubic_dot_(i) = cubicDot(sim_time_, control_start_time_, control_start_time_ + duration_time, robot->x_init_.translation()(i), x_target_.translation()(i), 0.0, 0.0);
                }

                Eigen::Vector3d rotation_cubic_dot;
                x_cubic_.linear() = rotationCubic(sim_time_, control_start_time_, control_start_time_ + duration_time, robot->x_init_.linear(), x_target_.linear());
                rotation_cubic_dot = rotationCubicDot(sim_time_, control_start_time_, control_start_time_ + duration_time, robot->x_init_.linear(), x_target_.linear());

                for (int i = 0; i < 3; i++)
                {
                    x_cubic_dot_(i + 3) = rotation_cubic_dot(i);
                }

                Eigen::Vector6d x_error_;
                x_error_ << x_cubic_.translation() - robot->x_.translation(), -getPhi(robot->x_.linear(), x_cubic_.linear());

                Eigen::Vector7d q_desired_;
                q_desired_ = robot->q_ + j_inverse_ * (x_cubic_dot_ + Kp_* x_error_)/2000.0;
                
                for (int i = 0; i < DOF; i++)
                {
                    robot->control_input_(i) = q_desired_(i);
                }
                robot->grasp_input_ << 0.2, 0.2;
            }
        }
        // for (auto &robot: robots_)
        // {
        //     if (robot->id_ == "right_arm")
        //     {
        //         robot->control_input_ <<  0.0172005, 0.19635, 0.0174662, -1.8951, -0.00102803, 0.511763, 0.820264;
        //     }
        //     else if (robot->id_ == "left_arm")
        //     {
        //         robot->control_input_ <<  -0.0173848, 0.196324, -0.0161123, -1.89513 , 0.0062706, 0.511765, 0.749106;
        //     }
        // }
        std::cout << " q left Pose:  " << left_arm_.q_.transpose() << std::endl;
        std::cout << " q right Pose:  " << right_arm_.q_.transpose() << std::endl;
    }
    else if (mode_ == MODE_2)
    {
        for (auto &robot: robots_)
        {
            if (robot->id_ == "right_arm")
            {
                Eigen::Isometry3d x_target_;
                x_target_.translation() << 0.600, 0.02, 0.473;
                x_target_.linear() << 0.7071 , -0.7071, 0, -0.7071, -0.7071, 0, 0 ,0, -1;
                double duration_time = 3.0;

                Eigen::MatrixXd j_inverse_;
                j_inverse_.resize(7,6);
                j_inverse_ = robot->j_.transpose()*((robot->j_*robot->j_.transpose()).inverse());

                Eigen::Isometry3d x_cubic_;
                Eigen::Vector6d x_cubic_dot_;
                for (int i = 0; i < 3; i++)
                {
                    x_cubic_.translation()(i) = cubic(sim_time_, control_start_time_, control_start_time_ + duration_time, robot->x_init_.translation()(i), x_target_.translation()(i), 0.0, 0.0);
                    x_cubic_dot_(i) = cubicDot(sim_time_, control_start_time_, control_start_time_ + duration_time, robot->x_init_.translation()(i), x_target_.translation()(i), 0.0, 0.0);
                }

                Eigen::Vector3d rotation_cubic_dot;
                x_cubic_.linear() = rotationCubic(sim_time_, control_start_time_, control_start_time_ + duration_time, robot->x_init_.linear(), x_target_.linear());
                rotation_cubic_dot = rotationCubicDot(sim_time_, control_start_time_, control_start_time_ + duration_time, robot->x_init_.linear(), x_target_.linear());

                for (int i = 0; i < 3; i++)
                {
                    x_cubic_dot_(i + 3) = rotation_cubic_dot(i);
                }

                // Eigen::Matrix6d Kp_;
                // Kp_.setZero();
                // for (int i = 0; i < 3; i++)
                // {
                //     Kp_(i,i) = 1600;    //2000
                //     Kp_(i+3, i+3) = 300;     //400
                // } 

                Eigen::Vector6d x_error_;
                x_error_ << x_cubic_.translation() - robot->x_.translation(), -getPhi(robot->x_.linear(), x_cubic_.linear());

                Eigen::Vector7d q_desired_;
                q_desired_ = robot->q_ + j_inverse_ * (x_cubic_dot_ + Kp_* x_error_)/2000.0;
                
                for (int i = 0; i < DOF; i++)
                {
                    robot->control_input_(i) = q_desired_(i);
                }
                robot->grasp_input_ << 0.0, 0.0;
            }
            else if (robot->id_ == "left_arm")
            {
                Eigen::Isometry3d x_target_;
                x_target_.translation() << 0.600, -0.02, 0.473;
                x_target_.linear() << 0.7071 , -0.7071, 0, -0.7071, -0.7071, 0, 0 ,0, -1;
                double duration_time = 3.0;

                Eigen::MatrixXd j_inverse_;
                j_inverse_.resize(7,6);
                j_inverse_ = robot->j_.transpose()*((robot->j_*robot->j_.transpose()).inverse());

                Eigen::Isometry3d x_cubic_;
                Eigen::Vector6d x_cubic_dot_;
                for (int i = 0; i < 3; i++)
                {
                    x_cubic_.translation()(i) = cubic(sim_time_, control_start_time_, control_start_time_ + duration_time, robot->x_init_.translation()(i), x_target_.translation()(i), 0.0, 0.0);
                    x_cubic_dot_(i) = cubicDot(sim_time_, control_start_time_, control_start_time_ + duration_time, robot->x_init_.translation()(i), x_target_.translation()(i), 0.0, 0.0);
                }

                x_cubic_.linear() = rotationCubic(sim_time_, control_start_time_, control_start_time_ + duration_time, robot->x_init_.linear(), x_target_.linear());
                x_cubic_dot_.tail(3) = rotationCubicDot(sim_time_, control_start_time_, control_start_time_ + duration_time, robot->x_init_.linear(), x_target_.linear());

                Eigen::Vector6d x_error_;
                x_error_ << x_cubic_.translation() - robot->x_.translation(), -getPhi(robot->x_.linear(), x_cubic_.linear());

                Eigen::Vector7d q_desired_;
                q_desired_ = robot->q_ + j_inverse_ * (x_cubic_dot_ + Kp_* x_error_)/2000.0;
                
                for (int i = 0; i < DOF; i++)
                {
                    robot->control_input_(i) = q_desired_(i);
                }
                robot->grasp_input_ << 0.0, 0.0;
            }
        }
        // for (auto &robot: robots_)
        // {
        //     if (robot->id_ == "right_arm")
        //     {
        //         robot->control_input_ <<  0.0172005, 0.236217, 0.0175297, -1.94225, -0.00191525, 0.598789, 0.820841;
        //     }
        //     else if (robot->id_ == "left_arm")
        //     {
        //         robot->control_input_ <<  -0.0173848, 0.236217, -0.0160665, -1.94228 , 0.00737671, 0.598789, 0.74836;
        //     }
        // }
        std::cout << " q left Pose:  " << left_arm_.q_.transpose() << std::endl;
        std::cout << " q right Pose:  " << right_arm_.q_.transpose() << std::endl;
    }
    else if (mode_ == MODE_3)
    {
        for (auto &robot: robots_)
        {
            if (robot->id_ == "right_arm")
            {

                
                robot->control_input_(6) = -M_PI / 4;


            }
            else if (robot->id_ == "left_arm")
            {
                robot->control_input_(6) = -M_PI / 4;
            }
        }
    }
    else if (mode_ == MODE_4)
    {
        for (auto &robot: robots_)
        {
            if (robot->id_ == "right_arm")
            {
                Eigen::Isometry3d x_target_;
                x_target_.translation() << 0.600, 0.02, 0.636;
                x_target_.linear() << 0.7071 , 0.7071, 0, 0.7071, -0.7071, 0, 0 ,0, -1; 
                double duration_time = 3.0;

                Eigen::MatrixXd j_inverse_;
                j_inverse_.resize(7,6);
                j_inverse_ = robot->j_.transpose()*((robot->j_*robot->j_.transpose()).inverse());

                Eigen::Isometry3d x_cubic_;
                Eigen::Vector6d x_cubic_dot_;
                for (int i = 0; i < 3; i++)
                {
                    x_cubic_.translation()(i) = cubic(sim_time_, control_start_time_, control_start_time_ + duration_time, robot->x_init_.translation()(i), x_target_.translation()(i), 0.0, 0.0);
                    x_cubic_dot_(i) = cubicDot(sim_time_, control_start_time_, control_start_time_ + duration_time, robot->x_init_.translation()(i), x_target_.translation()(i), 0.0, 0.0);
                }

                Eigen::Vector3d rotation_cubic_dot;
                x_cubic_.linear() = rotationCubic(sim_time_, control_start_time_, control_start_time_ + duration_time, robot->x_init_.linear(), x_target_.linear());
                rotation_cubic_dot = rotationCubicDot(sim_time_, control_start_time_, control_start_time_ + duration_time, robot->x_init_.linear(), x_target_.linear());

                for (int i = 0; i < 3; i++)
                {
                    x_cubic_dot_(i + 3) = rotation_cubic_dot(i);
                }

                Eigen::Vector6d x_error_;
                x_error_ << x_cubic_.translation() - robot->x_.translation(), -getPhi(robot->x_.linear(), x_cubic_.linear());

                Eigen::Vector7d q_desired_;
                q_desired_ = robot->q_ + j_inverse_ * (x_cubic_dot_ + Kp_* x_error_)/2000.0;

                std::cout << " q right Pose:  " << robot->q_.transpose() << std::endl;
                
                for (int i = 0; i < DOF; i++)
                {
                    robot->control_input_(i) = q_desired_(i);
                }
            }
            else if (robot->id_ == "left_arm")
            {
                Eigen::Isometry3d x_target_;
                x_target_.translation() << 0.600, -0.02, 0.636;
                x_target_.linear() << 0.7071 , 0.7071, 0, 0.7071, -0.7071, 0, 0 ,0, -1; 
                double duration_time = 3.0;

                Eigen::MatrixXd j_inverse_;
                j_inverse_.resize(7,6);
                j_inverse_ = robot->j_.transpose()*((robot->j_*robot->j_.transpose()).inverse());

                Eigen::Isometry3d x_cubic_;
                Eigen::Vector6d x_cubic_dot_;
                for (int i = 0; i < 3; i++)
                {
                    x_cubic_.translation()(i) = cubic(sim_time_, control_start_time_, control_start_time_ + duration_time, robot->x_init_.translation()(i), x_target_.translation()(i), 0.0, 0.0);
                    x_cubic_dot_(i) = cubicDot(sim_time_, control_start_time_, control_start_time_ + duration_time, robot->x_init_.translation()(i), x_target_.translation()(i), 0.0, 0.0);
                }

                x_cubic_.linear() = rotationCubic(sim_time_, control_start_time_, control_start_time_ + duration_time, robot->x_init_.linear(), x_target_.linear());
                x_cubic_dot_.tail(3) = rotationCubicDot(sim_time_, control_start_time_, control_start_time_ + duration_time, robot->x_init_.linear(), x_target_.linear());


                Eigen::Vector6d x_error_;
                x_error_ << x_cubic_.translation() - robot->x_.translation(), -getPhi(robot->x_.linear(), x_cubic_.linear());

                Eigen::Vector7d q_desired_;
                q_desired_ = robot->q_ + j_inverse_ * (x_cubic_dot_ + Kp_* x_error_)/2000.0;
                
                

                for (int i = 0; i < DOF; i++)
                {
                    robot->control_input_(i) = q_desired_(i);
                }
            }

            
        }        
        std::cout << " q left Pose:  " << left_arm_.q_.transpose() << std::endl;
        std::cout << " q right Pose:  " << right_arm_.q_.transpose() << std::endl;
    }
    else if (mode_ == MODE_5)
    {
        for (auto &robot: robots_)
        {
            if (robot->id_ == "right_arm")
            {
                robot->control_input_ <<  0.0173039, 0.136351, 0.0174178, -1.64991, 6.51859e-05, 0.206557, -0.751266;
            }
            else if (robot->id_ == "left_arm")
            {
                robot->control_input_ <<  -0.0174784, 0.136323, -0.0163918, -1.64994 , 0.00452102, 0.206561, -0.820473;
            }
        }
    }

    else if (mode_ == MODE_SCENE_1)
    {

        for (auto &robot: robots_)
        {
            if (robot->id_ == "right_arm")
            {
                Eigen::Vector6d ft_diff_;
                ft_diff_ = robot->ft_ - robot->ft_fix_;
                
                Eigen::Isometry3d x_target_;
                x_target_.translation() << 0.600, 0.2, 0.636;
                x_target_.linear() << 0.7071 , 0.7071, 0, 0.7071, -0.7071, 0, 0 ,0, -1; 
                double duration_time = 5.00;

                Eigen::MatrixXd j_inverse_;
                j_inverse_.resize(7,6);
                j_inverse_ = robot->j_.transpose()*((robot->j_*robot->j_.transpose()).inverse());

                Eigen::Isometry3d x_cubic_;
                Eigen::Vector6d x_cubic_dot_;
                for (int i = 0; i < 3; i++)
                {
                    x_cubic_.translation()(i) = cubic(sim_time_, control_start_time_, control_start_time_ + duration_time, robot->x_init_.translation()(i), x_target_.translation()(i), 0.0, 0.0);
                    x_cubic_dot_(i) = cubicDot(sim_time_, control_start_time_, control_start_time_ + duration_time, robot->x_init_.translation()(i), x_target_.translation()(i), 0.0, 0.0);
                    robot->x_desired_(i) = x_cubic_.translation()(i);
                }

                Eigen::Vector3d rotation_cubic_dot;
                x_cubic_.linear() = rotationCubic(sim_time_, control_start_time_, control_start_time_ + duration_time, robot->x_init_.linear(), x_target_.linear());
                rotation_cubic_dot = rotationCubicDot(sim_time_, control_start_time_, control_start_time_ + duration_time, robot->x_init_.linear(), x_target_.linear());

                for (int i = 0; i < 3; i++)
                {
                    x_cubic_dot_(i + 3) = rotation_cubic_dot(i);
                }

                Eigen::Vector6d x_error_;
                x_error_ << x_cubic_.translation() - robot->x_.translation(), -getPhi(robot->x_.linear(), x_cubic_.linear());

                Eigen::Vector7d q_desired_;
                q_desired_ = robot->q_ + j_inverse_ * (x_cubic_dot_ + Kp_* x_error_)/2000.0;

                
                for (int i = 0; i < DOF; i++)
                {

                    robot->control_input_(i) = q_desired_(i);        
 
                }
            }
            else if (robot->id_ == "left_arm")
            {
                Eigen::Vector6d ft_diff_;
                ft_diff_ = robot->ft_ - robot->ft_fix_;
                
                Eigen::Isometry3d x_target_;
                x_target_.translation() << 0.600, 0.16, 0.636;
                x_target_.linear() << 0.7071 , 0.7071, 0, 0.7071, -0.7071, 0, 0 ,0, -1; 
                double duration_time = 5.00;

                Eigen::MatrixXd j_inverse_;
                j_inverse_.resize(7,6);
                j_inverse_ = robot->j_.transpose()*((robot->j_*robot->j_.transpose()).inverse());

                Eigen::Isometry3d x_cubic_;
                Eigen::Vector6d x_cubic_dot_;
                for (int i = 0; i < 3; i++)
                {
                    x_cubic_.translation()(i) = cubic(sim_time_, control_start_time_, control_start_time_ + duration_time, robot->x_init_.translation()(i), x_target_.translation()(i), 0.0, 0.0);
                    x_cubic_dot_(i) = cubicDot(sim_time_, control_start_time_, control_start_time_ + duration_time, robot->x_init_.translation()(i), x_target_.translation()(i), 0.0, 0.0);
                    robot->x_desired_(i) = x_cubic_.translation()(i);
                }

                Eigen::Vector3d rotation_cubic_dot;
                x_cubic_.linear() = rotationCubic(sim_time_, control_start_time_, control_start_time_ + duration_time, robot->x_init_.linear(), x_target_.linear());
                rotation_cubic_dot = rotationCubicDot(sim_time_, control_start_time_, control_start_time_ + duration_time, robot->x_init_.linear(), x_target_.linear());

                for (int i = 0; i < 3; i++)
                {
                    x_cubic_dot_(i + 3) = rotation_cubic_dot(i);
                }

                Eigen::Vector6d x_error_;
                x_error_ << x_cubic_.translation() - robot->x_.translation(), -getPhi(robot->x_.linear(), x_cubic_.linear());

                Eigen::Vector7d q_desired_;
                q_desired_ = robot->q_ + j_inverse_ * (x_cubic_dot_ + Kp_* x_error_)/2000.0;

                
                for (int i = 0; i < DOF; i++)
                {

                    robot->control_input_(i) = q_desired_(i);        
 
                }
            }
        }
    }
    else if (mode_ == MODE_SCENE_2)
    {

        for (auto &robot: robots_)
        {
            if (robot->id_ == "right_arm")
            {
                Eigen::Vector6d ft_diff_;
                ft_diff_ = robot->ft_ - robot->ft_fix_;
                
                Eigen::Isometry3d x_target_;
                x_target_.translation() << 0.600, 0.2, 0.636;
                x_target_.linear() << 0.7071 , 0.7071, 0, 0.7071, -0.7071, 0, 0 ,0, -1; 
                double duration_time = 5.00;

                Eigen::MatrixXd j_inverse_;
                j_inverse_.resize(7,6);
                j_inverse_ = robot->j_.transpose()*((robot->j_*robot->j_.transpose()).inverse());

                Eigen::Isometry3d x_cubic_;
                Eigen::Vector6d x_cubic_dot_;
                for (int i = 0; i < 3; i++)
                {
                    x_cubic_.translation()(i) = cubic(sim_time_, control_start_time_, control_start_time_ + duration_time, robot->x_init_.translation()(i), x_target_.translation()(i), 0.0, 0.0);
                    x_cubic_dot_(i) = cubicDot(sim_time_, control_start_time_, control_start_time_ + duration_time, robot->x_init_.translation()(i), x_target_.translation()(i), 0.0, 0.0);
                    robot->x_desired_(i) = x_cubic_.translation()(i);
                }

                Eigen::Vector3d rotation_cubic_dot;
                x_cubic_.linear() = rotationCubic(sim_time_, control_start_time_, control_start_time_ + duration_time, robot->x_init_.linear(), x_target_.linear());
                rotation_cubic_dot = rotationCubicDot(sim_time_, control_start_time_, control_start_time_ + duration_time, robot->x_init_.linear(), x_target_.linear());

                for (int i = 0; i < 3; i++)
                {
                    x_cubic_dot_(i + 3) = rotation_cubic_dot(i);
                }

                Eigen::Vector6d x_error_;
                x_error_ << x_cubic_.translation() - robot->x_.translation(), -getPhi(robot->x_.linear(), x_cubic_.linear());

                Eigen::Vector7d q_desired_;
                q_desired_ = robot->q_ + j_inverse_ * (x_cubic_dot_ + Kp_* x_error_)/2000.0;

                
                for (int i = 0; i < DOF; i++)
                {

                    robot->control_input_(i) = q_desired_(i);        
 
                }
            }
            else if (robot->id_ == "left_arm")
            {
                // Admittance Control or Assist Control

                Eigen::Vector6d ft_diff_;
                Eigen::Vector6d ft_diff_prev_;

                
                for (int i =0; i < 3; i++)
                {
                    ft_diff_(i) = robot->ft_(i) - robot->ft_fix_(i);
                    ft_diff_prev_(i) = robot->ft_before_(i) - robot->ft_fix_(i);
                    ft_diff_(i) = lowPassFilter(ft_diff_(i), ft_diff_prev_(i), 1/2000.0, 1);
                    if (ft_diff_(i) < 0.1)
                    {
                        ft_diff_(i) = 0;
                    }
                }

                double mass_(4);
                double damping_(100);
                double stiff_(0.00008);

                Eigen::Isometry3d x_target_;
                x_target_.translation() << 0.600, -0.02, 0.636;
                x_target_.linear() << 0.7071 , 0.7071, 0, 0.7071, -0.7071, 0, 0 ,0, -1; 
                double duration_time = 5.00;

                Eigen::MatrixXd j_inverse_;
                j_inverse_.resize(7,6);
                j_inverse_ = robot->j_.transpose()*((robot->j_*robot->j_.transpose()).inverse());
  
                Eigen::Isometry3d x_cubic_;
                Eigen::Vector6d x_cubic_dot_;

                for (int i = 0; i < 3; i++)
                {
                    x_cubic_.translation()(i) = cubic(sim_time_, control_start_time_, control_start_time_ + duration_time, robot->x_init_.translation()(i), x_target_.translation()(i), 0.0, 0.0);
                    x_cubic_dot_(i) = cubicDot(sim_time_, control_start_time_, control_start_time_ + duration_time, robot->x_init_.translation()(i), x_target_.translation()(i), 0.0, 0.0);
                    robot->x_desired_(i) = x_cubic_.translation()(i);
                }

                for (int i = 1; i < 2; i++)
                {                  
                    x_cubic_dot_(i) = ( ft_diff_(i) / 2000.0 + mass_*robot->x_dot_(i) ) / ( mass_ + damping_ / 2000.0 ) ; 
                    x_cubic_.translation()(i) = robot->x_.translation()(i) + x_cubic_dot_(i) / 2000.0;               
                }

                x_cubic_.linear() = rotationCubic(sim_time_, control_start_time_, control_start_time_ + duration_time, robot->x_init_.linear(), x_target_.linear());
                x_cubic_dot_.tail(3) = rotationCubicDot(sim_time_, control_start_time_, control_start_time_ + duration_time, robot->x_init_.linear(), x_target_.linear());


                Eigen::Vector6d x_error_;
                x_error_ << x_cubic_.translation() - robot->x_.translation(), -getPhi(robot->x_.linear(), x_cubic_.linear());

                Eigen::Vector7d q_desired_;
                q_desired_ = robot->q_ + j_inverse_ * (x_cubic_dot_ + Kp_* x_error_)/2000.0;
                // q_desired_ = robot->q_ + j_inverse_ * (Kp_* x_error_)/2000.0;

                std::cout << " follower ft diff :  " << ft_diff_.transpose()  << std::endl;
                std::cout << " Desired X ad Pose:  " << x_cubic_.translation().transpose() << std::endl;
                
                for (int i = 0; i < DOF; i++)
                {

                    robot->control_input_(i) = q_desired_(i);        

                }
            }
        }
    }

    else if (mode_ == MODE_SCENE_3)
    {

        for (auto &robot: robots_)
        {
            if (robot->id_ == "right_arm")
            {
                Eigen::Vector6d ft_diff_;
                ft_diff_ = robot->ft_ - robot->ft_fix_;
                
                Eigen::Isometry3d x_target_;
                x_target_.translation() << 0.600, 0.2, 0.636;
                x_target_.linear() << 0.7071 , 0.7071, 0, 0.7071, -0.7071, 0, 0 ,0, -1; 
                double duration_time = 5.00;

                Eigen::MatrixXd j_inverse_;
                j_inverse_.resize(7,6);
                j_inverse_ = robot->j_.transpose()*((robot->j_*robot->j_.transpose()).inverse());

                Eigen::Isometry3d x_cubic_;
                Eigen::Vector6d x_cubic_dot_;
                for (int i = 0; i < 3; i++)
                {
                    x_cubic_.translation()(i) = cubic(sim_time_, control_start_time_, control_start_time_ + duration_time, robot->x_init_.translation()(i), x_target_.translation()(i), 0.0, 0.0);
                    x_cubic_dot_(i) = cubicDot(sim_time_, control_start_time_, control_start_time_ + duration_time, robot->x_init_.translation()(i), x_target_.translation()(i), 0.0, 0.0);
                    robot->x_desired_(i) = x_cubic_.translation()(i);
                }

                Eigen::Vector3d rotation_cubic_dot;
                x_cubic_.linear() = rotationCubic(sim_time_, control_start_time_, control_start_time_ + duration_time, robot->x_init_.linear(), x_target_.linear());
                rotation_cubic_dot = rotationCubicDot(sim_time_, control_start_time_, control_start_time_ + duration_time, robot->x_init_.linear(), x_target_.linear());

                for (int i = 0; i < 3; i++)
                {
                    x_cubic_dot_(i + 3) = rotation_cubic_dot(i);
                }

                Eigen::Vector6d x_error_;
                x_error_ << x_cubic_.translation() - robot->x_.translation(), -getPhi(robot->x_.linear(), x_cubic_.linear());

                Eigen::Vector7d q_desired_;
                q_desired_ = robot->q_ + j_inverse_ * (x_cubic_dot_ + Kp_* x_error_)/2000.0;

                
                for (int i = 0; i < DOF; i++)
                {

                    robot->control_input_(i) = q_desired_(i);        
 
                }
            }
            else if (robot->id_ == "left_arm")
            {
                // Admittance Control or Assist Control

                Eigen::Vector6d ft_diff_;
                Eigen::Vector6d ft_diff_prev_;
                
                for (int i =0; i < 3; i++)
                {
                    ft_diff_(i) = robot->ft_(i) - robot->ft_fix_(i);
                    ft_diff_prev_(i) = robot->ft_before_(i) - robot->ft_fix_(i);
                    ft_diff_(i) = lowPassFilter(ft_diff_(i), ft_diff_prev_(i), 1/2000.0, 1);
                    if (ft_diff_(i) < 0.1)
                    {
                        ft_diff_(i) = 0;
                    }
                }
                
                double mass_(4);
                double damping_(100);
                double stiff_(0.00008);
                double alpha_(3500);
                // P_ = M(xdot_f[i]-xdot_f[i-1]) - F_des_*dt
                // F_des_ = F_des + alpha_*P_
                Eigen::Vector6d P_as_;
                P_as_(1) = mass_ob_*(robot->x_dot_(1) - robot->x_dot_before_(1)) - robot->control_assist_(1)/2000.0;
                robot->control_assist_(1) = robot->control_assist_(1) + alpha_*P_as_(1);

                Eigen::Isometry3d x_target_;
                x_target_.translation() << 0.600, -0.02, 0.636;
                x_target_.linear() << 0.7071 , 0.7071, 0, 0.7071, -0.7071, 0, 0 ,0, -1; 
                double duration_time = 5.00;

                Eigen::MatrixXd j_inverse_;
                j_inverse_.resize(7,6);
                j_inverse_ = robot->j_.transpose()*((robot->j_*robot->j_.transpose()).inverse());
  
                Eigen::Isometry3d x_cubic_;
                Eigen::Vector6d x_cubic_dot_;

                for (int i = 0; i < 3; i++)
                {
                    x_cubic_.translation()(i) = cubic(sim_time_, control_start_time_, control_start_time_ + duration_time, robot->x_init_.translation()(i), x_target_.translation()(i), 0.0, 0.0);
                    x_cubic_dot_(i) = cubicDot(sim_time_, control_start_time_, control_start_time_ + duration_time, robot->x_init_.translation()(i), x_target_.translation()(i), 0.0, 0.0);
                    robot->x_desired_(i) = x_cubic_.translation()(i);
                }

                for (int i = 1; i < 2; i++)
                {                  
                    x_cubic_dot_(i) = ( robot->control_assist_(i) / 2000.0 + mass_*robot->x_dot_(i) ) / ( mass_ + damping_ / 2000.0 ) ; 
                    x_cubic_.translation()(i) = robot->x_.translation()(i) + x_cubic_dot_(i) / 2000.0;               
                }

                x_cubic_.linear() = rotationCubic(sim_time_, control_start_time_, control_start_time_ + duration_time, robot->x_init_.linear(), x_target_.linear());
                x_cubic_dot_.tail(3) = rotationCubicDot(sim_time_, control_start_time_, control_start_time_ + duration_time, robot->x_init_.linear(), x_target_.linear());

                // Eigen::Matrix6d Kp_;
                // Kp_.setZero();
                // for (int i = 0; i < 3; i++)
                // {
                //     Kp_(i,i) = 1600;    //2000
                //     Kp_(i+3, i+3) = 300;     //400
                // } 

                Eigen::Vector6d x_error_;
                x_error_ << x_cubic_.translation() - robot->x_.translation(), -getPhi(robot->x_.linear(), x_cubic_.linear());

                Eigen::Vector7d q_desired_;
                q_desired_ = robot->q_ + j_inverse_ * (x_cubic_dot_ + Kp_* x_error_)/2000.0;
                // q_desired_ = robot->q_ + j_inverse_ * (Kp_* x_error_)/2000.0;

                std::cout << " follower ft diff :  " << ft_diff_.transpose()  << std::endl;
                std::cout << " Desired X ad Pose:  " << x_cubic_.translation().transpose() << std::endl;
                
                for (int i = 0; i < DOF; i++)
                {

                    robot->control_input_(i) = q_desired_(i);        

                }
            }
        }
    }
    
    else if (mode_ == MODE_SCENE_4)
    {
        
        for (auto &robot: robots_)
        {
            if (robot->id_ == "right_arm")
            {
                Eigen::Vector6d ft_diff_;
                ft_diff_ = robot->ft_ - robot->ft_fix_;
                
                Eigen::Isometry3d x_target_;
                x_target_.translation() << 0.600, -0.2, 0.636;
                x_target_.linear() << 0.7071 , 0.7071, 0, 0.7071, -0.7071, 0, 0 ,0, -1; 
                double duration_time = 5.00;

                Eigen::MatrixXd j_inverse_;
                j_inverse_.resize(7,6);
                j_inverse_ = robot->j_.transpose()*((robot->j_*robot->j_.transpose()).inverse());

                Eigen::Isometry3d x_cubic_;
                Eigen::Vector6d x_cubic_dot_;
                for (int i = 0; i < 3; i++)
                {
                    x_cubic_.translation()(i) = cubic(sim_time_, control_start_time_, control_start_time_ + duration_time, robot->x_init_.translation()(i), x_target_.translation()(i), 0.0, 0.0);
                    x_cubic_dot_(i) = cubicDot(sim_time_, control_start_time_, control_start_time_ + duration_time, robot->x_init_.translation()(i), x_target_.translation()(i), 0.0, 0.0);
                    robot->x_desired_(i) = x_cubic_.translation()(i);
                }

                Eigen::Vector3d rotation_cubic_dot;
                x_cubic_.linear() = rotationCubic(sim_time_, control_start_time_, control_start_time_ + duration_time, robot->x_init_.linear(), x_target_.linear());
                rotation_cubic_dot = rotationCubicDot(sim_time_, control_start_time_, control_start_time_ + duration_time, robot->x_init_.linear(), x_target_.linear());

                for (int i = 0; i < 3; i++)
                {
                    x_cubic_dot_(i + 3) = rotation_cubic_dot(i);
                }

                Eigen::Vector6d x_error_;
                x_error_ << x_cubic_.translation() - robot->x_.translation(), -getPhi(robot->x_.linear(), x_cubic_.linear());

                Eigen::Vector7d q_desired_;
                q_desired_ = robot->q_ + j_inverse_ * (x_cubic_dot_ + Kp_* x_error_)/2000.0;

                
                for (int i = 0; i < DOF; i++)
                {

                    robot->control_input_(i) = q_desired_(i);        

                }
            }
            else if (robot->id_ == "left_arm")
            {
                // Admittance Control or Assist Control

                Eigen::Vector6d ft_diff_;
                Eigen::Vector6d ft_diff_prev_;

                
                for (int i =0; i < 3; i++)
                {
                    ft_diff_(i) = robot->ft_(i) - robot->ft_fix_(i);
                    ft_diff_prev_(i) = robot->ft_before_(i) - robot->ft_fix_(i);
                    ft_diff_(i) = lowPassFilter(ft_diff_(i), ft_diff_prev_(i), 1/2000.0, 1);
                    if (ft_diff_(i) < 0.1)
                    {
                        ft_diff_(i) = 0;
                    }
                }
                
                double mass_(4);
                double damping_(100);
                double stiff_(0.00008);
                double alpha2_(3500);
                // P_ = M(xdot_f[i]-xdot_f[i-1]) - F_des_*dt
                // F_des_ = F_des + alpha_*P_
                Eigen::Vector6d P_as_;
                P_as_(1) = mass_ob_*(robot->x_dot_(1) - robot->x_dot_before_(1)) - robot->control_assist_(1)/2000.0;
                robot->control_assist_(1) = robot->control_assist_(1) + alpha2_*P_as_(1);

                Eigen::Isometry3d x_target_;
                x_target_.translation() << 0.600, -0.02, 0.636;
                x_target_.linear() << 0.7071 , 0.7071, 0, 0.7071, -0.7071, 0, 0 ,0, -1; 
                double duration_time = 5.00;

                Eigen::MatrixXd j_inverse_;
                j_inverse_.resize(7,6);
                j_inverse_ = robot->j_.transpose()*((robot->j_*robot->j_.transpose()).inverse());

                Eigen::Isometry3d x_cubic_;
                Eigen::Vector6d x_cubic_dot_;

                for (int i = 0; i < 3; i++)
                {
                    x_cubic_.translation()(i) = cubic(sim_time_, control_start_time_, control_start_time_ + duration_time, robot->x_init_.translation()(i), x_target_.translation()(i), 0.0, 0.0);
                    x_cubic_dot_(i) = cubicDot(sim_time_, control_start_time_, control_start_time_ + duration_time, robot->x_init_.translation()(i), x_target_.translation()(i), 0.0, 0.0);
                    robot->x_desired_(i) = x_cubic_.translation()(i);
                }

                for (int i = 1; i < 2; i++)
                {                  
                    x_cubic_dot_(i) = ( robot->control_assist_(i) / 2000.0 + mass_*robot->x_dot_(i) ) / ( mass_ + damping_ / 2000.0 ) ; 
                    x_cubic_.translation()(i) = robot->x_.translation()(i) + x_cubic_dot_(i) / 2000.0;   
                    
                    // x_cubic_dot_(i) = ( ft_diff_(i) / 2000.0 + mass_*robot->x_dot_(i) ) / ( mass_ + damping_ / 2000.0 ) ; 
                    // x_cubic_.translation()(i) = robot->x_.translation()(i) + x_cubic_dot_(i) / 2000.0;               
                }

                x_cubic_.linear() = rotationCubic(sim_time_, control_start_time_, control_start_time_ + duration_time, robot->x_init_.linear(), x_target_.linear());
                x_cubic_dot_.tail(3) = rotationCubicDot(sim_time_, control_start_time_, control_start_time_ + duration_time, robot->x_init_.linear(), x_target_.linear());

                Eigen::Vector6d x_error_;
                x_error_ << x_cubic_.translation() - robot->x_.translation(), -getPhi(robot->x_.linear(), x_cubic_.linear());

                Eigen::Vector7d q_desired_;
                q_desired_ = robot->q_ + j_inverse_ * (x_cubic_dot_ + Kp_* x_error_)/2000.0;

                // std::cout << " follower ft diff :  " << ft_diff_.transpose()  << std::endl;
                // std::cout << " Desired X ad Pose:  " << x_cubic_.translation().transpose() << std::endl;
                
                for (int i = 0; i < DOF; i++)
                {

                    robot->control_input_(i) = q_desired_(i);        

                }
            }
        }

    }
    else
    {
        for (auto &robot: robots_)
        {
            if (robot->id_ == "right_arm")
            {
                robot->control_input_ << 0.0, 0.0, 0.0, -M_PI / 2., 0.0, 0, M_PI / 4;
            }
            else if (robot->id_ == "left_arm")
            {
                robot->control_input_ << 0.0, 0.0, 0.0, -M_PI / 2., 0.0, 0, M_PI / 4;
            }
        }
    }
             
   
    m_ci_.lock();
    dc_.control_input_.head(DOF) = right_arm_.control_input_;
    dc_.control_input_.segment(DOF+2, DOF) = left_arm_.control_input_;
    dc_.control_input_.segment(DOF, 2) = right_arm_.grasp_input_;
    dc_.control_input_.segment(DOF+DOF+2, 2) = left_arm_.grasp_input_;
    m_ci_.unlock();
}

