#include <mutex>
#include <Eigen/Dense>
#include <ros/ros.h>
#include <ros/package.h>
#include <fstream>

#include "mujoco_ros_msgs/JointSet.h"

#include "panda_controller/mujoco_interface.h"

#include <rbdl/rbdl.h>
#include <rbdl/addons/urdfreader/urdfreader.h>

#include "panda_controller/util.h"

#define DOF 7
# define MODE_INIT 105   // I
# define MODE_HOME 104   // H
// # define MODE_RANDOM 114    // R
# define MODE_FORCE 102     // F
# define MODE_STOP 115      // S
# define MODE_1 113 //q
# define MODE_2 119 //w
# define MODE_3 101 //e
# define MODE_4 114 //r
# define MODE_5 116 //t
# define MODE_SCENE_1 122 //z
# define MODE_SCENE_2 120 //x
# define MODE_SCENE_3 99 //c
# define MODE_SCENE_4 118 //v

class PandaController{
    public:
        PandaController(ros::NodeHandle &nh, DataContainer &dc);
        ~PandaController();
        void compute();
        void updateKinematicsDynamics();
        void computeControlInput();

    private:
        std::mutex m_dc_;
        std::mutex m_ci_;
        DataContainer &dc_;

        std::ofstream txt_right_;
        std::ofstream txt_left_;

        bool is_init_ = false;
        int mode_;

        double sim_time_ = 0.0;
        double control_start_time_ = 0.0;
        double mass_ob_ = 10.0;

        // Robot State
        struct RobotState
        {
            std::string id_;

            Eigen::Vector7d q_;
            Eigen::Vector7d q_desired;
            Eigen::Vector7d q_init_;
            Eigen::Vector7d q_dot_;
            Eigen::Vector7d q_dot_zero_;
            Eigen::Vector7d effort_;
            Eigen::Vector6d ft_;
            Eigen::Vector6d ft_before_;
            Eigen::Vector6d ft_fix_;

            Eigen::Vector3d x_desired_;
            Eigen::Vector3d xa_desired;

            // Kinematics & Dynamics
            RigidBodyDynamics::Model robot_model;
            RigidBodyDynamics::Model &robot_model_ = robot_model;

            Eigen::MatrixXd A_;
            Eigen::VectorXd g_;

            Eigen::Isometry3d x_;
            Eigen::Vector6d x_dot_;
            Eigen::Vector6d x_dot_before_;
            Eigen::Vector3d x_angle_;
            Eigen::MatrixXd j_temp_;
            Eigen::MatrixXd j_;

            Eigen::Vector7d control_input_;
            Eigen::VectorXd grasp_input_;
            Eigen::Vector6d control_assist_;
            Eigen::Isometry3d x_init_;
        };
        RobotState right_arm_;
        RobotState left_arm_;

        std::vector<RobotState*> robots_; 
};