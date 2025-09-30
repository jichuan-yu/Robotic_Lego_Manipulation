#pragma once
#include "Lego.hpp"
#include <ros/ros.h>
#include <string>
#include <std_msgs/Int32.h>
#include "gazebo_ros_link_attacher/Attach.h"


using namespace std;

enum class RobotMode : int {
    Idle = 0,
    Moving = 1,
    Picking = 2
};
enum class RobotID { Robot1, Robot2 };

class ExpertDemo {
public:
    ExpertDemo(ros::NodeHandle& nh, ros::NodeHandle& pnh);

    void init(); // Load config an initialize

    void reset_brick(const string& brick_name, const int& orientation,
                        const int& brick_loc_x, const int& brick_loc_y, const int& brick_loc_z);

    Eigen::Matrix4d random_perturbation(double pos_std, double rot_std_deg);
    // Robot Skills
    void homing(RobotID robot_id);
    void moveJoint(RobotID robot_id, lego_manipulation::math::VectorJd& q);
    void pick(RobotID robot_id, const std::string& brick_name, int press_side, int press_offset);
    
    void attach_brick(RobotID robot_id, const std::string& brick_name);
    void detach_brick(RobotID robot_id, const std::string& brick_name);
private:
    ros::NodeHandle nh_, private_nh_;
    lego_manipulation::lego::Lego::Ptr lego_ptr_;
    Json::Value config_, task_json_;
    ros::ServiceClient set_state_client_;  // Gazebo set model state client 
    // gazebo_ros_link_attacher plugin
    ros::ServiceClient attach_client_ = nh_.serviceClient<gazebo_ros_link_attacher::Attach>("/link_attacher_node/attach");
    ros::ServiceClient detach_client_ = nh_.serviceClient<gazebo_ros_link_attacher::Attach>("/link_attacher_node/detach");
    
    ros::Publisher r1_goal_pub_, r2_goal_pub_, r1_controller_time_pub_, r2_controller_time_pub_;
    ros::Subscriber r1_robot_state_sub_, r2_robot_state_sub_;

    ros::Publisher robot2_status_pub_, robot2_mode_pub_; // Robot mode publisher
    RobotMode robot1_mode_ = RobotMode::Idle;
    RobotMode robot2_mode_ = RobotMode::Idle;
    
    int control_rate_ = 10; // Hz
    double jpc_travel_time_;

    // Robot states
    void pubJointGoal(RobotID robot_id, const lego_manipulation::math::VectorJd& q);
    double EPS_ = 1e-5; // Threshold to determine if the robot has reached the goal
    bool reachGoal(lego_manipulation::math::VectorJd& current_q, lego_manipulation::math::VectorJd& goal_q);
    void robot1StateCallback(const std_msgs::Float32MultiArray::ConstPtr& msg);
    void robot2StateCallback(const std_msgs::Float32MultiArray::ConstPtr& msg);
    lego_manipulation::math::VectorJd robot1_q_ = Eigen::MatrixXd::Zero(6, 1);
    lego_manipulation::math::VectorJd robot1_qd_ = Eigen::MatrixXd::Zero(6, 1);
    lego_manipulation::math::VectorJd robot1_qdd_ = Eigen::MatrixXd::Zero(6, 1);
    lego_manipulation::math::VectorJd robot2_q_ = Eigen::MatrixXd::Zero(6, 1);
    lego_manipulation::math::VectorJd robot2_qd_ = Eigen::MatrixXd::Zero(6, 1);
    lego_manipulation::math::VectorJd robot2_qdd_ = Eigen::MatrixXd::Zero(6, 1);

    // Predefined positions
    lego_manipulation::math::VectorJd home_q_ = (Eigen::MatrixXd(6, 1) << 0, 0, 0, 0, -90, 0).finished();
    lego_manipulation::math::VectorJd zero_q_ = (Eigen::MatrixXd(6, 1) << 0, 0, 0, 0, 0, 0).finished();
    lego_manipulation::math::VectorJd home_receive_q_ = (Eigen::MatrixXd(6, 1) << 0, 0, 0, 0, 0, 180).finished();
    lego_manipulation::math::VectorJd receive_q_ = (Eigen::MatrixXd(6, 1) << 0, 0, 0, 0, 0, 180).finished();
    lego_manipulation::math::VectorJd home_handover_q_ = (Eigen::MatrixXd(6, 1) << 0, 0, 0, 0, -90, 0).finished();


    // Predefined transforms
    Eigen::Matrix4d y_n90_ = (Eigen::Matrix4d() << 
        0, 0, -1, 0, 
        0, 1, 0, 0,
        1, 0, 0, 0,
        0, 0, 0, 1).finished();
    Eigen::Matrix4d y_p90_ = (Eigen::Matrix4d() << 
        0, 0, 1, 0,
        0, 1, 0, 0,
        -1, 0, 0, 0,
        0, 0, 0, 1).finished();
    Eigen::Matrix4d z_180_ = (Eigen::Matrix4d() << 
        -1, 0, 0, 0,
        0, -1, 0, 0,
        0, 0, 1, 0,
        0, 0, 0, 1).finished();

    Eigen::Matrix4d support_T_ = Eigen::Matrix4d::Identity();
    Eigen::Matrix4d support_T_down_ = Eigen::Matrix4d::Identity();
    Eigen::Matrix4d support_T_down_pre_ = Eigen::Matrix4d::Identity();
    Eigen::Matrix4d press_T_ = Eigen::Matrix4d::Identity();
    Eigen::Matrix4d press_up_T_ = Eigen::Matrix4d::Identity();
    Eigen::MatrixXd twist_R_ = Eigen::MatrixXd::Identity(3, 3);
    Eigen::MatrixXd twist_T_ = Eigen::MatrixXd::Identity(4, 4);
    int twist_deg_ = 0;

};







