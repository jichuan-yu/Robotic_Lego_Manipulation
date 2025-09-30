// #pragma once
// #include <ros/ros.h>
// #include <geometry_msgs/Pose.h>
// #include <gazebo_ros_link_attacher/Attach.h>
// #include "Lego.hpp"
// #include "GetBrickPose.srv"
// #include "SetBrickPose.srv"

// using namespace lego_manipulation::lego;

// class LegoManager {
// public:
//     LegoManager(ros::NodeHandle& nh);

//     // 获取积木位姿
//     bool getBrickPoseCallback(GetBrickPose::Request& req, GetBrickPose::Response& res);

//     // 设置积木位姿
//     bool setBrickPoseCallback(SetBrickPose::Request& req, SetBrickPose::Response& res);

//     // attach brick to robot
//     bool attachBrickToRobot(const std::string& brick_name, const std::string& robot_name, const std::string& brick_link, const std::string& robot_link);

//     // detach brick from robot
//     bool detachBrickFromRobot(const std::string& brick_name, const std::string& robot_name, const std::string& brick_link, const std::string& robot_link);

// private:
//     ros::NodeHandle nh_;
//     ros::ServiceServer get_pose_srv_, set_pose_srv_;
//     ros::ServiceClient attach_client_, detach_client_;
//     Lego::Ptr lego_ptr_;
// };