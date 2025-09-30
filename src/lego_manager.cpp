// #include "lego_manager.hpp"

// LegoManager::LegoManager(ros::NodeHandle& nh)
//     : nh_(nh)
// {
//     get_pose_srv_ = nh_.advertiseService("get_brick_pose", &LegoManager::getBrickPoseCallback, this);
//     set_pose_srv_ = nh_.advertiseService("set_brick_pose", &LegoManager::setBrickPoseCallback, this);

//     attach_client_ = nh_.serviceClient<gazebo_ros_link_attacher::Attach>("/link_attacher_node/attach");
//     detach_client_ = nh_.serviceClient<gazebo_ros_link_attacher::Detach>("/link_attacher_node/detach");

//     lego_ptr_ = std::make_shared<Lego>();
//     // 可在此加载配置文件等
// }

// bool LegoManager::getBrickPoseCallback(GetBrickPose::Request& req, GetBrickPose::Response& res) {
//     const auto& brick_map = lego_ptr_->get_brick_map();
//     auto it = brick_map.find(req.brick_name);
//     if (it == brick_map.end()) return false;
//     lego_brick& brick = it->second;
//     res.pose.position.x = brick.cur_x;
//     res.pose.position.y = brick.cur_y;
//     res.pose.position.z = brick.cur_z;
//     res.pose.orientation.x = brick.cur_quat.x();
//     res.pose.orientation.y = brick.cur_quat.y();
//     res.pose.orientation.z = brick.cur_quat.z();
//     res.pose.orientation.w = brick.cur_quat.w();
//     return true;
// }

// bool LegoManager::setBrickPoseCallback(SetBrickPose::Request& req, SetBrickPose::Response& res) {
//     auto& brick_map = lego_ptr_->get_brick_map();
//     auto it = brick_map.find(req.brick_name);
//     if (it == brick_map.end()) {
//         res.success = false;
//         return true;
//     }
//     lego_brick& brick = it->second;
//     brick.cur_x = req.pose.position.x;
//     brick.cur_y = req.pose.position.y;
//     brick.cur_z = req.pose.position.z;
//     brick.cur_quat.x() = req.pose.orientation.x;
//     brick.cur_quat.y() = req.pose.orientation.y;
//     brick.cur_quat.z() = req.pose.orientation.z;
//     brick.cur_quat.w() = req.pose.orientation.w;
//     // 可同步到 Gazebo
//     res.success = true;
//     return true;
// }

// bool LegoManager::attachBrickToRobot(const std::string& brick_name, const std::string& robot_name, const std::string& brick_link, const std::string& robot_link) {
//     gazebo_ros_link_attacher::Attach srv;
//     srv.request.model_name_1 = brick_name;
//     srv.request.link_name_1 = brick_link;
//     srv.request.model_name_2 = robot_name;
//     srv.request.link_name_2 = robot_link;
//     return attach_client_.call(srv) && srv.response.ok;
// }

// bool LegoManager::detachBrickFromRobot(const std::string& brick_name, const std::string& robot_name, const std::string& brick_link, const std::string& robot_link) {
//     gazebo_ros_link_attacher::Detach srv;
//     srv.request.model_name_1 = brick_name;
//     srv.request.link_name_1 = brick_link;
//     srv.request.model_name_2 = robot_name;
//     srv.request.link_name_2 = robot_link;
//     return detach_client_.call(srv) && srv.response.ok;
// }

// int main(int argc, char** argv) {
//     ros::init(argc, argv, "lego_manager_node");
//     ros::NodeHandle nh;
//     LegoManager manager(nh);
//     ros::spin();
//     return 0;
// }































