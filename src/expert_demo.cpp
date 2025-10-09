#include "expert_demo.hpp"
#include <random>

ExpertDemo::ExpertDemo(ros::NodeHandle& nh, ros::NodeHandle& pnh)
    : nh_(nh), private_nh_(pnh)
{
}

void ExpertDemo::init()
{
    try{

        std::string config_fname, root_pwd, task_fname, world_base_fname;
        std::string r1_DH_fname, r1_DH_tool_fname, r1_DH_tool_assemble_fname, r1_DH_tool_disassemble_fname, r1_DH_tool_alt_fname, r1_DH_tool_alt_assemble_fname, r1_robot_base_fname;
        std::string r2_DH_fname, r2_DH_tool_fname, r2_DH_tool_assemble_fname, r2_DH_tool_disassemble_fname, r2_DH_tool_alt_fname, r2_DH_tool_alt_assemble_fname, r2_robot_base_fname;
        std::string lego_lib_fname, env_setup_fname;
        std::string r1_robot_state_topic, r1_controller_joint_goal_topic, r1_controller_cart_goal_topic, r1_controller_time_topic;
        std::string r2_robot_state_topic, r2_controller_joint_goal_topic, r2_controller_cart_goal_topic, r2_controller_time_topic;
        std::string robot1_name, robot2_name;

        ros::ServiceClient set_state_client = nh_.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");
        private_nh_.getParam("config_fname", config_fname);
        private_nh_.getParam("root_pwd", root_pwd);

        std::ifstream config_file(config_fname, std::ifstream::binary);
        Json::Value config;
        config_file >> config;
        world_base_fname = root_pwd + config["world_base_fname"].asString();
        r1_DH_fname = root_pwd + config["r1_DH_fname"].asString();
        r1_DH_tool_fname = root_pwd + config["r1_DH_tool_fname"].asString();
        r1_DH_tool_assemble_fname = root_pwd + config["r1_DH_tool_assemble_fname"].asString();
        r1_DH_tool_disassemble_fname = root_pwd + config["r1_DH_tool_disassemble_fname"].asString();
        r1_DH_tool_alt_fname = root_pwd + config["r1_DH_tool_alt_fname"].asString();
        r1_DH_tool_alt_assemble_fname = root_pwd + config["r1_DH_tool_alt_assemble_fname"].asString();
        r1_robot_base_fname = root_pwd + config["Robot1_Base_fname"].asString();
        r2_DH_fname = root_pwd + config["r2_DH_fname"].asString();
        r2_DH_tool_fname = root_pwd + config["r2_DH_tool_fname"].asString();
        r2_DH_tool_assemble_fname = root_pwd + config["r2_DH_tool_assemble_fname"].asString();
        r2_DH_tool_disassemble_fname = root_pwd + config["r2_DH_tool_disassemble_fname"].asString();
        r2_DH_tool_alt_fname = root_pwd + config["r2_DH_tool_alt_fname"].asString();
        r2_DH_tool_alt_assemble_fname = root_pwd + config["r2_DH_tool_alt_assemble_fname"].asString();
        r2_robot_base_fname = root_pwd + config["Robot2_Base_fname"].asString();
        
        env_setup_fname = root_pwd + config["Env_Setup_fname"].asString();
        task_fname = root_pwd + config["Task_Graph_fname"].asString();
        lego_lib_fname = root_pwd + config["lego_lib_fname"].asString();
        robot1_name = config["Robot1_name"].asString();
        robot2_name = config["Robot2_name"].asString();

        r1_robot_state_topic = "/" + robot1_name + "/robot_state";
        r1_controller_joint_goal_topic = "/" + robot1_name + "/robot_goal";
        r1_controller_cart_goal_topic = "/" + robot1_name + "/robot_cart_goal";
        r1_controller_time_topic = "/" + robot1_name + "/jpc_travel_time";
        r2_robot_state_topic = "/" + robot2_name + "/robot_state";
        r2_controller_joint_goal_topic = "/" + robot2_name + "/robot_goal";
        r2_controller_cart_goal_topic = "/" + robot2_name + "/robot_cart_goal";
        r2_controller_time_topic = "/" + robot2_name + "/jpc_travel_time";

        twist_deg_ = config["Twist_Deg"].asInt();
        // jpc_travel_time_ = config["Waypoint_Travel_Time"].asDouble();

        std::ifstream task_file(task_fname, std::ifstream::binary);
        Json::Value task_json;
        task_file >> task_json;

        lego_ptr_ = std::make_shared<lego_manipulation::lego::Lego>();
        lego_ptr_->setup_dual_arm(env_setup_fname, lego_lib_fname, task_json, world_base_fname,
                                 r1_DH_fname, r1_DH_tool_fname, r1_DH_tool_disassemble_fname, r1_DH_tool_assemble_fname, r1_DH_tool_alt_fname, r1_DH_tool_alt_assemble_fname, r1_robot_base_fname,
                                 r2_DH_fname, r2_DH_tool_fname, r2_DH_tool_disassemble_fname, r2_DH_tool_assemble_fname, r2_DH_tool_alt_fname, r2_DH_tool_alt_assemble_fname, r2_robot_base_fname, set_state_client);

        r1_goal_pub_ = nh_.advertise<std_msgs::Float32MultiArray>(r1_controller_joint_goal_topic, lego_ptr_->robot_dof_1());
        r1_controller_time_pub_ = nh_.advertise<std_msgs::Float64>(r1_controller_time_topic, 1);
        r2_goal_pub_ = nh_.advertise<std_msgs::Float32MultiArray>(r2_controller_joint_goal_topic, lego_ptr_->robot_dof_2());
        r2_controller_time_pub_ = nh_.advertise<std_msgs::Float64>(r2_controller_time_topic, 1);
        r1_robot_state_sub_ = nh_.subscribe(r1_robot_state_topic, lego_ptr_->robot_dof_1() * 3, &ExpertDemo::robot1StateCallback, this);
        r2_robot_state_sub_ = nh_.subscribe(r2_robot_state_topic, lego_ptr_->robot_dof_2() * 3, &ExpertDemo::robot2StateCallback, this);
        
        start_recording_pub_ = nh_.advertise<std_msgs::Bool>("/start_recording", 1);
        stop_recording_pub_ = nh_.advertise<std_msgs::Bool>("/stop_recording", 1);

        std_msgs::Float64 r1_controller_time_msg;
        std_msgs::Float64 r2_controller_time_msg;
        // Modify stmotion controller runtime
        pub_jpc_travel_time(RobotID::Robot1, jpc_travel_time_);
        pub_jpc_travel_time(RobotID::Robot2, jpc_travel_time_); 
    }
    catch (const std::exception& e) {
        ROS_ERROR("ExpertDemo initialization error: %s", e.what());
    }
}

void ExpertDemo::reset_brick(const string& brick_name, const int& orientation,
                        const int& brick_loc_x, const int& brick_loc_y, const int& brick_loc_z)
{
    lego_ptr_->reset_brick(brick_name, orientation, brick_loc_x, brick_loc_y, brick_loc_z);
}

void ExpertDemo::set_marker(const int& orientation,
                        const int& brick_loc_x, const int& brick_loc_y, const int& brick_loc_z)
{
    lego_ptr_->set_marker(orientation, brick_loc_x, brick_loc_y, brick_loc_z);
}

Eigen::Matrix4d ExpertDemo::random_perturbation(double pos_std, double rot_std_deg)
{
    /* pos_std in meters, rot_std_deg in degrees */
    std::mt19937 gen(std::random_device{}());
    std::normal_distribution<> pos_dist(0.0, pos_std);      
    std::normal_distribution<> rot_dist(0.0, rot_std_deg);  

    double dx = pos_dist(gen);
    double dy = pos_dist(gen);
    double dz = pos_dist(gen);

    double dtheta_x = rot_dist(gen) * M_PI / 180.0;
    double dtheta_y = rot_dist(gen) * M_PI / 180.0;
    double dtheta_z = rot_dist(gen) * M_PI / 180.0;

    Eigen::AngleAxisd rot_x(dtheta_x, Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd rot_y(dtheta_y, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd rot_z(dtheta_z, Eigen::Vector3d::UnitZ());
    Eigen::Matrix3d rot = (rot_z * rot_y * rot_x).toRotationMatrix();

    Eigen::Matrix4d perturb = Eigen::Matrix4d::Identity();
    perturb.block<3,3>(0,0) = rot;
    perturb(0,3) = dx;
    perturb(1,3) = dy;
    perturb(2,3) = dz;

    return perturb;
}

void ExpertDemo::robot1StateCallback(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
    robot1_q_ << msg->data[0], msg->data[3], msg->data[6], msg->data[9], msg->data[12], msg->data[15];
    robot1_qd_ << msg->data[1], msg->data[4], msg->data[7], msg->data[10], msg->data[13], msg->data[16];
    robot1_qdd_ << msg->data[2], msg->data[5], msg->data[8], msg->data[11], msg->data[14], msg->data[17];
}

void ExpertDemo::robot2StateCallback(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
    robot2_q_ << msg->data[0], msg->data[3], msg->data[6], msg->data[9], msg->data[12], msg->data[15];
    robot2_qd_ << msg->data[1], msg->data[4], msg->data[7], msg->data[10], msg->data[13], msg->data[16];
    robot2_qdd_ << msg->data[2], msg->data[5], msg->data[8], msg->data[11], msg->data[14], msg->data[17];
}

void ExpertDemo::pubJointGoal(RobotID robot_id, const lego_manipulation::math::VectorJd& q)
{
    std_msgs::Float32MultiArray goal_msg;
    goal_msg.data.clear();
    if (robot_id == RobotID::Robot1) {
        for (int i = 0; i < lego_ptr_->robot_dof_1(); ++i) {
            goal_msg.data.push_back(q(i));
        }
        r1_goal_pub_.publish(goal_msg);
    } else {
        for (int i = 0; i < lego_ptr_->robot_dof_2(); ++i) {
            goal_msg.data.push_back(q(i));
        }
        r2_goal_pub_.publish(goal_msg);
    }
}

bool ExpertDemo::reachGoal(lego_manipulation::math::VectorJd& current_q, lego_manipulation::math::VectorJd& goal_q)
{
    double diff = (current_q - goal_q).cwiseAbs().maxCoeff();
    return diff < EPS_;
}

void ExpertDemo::homing(RobotID robot_id)
{
    std::cout << "Executing Homing for " << (robot_id == RobotID::Robot1 ? "Robot 1" : "Robot 2") << std::endl;

    ros::Rate rate(control_rate_);
    pub_jpc_travel_time(robot_id, jpc_travel_time_);
    if (robot_id == RobotID::Robot1) {
        while (ros::ok()) {
            if (reachGoal(robot1_q_, home_q_)) {
                std::cout << "Robot 1 reached home position." << std::endl;
                break;
            }
            pubJointGoal(robot_id, home_q_);
            rate.sleep();
        }
    } else {
        while (ros::ok()) {
            if (reachGoal(robot2_q_, home_q_)) {
                std::cout << "Robot 2 reached home position." << std::endl;
                break;
            }
            pubJointGoal(robot_id, home_q_);
            rate.sleep();
        }
    }
}


void ExpertDemo::moveJoint(RobotID robot_id, lego_manipulation::math::VectorJd& q)
{
    // std::cout << "Executing Move Joint for " << (robot_id == RobotID::Robot1 ? "Robot 1" : "Robot 2") << std::endl;

    ros::Rate rate(control_rate_);
    while (ros::ok()) {
        if (robot_id == RobotID::Robot1) {
            if (reachGoal(robot1_q_, q)) {
                std::cout << "Robot 1 reached target joint position." << std::endl;
                break;
            }
        } else {
            if (reachGoal(robot2_q_, q)) {
                std::cout << "Robot 2 reached target joint position." << std::endl;
                break;
            }
        }
        pubJointGoal(robot_id, q);
        rate.sleep();
    }
}

void ExpertDemo::attach_brick(RobotID robot_id, const std::string& brick_name)
{
    gazebo_ros_link_attacher::Attach attach_srv;
    attach_srv.request.model_name_1 = brick_name;
    attach_srv.request.link_name_1 = "brick";  
    if (robot_id == RobotID::Robot1) {
        attach_srv.request.model_name_2 = "gp4_arm_/r1/"; // TODO add from configuration file
    } else {
        attach_srv.request.model_name_2 = "gp4_arm_/r2/"; // TODO add from configuration file
    }
    attach_srv.request.link_name_2 = "link_tool";  // TODO add from configuration file

    if (attach_client_.call(attach_srv)) {
        if (attach_srv.response.ok) {
            std::cout << "Successfully attached " << brick_name << " to " << (robot_id == RobotID::Robot1 ? "Robot 1" : "Robot 2") << std::endl;
        } else {
            std::cerr << "Failed to attach " << brick_name << std::endl;
        }
    } else {
        std::cerr << "Service call to attach failed!" << std::endl;
    }
    ros::Duration(0.5).sleep(); // wait for a while to avoid collision

}


void ExpertDemo::detach_brick(RobotID robot_id, const std::string& brick_name)
{
    gazebo_ros_link_attacher::Attach detach_srv;
    detach_srv.request.model_name_1 = brick_name;
    detach_srv.request.link_name_1 = "brick"; 
    if (robot_id == RobotID::Robot1) {
        detach_srv.request.model_name_2 = "gp4_arm_/r1/"; 
    } else {
        detach_srv.request.model_name_2 = "gp4_arm_/r2/";  
    }
    detach_srv.request.link_name_2 = "link_tool";  

    if (detach_client_.call(detach_srv)) {
        if (detach_srv.response.ok) {
            std::cout << "Successfully detached " << brick_name << " from " << (robot_id == RobotID::Robot1 ? "Robot 1" : "Robot 2") << std::endl;
        } else {
            std::cerr << "Failed to detach " << brick_name << std::endl;
        }
    } else {
        std::cerr << "Service call to detach failed!" << std::endl;
    }
    ros::Duration(0.5).sleep(); // wait for a while to avoid collision
}

/* Scripted Pick Policy */
void ExpertDemo::pick(RobotID robot_id, const std::string& brick_name, int press_side, int press_offset)
{
    std::cout << "Executing Pick for " << (robot_id == RobotID::Robot1 ? "Robot 1" : "Robot 2") << std::endl;

    /* 0. Detect brick pose */

    Eigen::Matrix4d cart_T = Eigen::Matrix4d::Identity(4, 4);
    lego_ptr_->brick_pose_in_stock(brick_name, 1, 0, cart_T); // get brick pose, do not consider press side here
    
    /* 1. Move to prepare pose (with certainty)*/
    Eigen::Matrix4d prepare_T = cart_T;
    prepare_T(0, 3) += 0.05;  
    prepare_T(1, 3) -= 0.05;  // To make sure that wrist camera can see the brick
    prepare_T(2, 3) += 0.10;  // Move up 10cm

    Eigen::Matrix4d perturb = random_perturbation(0.03, 10.0); // add some random perturbation

    prepare_T = prepare_T * perturb;
    lego_manipulation::math::VectorJd prepare_q;
    bool IK_status = false;
    if (robot_id == RobotID::Robot1) {
        prepare_q = robot1_q_;
        prepare_q = lego_ptr_->IK(prepare_q, prepare_T, lego_ptr_->robot_DH_tool_r1(), lego_ptr_->robot_base_r1(), lego_ptr_->robot_base_inv_r1(),
                                                    lego_ptr_->robot_tool_inv_r1(), 0, IK_status);
    } else {
        prepare_q = robot2_q_;
        prepare_q = lego_ptr_->IK(prepare_q, prepare_T, lego_ptr_->robot_DH_tool_r2(), lego_ptr_->robot_base_r2(), lego_ptr_->robot_base_inv_r2(),
                                                    lego_ptr_->robot_tool_inv_r2(), 0, IK_status);
    }
    if (!IK_status) {
        std::cerr << "IK failed!" << std::endl;
        std::cout << "robot_goal: " << prepare_T<< std::endl;
        return;
    }

    /* 2. Press Down */
    lego_ptr_->brick_pose_in_stock(brick_name, press_side, press_offset, cart_T); // get brick pose

    lego_manipulation::math::VectorJd press_q;

    if (robot_id == RobotID::Robot1) {
        press_q = robot1_q_;
        press_q = lego_ptr_->IK(press_q, cart_T, lego_ptr_->robot_DH_tool_r1(), lego_ptr_->robot_base_r1(), lego_ptr_->robot_base_inv_r1(),
                                                    lego_ptr_->robot_tool_inv_r1(), 0, IK_status);
    } else {
        press_q = robot2_q_;
        press_q = lego_ptr_->IK(press_q, cart_T, lego_ptr_->robot_DH_tool_r2(), lego_ptr_->robot_base_r2(), lego_ptr_->robot_base_inv_r2(),
                                             lego_ptr_->robot_tool_inv_r2(), 0, IK_status);
    }
    if (!IK_status) {
        std::cerr << "IK failed!" << std::endl;
        std::cout << "robot_goal: " << cart_T << std::endl;
        return;
    }

    /* 3. Twist */

    lego_manipulation::math::VectorJd twist_q;

    double twist_rad = twist_deg_ / 180.0 * PI;
    Eigen::Matrix3d twist_R;
    Eigen::Matrix4d twist_T;
    twist_R << cos(twist_rad), 0, sin(twist_rad), 
                0, 1, 0, 
                -sin(twist_rad), 0, cos(twist_rad);
    twist_T = Eigen::MatrixXd::Identity(4, 4);
    twist_T.block(0, 0, 3, 3) << twist_R;


    if (robot_id == RobotID::Robot1) {
        cart_T = lego_manipulation::math::FK(press_q, lego_ptr_->robot_DH_tool_disassemble_r1(), lego_ptr_->robot_base_r1(), false);
        twist_T = cart_T * twist_T;
        twist_q = robot1_q_;
        twist_q = lego_ptr_->IK(twist_q, twist_T, lego_ptr_->robot_DH_tool_disassemble_r1(), lego_ptr_->robot_base_r1(), lego_ptr_->robot_base_inv_r1(),
                                                    lego_ptr_->robot_tool_disassemble_inv_r1(), 0, IK_status);
    } else {
        cart_T = lego_manipulation::math::FK(press_q, lego_ptr_->robot_DH_tool_disassemble_r2(), lego_ptr_->robot_base_r2(), false);
        twist_T = cart_T * twist_T;
        twist_q = robot2_q_;
        twist_q = lego_ptr_->IK(twist_q, twist_T, lego_ptr_->robot_DH_tool_disassemble_r2(), lego_ptr_->robot_base_r2(), lego_ptr_->robot_base_inv_r2(),
                                             lego_ptr_->robot_tool_disassemble_inv_r2(), 0, IK_status);
    }
    if (!IK_status) {
        std::cerr << "IK failed!" << std::endl;
        std::cout << "robot_goal: " << twist_T << std::endl;
        return;
    }

    /* 4. Pick Up */
    Eigen::Matrix4d pickup_T = twist_T;
    pickup_T(2, 3) += 0.15;  // Move up 15cm
    lego_manipulation::math::VectorJd pickup_q;

    if (robot_id == RobotID::Robot1) {
        pickup_q = robot1_q_;
        pickup_q = lego_ptr_->IK(pickup_q, pickup_T, lego_ptr_->robot_DH_tool_disassemble_r1(), lego_ptr_->robot_base_r1(), lego_ptr_->robot_base_inv_r1(),
                                                    lego_ptr_->robot_tool_disassemble_inv_r1(), 0, IK_status);
    } else {
        pickup_q = robot2_q_;
        pickup_q = lego_ptr_->IK(pickup_q, pickup_T, lego_ptr_->robot_DH_tool_disassemble_r2(), lego_ptr_->robot_base_r2(), lego_ptr_->robot_base_inv_r2(),
                                            lego_ptr_->robot_tool_disassemble_inv_r2(), 0, IK_status);
    }
    if (!IK_status) {
        std::cerr << "IK failed!" << std::endl;
        std::cout << "robot_goal: " << pickup_T << std::endl;
        return;
    }

    /* Execute Movement */
    moveJoint(robot_id, prepare_q);
    
    start_recording();
    ros::Duration(0.5).sleep(); // wait for a while to stabilize

    moveJoint(robot_id, press_q);
    attach_brick(robot_id, brick_name); // attach brick to robot

    moveJoint(robot_id, twist_q);
    moveJoint(robot_id, pickup_q);

    stop_recording();
}

/* Scripted Pick Policy */
void ExpertDemo::pick_prepare(RobotID robot_id, const std::string& brick_name, int press_side, int press_offset)
{
    std::cout << "Executing Pick for " << (robot_id == RobotID::Robot1 ? "Robot 1" : "Robot 2") << std::endl;

    /* 0. Detect brick pose */

    Eigen::Matrix4d cart_T = Eigen::Matrix4d::Identity(4, 4);
    lego_ptr_->brick_pose_in_stock(brick_name, 1, 0, cart_T); // get brick pose, do not consider press side here
    
    /* 1. Move to prepare pose (with certainty)*/
    Eigen::Matrix4d prepare_T = cart_T;
    prepare_T(0, 3) += 0.05;  
    prepare_T(1, 3) -= 0.05;  // To make sure that wrist camera can see the brick
    prepare_T(2, 3) += 0.10;  // Move up 10cm

    Eigen::Matrix4d perturb = random_perturbation(0.03, 10.0); // add some random perturbation

    prepare_T = prepare_T * perturb;
    lego_manipulation::math::VectorJd prepare_q;
    bool IK_status = false;
    if (robot_id == RobotID::Robot1) {
        prepare_q = robot1_q_;
        prepare_q = lego_ptr_->IK(prepare_q, prepare_T, lego_ptr_->robot_DH_tool_r1(), lego_ptr_->robot_base_r1(), lego_ptr_->robot_base_inv_r1(),
                                                    lego_ptr_->robot_tool_inv_r1(), 0, IK_status);
    } else {
        prepare_q = robot2_q_;
        prepare_q = lego_ptr_->IK(prepare_q, prepare_T, lego_ptr_->robot_DH_tool_r2(), lego_ptr_->robot_base_r2(), lego_ptr_->robot_base_inv_r2(),
                                                    lego_ptr_->robot_tool_inv_r2(), 0, IK_status);
    }
    if (!IK_status) {
        std::cerr << "IK failed!" << std::endl;
        std::cout << "robot_goal: " << prepare_T<< std::endl;
        return;
    }

    /* Execute Movement */
    moveJoint(robot_id, prepare_q);
}


void ExpertDemo::start_recording()
{
    std_msgs::Bool msg;
    msg.data = true;
    start_recording_pub_.publish(msg);
}

void ExpertDemo::stop_recording()
{
    std_msgs::Bool msg;
    msg.data = true;
    stop_recording_pub_.publish(msg);
}


void ExpertDemo::pub_jpc_travel_time(RobotID robot_id, double time)
{
    std_msgs::Float64 time_msg;
    time_msg.data = time;
    if (robot_id == RobotID::Robot1) {
        r1_controller_time_pub_.publish(time_msg);
    } else {
        r2_controller_time_pub_.publish(time_msg);
    }
}


