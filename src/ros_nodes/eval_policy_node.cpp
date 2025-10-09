#include "expert_demo.hpp"
#include <random>

void random_brick_placement(int& orientation, int& x, int& y, int& z, int& press_side, int& press_offset)
{
    std::mt19937 gen(std::random_device{}());
    std::uniform_int_distribution<> orientation_dist(0, 1); // 0-1 for 0,90 degrees
    std::uniform_int_distribution<> x_dist(15, 20); // x position range
    std::uniform_int_distribution<> y_dist(20, 30); // y position range
    std::uniform_int_distribution<> z_dist(1, 1); // z position range
    std::uniform_int_distribution<> press_side_dist(1, 4); // press side 1-4
    std::uniform_int_distribution<> press_offset_dist(0, 2); // press offset 0-2 for 2x4 brick


    orientation = orientation_dist(gen);
    x = x_dist(gen);
    y = y_dist(gen);
    z = z_dist(gen);
    press_side = press_side_dist(gen);
    press_offset = press_offset_dist(gen);

    if (press_side == 2 || press_side == 3) { // width side
        press_offset = 0; 
    }
}


void get_marker_position(int orientation, int brick_loc_x, int brick_loc_y, int brick_loc_z,
                        int press_side, int press_offset,
                        int& marker_orientation, int& marker_x, int& marker_y, int& marker_z)
{
    /* Given the brick position, orientation and press_side/offset, return the marker position */
    marker_z = brick_loc_z;
    marker_orientation = orientation;
    
    switch (press_side) {
        case 1: // length size, bottom  
            if (orientation == 0) {  // checked
                marker_x = brick_loc_x;
                marker_y = brick_loc_y + press_offset;
                marker_orientation = 0;
            }
            else { // orientation == 1  checked
                marker_x = brick_loc_x + 2 - press_offset;
                marker_y = brick_loc_y;
                marker_orientation = 1;
            }
            break;
            
        case 2: // width side, left, offset = 0
            if (orientation == 0) { // checked
                marker_x = brick_loc_x; 
                marker_y = brick_loc_y + 3;
                marker_orientation = 1; 
            }
            else{ // orientation == 1 // checked
                marker_x = brick_loc_x;
                marker_y = brick_loc_y;
                marker_orientation = 0;
            }
            break;
            
        case 3: // width side, right, offset = 0
            if (orientation == 0) {  // checked
                marker_x = brick_loc_x; 
                marker_y = brick_loc_y;
                marker_orientation = 1;
            }
            else{ // orientation == 1 checked
                marker_x = brick_loc_x + 3;
                marker_y = brick_loc_y;
                marker_orientation = 0;
            }
            break;
            
        case 4: // length side, top
            if (orientation == 0) { // checked
                marker_x = brick_loc_x + 1;
                marker_y = brick_loc_y + press_offset; 
                marker_orientation = 0; 
            }
            else{ // orientation == 1  checked
                marker_x = brick_loc_x + 2 - press_offset; 
                marker_y = brick_loc_y + 1;
                marker_orientation = 1;
            }
            break;
            /* orientation: 0 
                 2  1  0
            --------------
            | o  o  o  o |
            | o  o  o  o | ^ x
            -------------- |
                 2  1  0 
                    <- y
            */
            /* orientation: 1
                --------
              0 | o  o | 0
              1 | o  o | 1
              2 | o  o | 2
                | o  o |   ^ x
                --------   |
                    <- y
            */
    }
}


void single_b2_1_pick(int argc, char **argv) {
    ros::init(argc, argv, "policy_eval_node");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    ExpertDemo expert_demo(nh, private_nh);
    ros::Duration(1.0).sleep();
    expert_demo.init();
    ros::AsyncSpinner async_spinner(1);
    async_spinner.start();

    int max_episodes;
    private_nh.param("max_episodes", max_episodes, 10);

    std::string brick_name = "b2_1";
    int orientation, brick_loc_x, brick_loc_y, brick_loc_z, press_side, press_offset;
    int marker_orientation, marker_x, marker_y, marker_z;
    
    expert_demo.show_marker = true;

    for (int episode = 0; episode < max_episodes; ++episode) {
        ROS_INFO("Starting episode %d", episode + 1);
        expert_demo.homing(RobotID::Robot1);
        random_brick_placement(orientation, brick_loc_x, brick_loc_y, brick_loc_z, press_side, press_offset);
        std::cout << "Brick placement - Orientation: " << orientation << ", Position: (" 
                  << brick_loc_x << ", " << brick_loc_y << ", " << brick_loc_z << "), Press Side: " 
                  << press_side << ", Press Offset: " << press_offset << std::endl;
        expert_demo.reset_brick(brick_name, orientation, brick_loc_x, brick_loc_y, brick_loc_z);
        if (expert_demo.show_marker) {
            get_marker_position(orientation, brick_loc_x, brick_loc_y, brick_loc_z,
                                press_side, press_offset,
                                marker_orientation, marker_x, marker_y, marker_z);
            expert_demo.set_marker(marker_orientation, marker_x, marker_y, marker_z);
        }
        expert_demo.pick_prepare(RobotID::Robot1, brick_name, press_side, press_offset);
        expert_demo.start_recording();
        ros::Duration(45.0).sleep(); 
        

        
        expert_demo.stop_recording();
        ros::Duration(0.5).sleep(); // wait for a while to stabilize
        expert_demo.stop_recording();
        ros::Duration(0.5).sleep(); // wait for a while to stabilize
        ROS_INFO("Episode %d completed, update brick", episode + 1);

    }
}



int main(int argc, char **argv)
{
    single_b2_1_pick(argc, argv);
    return 0;
}









