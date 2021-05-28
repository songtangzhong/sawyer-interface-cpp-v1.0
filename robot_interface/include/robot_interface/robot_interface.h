#ifndef ROBOT_INTERFACE_H_
#define ROBOT_INTERFACE_H_

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <intera_core_msgs/JointCommand.h>
#include <vector>

namespace robot_interface
{
class Robot
{
public:
    Robot();
    ~Robot();

    void wait_for_ready(unsigned int seconds);

    void get_joint_positions(std::vector<double> &positions);
    void get_joint_velocities(std::vector<double> &velocities);
    void get_joint_efforts(std::vector<double> &efforts);

    int set_joint_positions(const std::vector<double> positions);
    int set_joint_efforts(const std::vector<double> efforts);

    void set_joint_position_speed(const double speed);

    void disable_cuff();
    
    int move_to_target_joint_positions(const std::vector<double> positions, 
        const double speed, const double threshold);

    unsigned int dof;
    std::vector<std::string> joint_names;

private:
    void joint_state_sub_cb_(const sensor_msgs::JointState::ConstPtr& msg);

    ros::NodeHandle nh_; 
    
	ros::Subscriber joint_state_sub_;
    ros::Publisher joint_cmd_pub_;
    ros::Publisher speed_ratio_pub_;
    ros::Publisher cuff_disable_pub_;
    
    std::vector<double> cur_positions_;
    std::vector<double> cur_velocities_;
    std::vector<double> cur_efforts_;

    intera_core_msgs::JointCommand joint_cmd_;

};

}

#endif