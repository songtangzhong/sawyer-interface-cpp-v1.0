#include <robot_interface/robot_interface.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Empty.h>
#include <cmath>

namespace robot_interface
{
Robot::Robot()
{
    dof = 7;

    joint_names.resize(dof);
    joint_names[0] = "right_j0";
    joint_names[1] = "right_j1";
    joint_names[2] = "right_j2";
    joint_names[3] = "right_j3";
    joint_names[4] = "right_j4";
    joint_names[5] = "right_j5";
    joint_names[6] = "right_j6";

    joint_state_sub_ = nh_.subscribe("/robot/joint_states", 10, 
        &Robot::joint_state_sub_cb_, this);
    joint_cmd_pub_ = nh_.advertise<intera_core_msgs::JointCommand>(
        "/robot/limb/right/joint_command", 10);
    speed_ratio_pub_ = nh_.advertise<std_msgs::Float64>(
            "/robot/limb/right/set_speed_ratio", 10);
    cuff_disable_pub_ = nh_.advertise<std_msgs::Empty>(
        "/robot/limb/right/suppress_cuff_interaction", 10);

    cur_positions_.resize(dof);
    cur_velocities_.resize(dof);
    cur_efforts_.resize(dof);

    joint_cmd_.names.resize(dof);
    joint_cmd_.position.resize(dof);
    joint_cmd_.velocity.resize(dof);
    joint_cmd_.effort.resize(dof);
}

Robot::~Robot(){}

void Robot::wait_for_ready(unsigned int seconds)
{
    unsigned int count = seconds;
    ros::Rate loop_rate(1.0);
    /*for (unsigned int j=0; j<=count; j++)
    {
        ROS_INFO("Wait for ready... %d.", count-j);
        loop_rate.sleep();
    }*/

    unsigned int j = 0;
    while (ros::ok())
    {
        ROS_INFO("Wait for ready... %d.", count-j);
        loop_rate.sleep();
        if (j++ == count)
        {
            break;
        }
    }
}

void Robot::joint_state_sub_cb_(const sensor_msgs::JointState::ConstPtr& msg)
{
    for (unsigned int j=0; j< msg->name.size(); j++)
    {
        for (unsigned int i=0; i<dof; i++)
        {
            if (msg->name[j] == joint_names[i])
            {
                cur_positions_[i] = msg->position[j];
                cur_velocities_[i] = msg->velocity[j];
                cur_efforts_[i] = msg->effort[j];
            }
        }
    }
}

void Robot::get_joint_positions(std::vector<double> &positions)
{
    for (unsigned int j=0; j<dof; j++)
    {
        positions[j] = cur_positions_[j];
    }
}

void Robot::get_joint_velocities(std::vector<double> &velocities)
{
    for (unsigned int j=0; j<dof; j++)
    {
        velocities[j] = cur_velocities_[j];
    }
}

void Robot::get_joint_efforts(std::vector<double> &efforts)
{
    for (unsigned int j=0; j<dof; j++)
    {
        efforts[j] = cur_efforts_[j];
    }
}

int Robot::set_joint_positions(const std::vector<double> positions)
{
    if (positions.size() != dof)
    {
        ROS_ERROR("Command position size is not right, canceling...");
        return -1;
    }

    for (unsigned int j=0; j<dof; j++)
    {
        joint_cmd_.names[j] = joint_names[j];
        joint_cmd_.position[j] = positions[j];
    }

    joint_cmd_.mode = joint_cmd_.POSITION_MODE;
    joint_cmd_.header.stamp = ros::Time::now();
    joint_cmd_pub_.publish(joint_cmd_);

    return 1;
}

int Robot::set_joint_efforts(const std::vector<double> efforts)
{
    if (efforts.size() != dof)
    {
        ROS_ERROR("Command effort size is not right, canceling...");
        return -1;
    }

    for (unsigned int j=0; j<dof; j++)
    {
        joint_cmd_.names[j] = joint_names[j];
        joint_cmd_.effort[j] = efforts[j];
    }

    joint_cmd_.mode = joint_cmd_.TORQUE_MODE;
    joint_cmd_.header.stamp = ros::Time::now();
    joint_cmd_pub_.publish(joint_cmd_);

    return 1;
}

void Robot::set_joint_position_speed(const double speed)
{
    std_msgs::Float64 ratio;
    ratio.data = speed;
    speed_ratio_pub_.publish(ratio);
}

void Robot::disable_cuff()
{
    std_msgs::Empty empty_msg;
    cuff_disable_pub_.publish(empty_msg);
}

int Robot::move_to_target_joint_positions(const std::vector<double> positions, 
    const double speed, const double threshold)
{
    Robot::set_joint_position_speed(speed);
    
    std::vector<double> cur_positions;
    cur_positions.resize(dof);

    ros::Rate loop_rate(1000.0);
    while (ros::ok())
    {
        ros::spinOnce();
        Robot::get_joint_positions(cur_positions);

        unsigned int count = 0;
        for (unsigned int j=0; j<dof; j++)
        {
            if (fabs(cur_positions[j]-positions[j])<threshold)
            {
                count ++;
            }
        }

        if (count==dof)
        {
            return 1;
        }
        else
        {
            Robot::disable_cuff();
            if (Robot::set_joint_positions(positions)<0)
            {
                return -1;
            }
            loop_rate.sleep();
        }
    }
}

}
