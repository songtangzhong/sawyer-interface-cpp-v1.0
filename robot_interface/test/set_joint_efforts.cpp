#include <robot_interface/robot_interface.h>
#include <vector>
#include <iostream>
#include <math.h>
#include <ros/ros.h>

int main(int argc, char ** argv)
{
    ros::init(argc, argv, "set_joint_efforts");

    std::shared_ptr<robot_interface::Robot> robot =
        std::make_shared<robot_interface::Robot>();
    robot->wait_for_ready(3);

    std::vector<double> cmd_positions;
    cmd_positions.resize(robot->dof);
    cmd_positions[0] = 0.0;
    cmd_positions[1] = 0.0;
    cmd_positions[2] = 0.0;
    cmd_positions[3] = 0.0;
    cmd_positions[4] = 0.0;
    cmd_positions[5] = 0.0;
    cmd_positions[6] = 0.0;
    robot->move_to_target_joint_positions(cmd_positions,0.1,0.001);
    ros::Duration(3.0).sleep();
    
    std::vector<double> cmd_efforts;
    cmd_efforts.resize(robot->dof);
    double rate = 1000.0;
    ros::Rate loop_rate(rate);
    double t = -1.0/rate;

    while (ros::ok())
    {
        t += (1.0/rate);
        cmd_efforts[0] = 0.0;
        cmd_efforts[1] = 0.0;
        cmd_efforts[2] = 0.0;
        cmd_efforts[3] = 0.0;
        cmd_efforts[4] = 0.0;
        cmd_efforts[5] = 0.0;
        cmd_efforts[6] = 0.0;
        robot->disable_cuff();
        robot->set_joint_efforts(cmd_efforts);
        loop_rate.sleep();
    }

    ros::shutdown();
    return 0;
}
