#include <robot_interface/robot_interface.h>
#include <vector>
#include <iostream>
#include <math.h>
#include <ros/ros.h>

int main(int argc, char ** argv)
{
    ros::init(argc, argv, "set_joint_positions");

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
    
    double rate = 1000.0;
    ros::Rate loop_rate(rate);
    double t = -1.0/rate;

    while (ros::ok())
    {
        t += (1.0/rate);
        cmd_positions[0] = 0.5*sin(t);
        cmd_positions[1] = 0.5*sin(t);
        cmd_positions[2] = 0.5*sin(t);
        cmd_positions[3] = 0.5*sin(t);
        cmd_positions[4] = 0.5*sin(t);
        cmd_positions[5] = 0.5*sin(t);
        cmd_positions[6] = 0.5*sin(t);
        robot->disable_cuff();
        robot->set_joint_positions(cmd_positions);
        // ros::spinOnce();
        loop_rate.sleep();
    }

    ros::shutdown();
    return 0;
}
