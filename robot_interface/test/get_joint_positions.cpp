#include <robot_interface/robot_interface.h>
#include <vector>
#include <iostream>
#include <ros/ros.h>

int main(int argc, char ** argv)
{
    ros::init(argc, argv, "get_joint_positions");

    std::shared_ptr<robot_interface::Robot> robot =
        std::make_shared<robot_interface::Robot>();
    robot->wait_for_ready(5);

    std::vector<double> cur_positions;
    cur_positions.resize(robot->dof);

    ros::Rate loop_rate(1000.0);

    while (ros::ok())
    {
        ros::spinOnce();
        robot->get_joint_positions(cur_positions);
        for (unsigned int j=0; j< robot->dof; j++)
        {
            std::cout << "cur_positions[" << j << "]: " << cur_positions[j] << std::endl;
        }

        loop_rate.sleep();
    }

    ros::shutdown();
    return 0;
}
