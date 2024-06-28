#include <iostream>
#include <ros/ros.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "async_algorithm_server");

    ros::AsyncSpinner spinner(0);
    spinner.start();

    ros::waitForShutdown();

    return 0;
}