#include <csignal>
#include <iostream>

#include "main.hpp"

void signalHandler(int _signal)
{
    (void)_signal;
    std::cout << "You pressed Ctrl+C, exiting Surfel FAST-LIO2 Eigen!" << std::endl;
    SurfelLioApplication::requestExit();
}

int main(int _argc,
         char **_argv)
{
    ros::init(_argc, _argv, "laser_mapping", ros::init_options::NoSigintHandler);
    std::signal(SIGINT, signalHandler);
    static SurfelLioApplication application;
    return application.mainFunction();
}
