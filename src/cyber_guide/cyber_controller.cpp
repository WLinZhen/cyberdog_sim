
#include <time.h>
#include <math.h>
#include "rclcpp/rclcpp.hpp"
#include <iostream>
#include <unistd.h>
#include <csignal>
#include <sched.h>


#include "robot/include/main_helper.h" // a
#include "cyber_controller.hpp"
#include "robot/include/RobotController.h"
#include "interface/IOROS.h"

bool running = true;
// over watch the ctrl+c command
void ShutDown(int sig){
    (void) sig;
    std::cout << "stop the controller" << std::endl;
    running = false;
}
void setProcessScheduler(){
    pid_t pid = getpid();
    sched_param param;
    param.sched_priority = sched_get_priority_max(SCHED_FIFO);
    if (sched_setscheduler(pid, SCHED_FIFO, &param) == -1) {
        std::cout << "[ERROR] Function setProcessScheduler failed." << std::endl;
        return;
    }
}

int main(int argc, char **argv)
{
    printf("Cheetah_Controller main() begin ... \n");
    // create the Robot instance.

    setProcessScheduler();
    rclcpp::init(argc, argv);
    signal(SIGINT, ShutDown);
    IOInterface *ioInter;
    ioInter = new IOROS();
    RobotController *robot_controller = new MIT_Controller(ioInter);
    while(running) {
        main_helper(robot_controller);
        // Enter here exit cleanup code.
    }
    return 0;
}
