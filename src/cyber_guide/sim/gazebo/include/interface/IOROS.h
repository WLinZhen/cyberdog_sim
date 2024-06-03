
#ifndef CYBER_GUIDE_IOROS_H
#define CYBER_GUIDE_IOROS_H

#include "rclcpp/rclcpp.hpp"
#include "interface/IOInterface.h"
#include "ros_bridge/msg/legged_command.hpp"
#include "ros_bridge/msg/sim_imu.hpp"
#include "ros_bridge/msg/sim_joint.hpp"
#include <string>

class IOROS : public IOInterface{
public:
    IOROS();
    ~IOROS();
    void sendRecv(const LowlevelCmd *cmd, LowlevelState *state);
    void recvUserParas(User_Parameters *paras);
private:
    void sendCmd(const LowlevelCmd *cmd);
    void recvState(LowlevelState *state);

    rclcpp::Node::SharedPtr node;
    rclcpp::Subscription<ros_bridge::msg::SimJoint>::SharedPtr state_sub;
    rclcpp::Subscription<ros_bridge::msg::SimImu>::SharedPtr imu_sub;
    rclcpp::Publisher<ros_bridge::msg::LeggedCommand>::SharedPtr cmd_pub;

    ros_bridge::msg::LeggedCommand cmd;
    ros_bridge::msg::SimImu imu;
    ros_bridge::msg::SimJoint state;

    LowlevelState _lowState;
    //repeated functions for multi-thread
    void initRecv();
    void initSend();

    //Callback functions for ROS
    void imuCallback(const ros_bridge::msg::SimImu & msg);
    void jointStateCallback(const ros_bridge::msg::SimJoint & msg);

};


#endif //CYBER_GUIDE_IOROS_H
