#ifndef motor_ctrl_hpp
#define motor_ctrl_hpp
//----- 电机SDK接口 -----//
#include <custom_interface.hpp>
//----- ROS接口 -----//
#include "rclcpp/rclcpp.hpp"
#include "ros_bridge/msg/legged_command.hpp"
#include "ros_bridge/msg/sim_imu.hpp"
#include "ros_bridge/msg/sim_joint.hpp"
#include "ros_bridge/msg/qr.hpp"
//----- C++标准库 -----//
#include <iostream>
#include <unistd.h>
#include <csignal>
//----- 线程优化 -----//
#include <thread>
#include <mutex>

class MotorCtrl : public CustomInterface {
public:
    MotorCtrl( const double& loop_rate );
    ~MotorCtrl();
    void reset();
    void UserCode(bool first_run);
    //  接收机器人状态
    void recvState();
    void imustate();
    void jointstate();
    //  发送关节指令
    void sendCmd();
    //  打印数据
    void prtstate();
public:
    // 
    bool            first_run = true;
    long long       count     = 0;
    MotorCmd        motor_cmd_tmp;
    // ros
    rclcpp::Node::SharedPtr node;
    rclcpp::Publisher<ros_bridge::msg::SimJoint>::SharedPtr state_pub;
    ros_bridge::msg::SimJoint state;

    rclcpp::Publisher<ros_bridge::msg::SimImu>::SharedPtr imu_pub;
    ros_bridge::msg::SimImu imu;

    rclcpp::Publisher<ros_bridge::msg::SimImu>::SharedPtr esimu_pub;
    ros_bridge::msg::SimImu esimu;

    rclcpp::Subscription<ros_bridge::msg::LeggedCommand>::SharedPtr cmd_sub;
    ros_bridge::msg::LeggedCommand cmd;
    void lcCallback(const ros_bridge::msg::LeggedCommand & msg);
};
#endif