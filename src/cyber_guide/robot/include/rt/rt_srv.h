#ifndef RT_SRV_H
#define RT_SRV_H

#include <stdio.h>
#include <stdlib.h>
#include <sys/time.h>
#include <sys/types.h>
#include <unistd.h>
#include "rt_rc_interface.h"
#include <pthread.h>
#include <rclcpp/rclcpp.hpp>
#include "ros_bridge/srv/usr_cmd.hpp"
#include "rt_keyboard.h"

void get_srv_control_settings(void* settings);


class SrvCtrl : public CmdPanel{
public:
    SrvCtrl();
    ~SrvCtrl();
    void rosspin();
    rclcpp::Node::SharedPtr node;
    rclcpp::Service<ros_bridge::srv::UsrCmd>::SharedPtr cmd_srv;        // 用户命令服务
    void cmd_srv_request(const  std::shared_ptr<ros_bridge::srv::UsrCmd::Request> request,
                                std::shared_ptr<ros_bridge::srv::UsrCmd::Response> response);   // 用户命令服务请求回调函数
private:
    static void* runSrvCtrl(void *arg);
    void* run(void *arg);
    
    pthread_t _tid;
    pthread_t _tid_1;
};

#endif  