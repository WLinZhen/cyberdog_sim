
#include "interface/IOROS.h"
//#include "interface/KeyBoard.h"
#include <iostream>
#include <unistd.h>
#include <csignal>

void RosShutDown(int sig){
    (void) sig;
    rclcpp::shutdown();
}

IOROS::IOROS():IOInterface(){
    std::cout << "The control interface for ROS Gazebo simulation" << std::endl;
    // start subscriber
    this->node = std::make_shared<rclcpp::Node>("IOROS");
    initRecv();
    usleep(300000);     //wait for subscribers start
    // initialize publisher
    initSend();
    _lowState = LowlevelState();
    signal(SIGINT, RosShutDown);
    _userFlag = false;
}

IOROS::~IOROS(){
    //delete cmdPanel;
    rclcpp::shutdown();
}

void IOROS::sendRecv(const LowlevelCmd *cmd, LowlevelState *state){
    sendCmd(cmd);
    recvState(state);
}

void IOROS::sendCmd(const LowlevelCmd *lowCmd){
  rclcpp::spin_some(this->node);

  for(int i(0); i < 12; ++i){
    if(i % 3 == 0){
      cmd.q_des_abad[i / 3] = lowCmd->motorCmd[i].q;
      cmd.qd_des_abad[i / 3] = lowCmd->motorCmd[i].dq;
      cmd.kp_abad[i / 3] = lowCmd->motorCmd[i].Kp;
      cmd.kd_abad[i / 3] = lowCmd->motorCmd[i].Kd;
      cmd.tau_abad_ff[i / 3] = lowCmd->motorCmd[i].tau;
    }
    if(i % 3 == 1){
      cmd.q_des_hip[i / 3] = lowCmd->motorCmd[i].q;
      cmd.qd_des_hip[i / 3] = lowCmd->motorCmd[i].dq;
      cmd.kp_hip[i / 3] = lowCmd->motorCmd[i].Kp;
      cmd.kd_hip[i / 3] = lowCmd->motorCmd[i].Kd;
      cmd.tau_hip_ff[i / 3] = lowCmd->motorCmd[i].tau;
    }
    if(i % 3 == 2){
      cmd.q_des_knee[i / 3] = lowCmd->motorCmd[i].q;
      cmd.qd_des_knee[i / 3] = lowCmd->motorCmd[i].dq;
      cmd.kp_knee[i / 3] = lowCmd->motorCmd[i].Kp;
      cmd.kd_knee[i / 3] = lowCmd->motorCmd[i].Kd;
      cmd.tau_knee_ff[i / 3] = lowCmd->motorCmd[i].tau;
    }

  }

  this->cmd_pub->publish(cmd);
}

void IOROS::recvState(LowlevelState *state){
    for(int i(0); i < 12; ++i){
        state->motorState[i].q = _lowState.motorState[i].q;
        state->motorState[i].dq = _lowState.motorState[i].dq;
        // state->motorState[i].ddq = _lowState.motorState[i].ddq;
        state->motorState[i].tauEst = _lowState.motorState[i].tauEst;
    }

    for(int i(0); i < 3; ++i){
        state->imu.quaternion[i] = _lowState.imu.quaternion[i];
        state->imu.accelerometer[i] = _lowState.imu.accelerometer[i];
        state->imu.gyroscope[i] = _lowState.imu.gyroscope[i];
    }
    state->imu.quaternion[3] = _lowState.imu.quaternion[3];
}

void IOROS::initSend(){
    this->cmd_pub = this->node->create_publisher<ros_bridge::msg::LeggedCommand>("legged_command", 10);
}

void IOROS::initRecv(){
    this->imu_sub = this->node->create_subscription<ros_bridge::msg::SimImu>
            ("imu_data", 10, std::bind(&IOROS::imuCallback, this, std::placeholders::_1));
    this->state_sub = this->node->create_subscription<ros_bridge::msg::SimJoint>
            ("joint_data", 10, std::bind(&IOROS::jointStateCallback, this, std::placeholders::_1));
}

void IOROS::imuCallback(const ros_bridge::msg::SimImu & msg)
{
    _lowState.imu.quaternion[0] = msg.quat_w;
    _lowState.imu.quaternion[1] = msg.quat_x;
    _lowState.imu.quaternion[2] = msg.quat_y;
    _lowState.imu.quaternion[3] = msg.quat_z;

    _lowState.imu.gyroscope[0] = msg.gyro_x;
    _lowState.imu.gyroscope[1] = msg.gyro_y;
    _lowState.imu.gyroscope[2] = msg.gyro_z;

    _lowState.imu.accelerometer[0] = msg.accelerometer_x;
    _lowState.imu.accelerometer[1] = msg.accelerometer_y;
    _lowState.imu.accelerometer[2] = msg.accelerometer_z;
    // std::cout<<"-----------------------------"<<std::endl;
    // std::cout<<"_lowState.imu.quaternion[0] "<<_lowState.imu.quaternion[0]<<std::endl;
    // std::cout<<"_lowState.imu.quaternion[1] "<<_lowState.imu.quaternion[1]<<std::endl;
    // std::cout<<"_lowState.imu.quaternion[2] "<<_lowState.imu.quaternion[2]<<std::endl;
    // std::cout<<"_lowState.imu.quaternion[3] "<<_lowState.imu.quaternion[3]<<std::endl;
    // std::cout<<"-----------------------------"<<std::endl;
}

void IOROS::jointStateCallback(const ros_bridge::msg::SimJoint & msg)
{
    // fl
    _lowState.motorState[0].q = msg.q_abad_fr;
    _lowState.motorState[0].dq = msg.qd_abad_fr;
    _lowState.motorState[0].tauEst = msg.tau_abad_fr;
    //std::cout<<"msg.q_abad_fr : "<<msg.q_abad_fr<<std::endl;
    _lowState.motorState[1].q = msg.q_hip_fr;
    _lowState.motorState[1].dq = msg.qd_hip_fr;
    _lowState.motorState[1].tauEst = msg.tau_hip_fr;
    //std::cout<<"msg.q_hip_fr : "<<msg.q_hip_fr<<std::endl;
    _lowState.motorState[2].q = msg.q_knee_fr;
    _lowState.motorState[2].dq = msg.qd_knee_fr;
    _lowState.motorState[2].tauEst = msg.tau_knee_fr;
    //std::cout<<"msg.q_knee_fr : "<<msg.q_knee_fr<<std::endl;
    // fr
    _lowState.motorState[3].q = msg.q_abad_fl;
    _lowState.motorState[3].dq = msg.qd_abad_fl;
    _lowState.motorState[3].tauEst = msg.tau_abad_fl;

    _lowState.motorState[4].q = msg.q_hip_fl;
    _lowState.motorState[4].dq = msg.qd_hip_fl;
    _lowState.motorState[4].tauEst = msg.tau_hip_fl;

    _lowState.motorState[5].q = msg.q_knee_fl;
    _lowState.motorState[5].dq = msg.qd_knee_fl;
    _lowState.motorState[5].tauEst = msg.tau_knee_fl;
    // rl
    _lowState.motorState[6].q = msg.q_abad_rr;
    _lowState.motorState[6].dq = msg.qd_abad_rr;
    _lowState.motorState[6].tauEst = msg.tau_abad_rr;

    _lowState.motorState[7].q = msg.q_hip_rr;
    _lowState.motorState[7].dq = msg.qd_hip_rr;
    _lowState.motorState[7].tauEst = msg.tau_hip_rr;

    _lowState.motorState[8].q = msg.q_knee_rr;
    _lowState.motorState[8].dq = msg.qd_knee_rr;
    _lowState.motorState[8].tauEst = msg.tau_knee_rr;
    // rr
    _lowState.motorState[9].q = msg.q_abad_rl;
    _lowState.motorState[9].dq = msg.qd_abad_rl;
    _lowState.motorState[9].tauEst = msg.tau_abad_rl;

    _lowState.motorState[10].q = msg.q_hip_rl;
    _lowState.motorState[10].dq = msg.qd_hip_rl;
    _lowState.motorState[10].tauEst = msg.tau_hip_rl;

    _lowState.motorState[11].q = msg.q_knee_rl;
    _lowState.motorState[11].dq = msg.qd_knee_rl;
    _lowState.motorState[11].tauEst = msg.tau_knee_rl;
}