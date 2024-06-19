#include "MotorCtrl.hpp"

MotorCtrl::MotorCtrl( const double& loop_rate ):CustomInterface( loop_rate )
{
    reset();
    period = loop_rate;
    this->node = std::make_shared<rclcpp::Node>("MotorCtrl");

    //usleep(300000);
    this->cmd_sub = this->node->create_subscription<ros_bridge::msg::LeggedCommand>("legged_command", 10, std::bind(&MotorCtrl::lcCallback, this, std::placeholders::_1));
    this->imu_pub = this->node->create_publisher<ros_bridge::msg::SimImu>("imu_data", 10);
   // this->imu_data_pub = this->node->create_publisher<sensor_msgs::msg::Imu>("imu/data_raw", 10);
    this->esimu_pub = this->node->create_publisher<ros_bridge::msg::SimImu>("esimu_data", 10);
    this->state_pub = this->node->create_publisher<ros_bridge::msg::SimJoint>("joint_data", 10);

}

MotorCtrl::~MotorCtrl()
{
    rclcpp::shutdown();
}

void MotorCtrl::lcCallback(const ros_bridge::msg::LeggedCommand & msg)
{
    //std::cout<< "enter success cb"<<std::endl;
    for ( int i = 0; i < 4; i++ ) {    
        motor_cmd_tmp.q_des[ i * 3 ]        = msg.q_des_abad[i];
        motor_cmd_tmp.q_des[ i * 3 + 1 ]    = msg.q_des_hip[i];
        motor_cmd_tmp.q_des[ i * 3 + 2 ]    = msg.q_des_knee[i];
        motor_cmd_tmp.kp_des[ i * 3 ]       = msg.kp_abad[i];
        motor_cmd_tmp.kp_des[ i * 3 + 1]    = msg.kp_hip[i];
        motor_cmd_tmp.kp_des[ i * 3 + 2]    = msg.kp_knee[i];
        motor_cmd_tmp.kd_des[ i * 3 ]       = msg.kd_abad[i];
        motor_cmd_tmp.kd_des[ i * 3 + 1]    = msg.kd_hip[i];
        motor_cmd_tmp.kd_des[ i * 3 + 2]    = msg.kd_knee[i];
        motor_cmd_tmp.qd_des[ i * 3 ]       = msg.qd_des_abad[i];
        motor_cmd_tmp.qd_des[ i * 3 + 1]    = msg.qd_des_hip[i];
        motor_cmd_tmp.qd_des[ i * 3 + 2]    = msg.qd_des_knee[i];
        motor_cmd_tmp.tau_des[ i * 3 ]      = msg.tau_abad_ff[i];
        motor_cmd_tmp.tau_des[ i * 3 + 1]   = msg.tau_hip_ff[i];
        motor_cmd_tmp.tau_des[ i * 3 + 2]   = msg.tau_knee_ff[i];
    }
}


void MotorCtrl::reset()
{
    for ( int i = 0; i < 12; i++ ) {        
        motor_cmd_tmp.q_des[ i ]   = 0;
        motor_cmd_tmp.kp_des[ i ]  = 0;
        motor_cmd_tmp.kd_des[ i ]  = 0;
        motor_cmd_tmp.qd_des[ i ]  = 0;
        motor_cmd_tmp.tau_des[ i ] = 0;
    }
}

void MotorCtrl::UserCode(bool first_run)
{
    
    if ( first_run == true ) 
    {
        first_run = false;
        count     = 0;
    }
    sendCmd();
    recvState();
    if(count<=1000)
    {
        count++;
    }
    return;
    
}

void MotorCtrl::recvState()
{
    
    if(count<=1000)
    {
        return;
    }
    imustate();
    jointstate();
    prtstate();
}

void MotorCtrl::imustate()
{
    imu.quat_w = robot_data_.quat[0];
    imu.quat_x = robot_data_.quat[1];
    imu.quat_y = robot_data_.quat[2];
    imu.quat_z = robot_data_.quat[3];

    imu.accelerometer_x = robot_data_.acc[0];
    imu.accelerometer_y = robot_data_.acc[1];
    imu.accelerometer_z = robot_data_.acc[2];

    imu.gyro_x = robot_data_.omega[0];
    imu.gyro_y = robot_data_.omega[1];
    imu.gyro_z = robot_data_.omega[2];

    imu_pub->publish(imu);
}

void MotorCtrl::jointstate()
{
    state.q_abad_fr = robot_data_.q[0];
    state.qd_abad_fr = robot_data_.qd[0];
    state.tau_abad_fr = robot_data_.tau[0];

    state.q_hip_fr = robot_data_.q[1];
    state.qd_hip_fr = robot_data_.qd[1];
    state.tau_hip_fr = robot_data_.tau[1];
    
    state.q_knee_fr = robot_data_.q[2];
    state.qd_knee_fr = robot_data_.qd[2];
    state.tau_knee_fr = robot_data_.tau[2];

    state.q_abad_fl = robot_data_.q[3];
    state.qd_abad_fl = robot_data_.qd[3];
    state.tau_abad_fl = robot_data_.tau[3];

    state.q_hip_fl = robot_data_.q[4];
    state.qd_hip_fl = robot_data_.qd[4];
    state.tau_hip_fl = robot_data_.tau[4];
    
    state.q_knee_fl = robot_data_.q[5];
    state.qd_knee_fl = robot_data_.qd[5];
    state.tau_knee_fl = robot_data_.tau[5];

    state.q_abad_rr = robot_data_.q[6];
    state.qd_abad_rr = robot_data_.qd[6];
    state.tau_abad_rr = robot_data_.tau[6];

    state.q_hip_rr = robot_data_.q[7];
    state.qd_hip_rr = robot_data_.qd[7];
    state.tau_hip_rr = robot_data_.tau[7];
    
    state.q_knee_rr = robot_data_.q[8];
    state.qd_knee_rr = robot_data_.qd[8];
    state.tau_knee_rr = robot_data_.tau[8];

    state.q_abad_rl = robot_data_.q[9];
    state.qd_abad_rl = robot_data_.qd[9];
    state.tau_abad_rl = robot_data_.tau[9];

    state.q_hip_rl = robot_data_.q[10];
    state.qd_hip_rl = robot_data_.qd[10];
    state.tau_hip_rl = robot_data_.tau[10];
    
    state.q_knee_rl = robot_data_.q[11];
    state.qd_knee_rl = robot_data_.qd[11];
    state.tau_knee_rl = robot_data_.tau[11];
    state_pub->publish(state);
}

void MotorCtrl::prtstate()
{
    // std::cout<<"imu.quat_w:"<<imu.quat_w<<std::endl;
    // std::cout<<"imu.quat_x:"<<imu.quat_x<<std::endl;
    // std::cout<<"imu.quat_y:"<<imu.quat_y<<std::endl;
    // std::cout<<"imu.quat_z:"<<imu.quat_z<<std::endl;
    // std::cout<<"-----------------------"<<std::endl;
    // std::cout<<"imu.accelerometer_x:"<<imu.accelerometer_x<<std::endl;
    // std::cout<<"imu.accelerometer_y:"<<imu.accelerometer_y<<std::endl;
    // std::cout<<"imu.accelerometer_z:"<<imu.accelerometer_z<<std::endl;
    // std::cout<<"-----------------------"<<std::endl;
    // std::cout<<"imu.gyro_x:"<<imu.gyro_x<<std::endl;
    // std::cout<<"imu.gyro_y:"<<imu.gyro_y<<std::endl;
    // std::cout<<"imu.gyro_z:"<<imu.gyro_z<<std::endl;
    // std::cout<<"-----------------------"<<std::endl;
    // std::cout<<"state.q_abad_fr:"<<state.q_abad_fr<<std::endl;
    // std::cout<<"state.qd_abad_fr:"<<state.qd_abad_fr<<std::endl;
    // std::cout<<"state.tau_abad_fr:"<<state.tau_abad_fr<<std::endl;
    // std::cout<<"-----------------------"<<std::endl;
    // std::cout<<"state.q_hip_fr:"<<state.q_hip_fr<<std::endl;
    // std::cout<<"state.qd_hip_fr:"<<state.qd_hip_fr<<std::endl;
    // std::cout<<"state.tau_hip_fr:"<<state.tau_hip_fr<<std::endl;
    // std::cout<<"-----------------------"<<std::endl;
    // std::cout<<"state.q_knee_fr:"<<state.q_knee_fr<<std::endl;
    // std::cout<<"state.qd_knee_fr:"<<state.qd_knee_fr<<std::endl;
    // std::cout<<"state.tau_knee_fr:"<<state.tau_knee_fr<<std::endl;
    // std::cout<<"-----------------------"<<std::endl;
    // std::cout<<"motor_cmd_.q_des[1] :"<<motor_cmd_.q_des[1]<<std::endl;
    // std::cout<<"motor_cmd_.kp_des[1] :"<<motor_cmd_.kp_des[1]<<std::endl;
    // std::cout<<"motor_cmd_.kd_des[1] :"<<motor_cmd_.kd_des[1]<<std::endl;
    // std::cout<<"-----------------------"<<std::endl;
}

void MotorCtrl::sendCmd()
{
    if(count<=1000)
    {
        return;
    }
    rclcpp::spin_some(this->node);
    motor_cmd_ = motor_cmd_tmp;
}