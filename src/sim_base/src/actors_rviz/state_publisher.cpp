#include "actors_rviz/state_publisher.hpp"
#include <vector>
#include <eigen3/Eigen/Dense>

StatePublisher::StatePublisher() : Node("state_publisher_node")
{
    // 创建关节命令订阅者
    this->base_sub = this->create_subscription<ros_bridge::msg::SimBase>
    ("base_data", 10, std::bind(&StatePublisher::BaseCom, this, std::placeholders::_1));
    //
    this->joint_sub = this->create_subscription<ros_bridge::msg::SimJoint>
    ("joint_data", 10, std::bind(&StatePublisher::CtrlCom, this, std::placeholders::_1));
    this->traj_pub = this->create_publisher<ros_bridge::msg::TrajVisual>
    ("traj_data", 10);

    joint_msg.name = {"FL_abad_joint","FR_abad_joint","RL_abad_joint","RR_abad_joint","FL_hip_joint","FR_hip_joint","RL_hip_joint","RR_hip_joint","FL_knee_joint","FR_knee_joint","RL_knee_joint","RR_knee_joint"};
    joint_state_publisher = this->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);
    // 初始化 msg position
    joint_msg.position = std::vector<double>(12, 0.0);
    // 初始化 msg velocity
    joint_msg.velocity = std::vector<double>(12, 0.0);
    // 初始化 msg effort
    joint_msg.effort = std::vector<double>(12, 0.0);
    //
    tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    transform_stamped.header.stamp = this->now();

    transform_stamped.header.frame_id = "world";  // 世界坐标系
    transform_stamped.child_frame_id = "base_link";  // 机器人坐标系
    transform_stamped.transform.translation.x = 0.0;  // 设置机器人在世界坐标系中的位置
    transform_stamped.transform.translation.y = 0.0;
    transform_stamped.transform.translation.z = 0.0;
    transform_stamped.transform.rotation.x = 0.0;
    transform_stamped.transform.rotation.y = 0.0;
    transform_stamped.transform.rotation.z = 0.0;
    transform_stamped.transform.rotation.w = 1.0;

    tf_broadcaster->sendTransform(transform_stamped);

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
}

void StatePublisher::BaseCom(const ros_bridge::msg::SimBase msg)
{
    sim_base = msg;
    transform_stamped.header.stamp = this->now(); 
    transform_stamped.transform.translation.x = sim_base.position_x;  // 设置机器人在世界坐标系中的位置
    transform_stamped.transform.translation.y = sim_base.position_y;
    transform_stamped.transform.translation.z = sim_base.position_z;
    transform_stamped.transform.rotation.x = sim_base.orientation_x;
    transform_stamped.transform.rotation.y = sim_base.orientation_y;
    transform_stamped.transform.rotation.z = sim_base.orientation_z;
    transform_stamped.transform.rotation.w = sim_base.orientation_w; 
    tf_broadcaster->sendTransform(transform_stamped);
    traj_visual.base_x = sim_base.position_x;
    traj_visual.base_y = sim_base.position_y;
    traj_visual.base_z = sim_base.position_z;
}

void StatePublisher::CtrlCom(const ros_bridge::msg::SimJoint msg)
{
    sim_joint = msg;
    joint_msg.position[0] = sim_joint.q_abad_fl;
    joint_msg.position[1] = sim_joint.q_abad_fr;
    joint_msg.position[2] = sim_joint.q_abad_rl;
    joint_msg.position[3] = sim_joint.q_abad_rr;
    joint_msg.position[4] = sim_joint.q_hip_fl;
    joint_msg.position[5] = sim_joint.q_hip_fr;
    joint_msg.position[6] = sim_joint.q_hip_rl;
    joint_msg.position[7] = sim_joint.q_hip_rr;
    joint_msg.position[8] = sim_joint.q_knee_fl;
    joint_msg.position[9] = sim_joint.q_knee_fr;
    joint_msg.position[10] = sim_joint.q_knee_rl;
    joint_msg.position[11] = sim_joint.q_knee_rr;
    joint_msg.header.stamp = this->now();
    joint_state_publisher->publish(joint_msg);
    count++;
    if(count>200)
    {
        count--;
    // // 尝试获取目标链接相对于参考链接的变换
    geometry_msgs::msg::TransformStamped fl_tf = tf_buffer_->lookupTransform("world", "FL_foot", tf2::TimePoint());
    geometry_msgs::msg::TransformStamped fr_tf = tf_buffer_->lookupTransform("world", "FR_foot", tf2::TimePoint());
    geometry_msgs::msg::TransformStamped rl_tf = tf_buffer_->lookupTransform("world", "RL_foot", tf2::TimePoint());
    geometry_msgs::msg::TransformStamped rr_tf = tf_buffer_->lookupTransform("world", "RR_foot", tf2::TimePoint());
    // 输出链接的位置坐标
    traj_visual.fl_foot_x = fl_tf.transform.translation.x;
    traj_visual.fl_foot_y = fl_tf.transform.translation.y;
    traj_visual.fl_foot_z = fl_tf.transform.translation.z;

    traj_visual.fr_foot_x = fr_tf.transform.translation.x;
    traj_visual.fr_foot_y = fr_tf.transform.translation.y;
    traj_visual.fr_foot_z = fr_tf.transform.translation.z;

    traj_visual.rl_foot_x = rl_tf.transform.translation.x;
    traj_visual.rl_foot_y = rl_tf.transform.translation.y;
    traj_visual.rl_foot_z = rl_tf.transform.translation.z;

    traj_visual.rr_foot_x = rr_tf.transform.translation.x;
    traj_visual.rr_foot_y = rr_tf.transform.translation.y;
    traj_visual.rr_foot_z = rr_tf.transform.translation.z;

    traj_pub->publish(traj_visual);
    }
    
}
    
