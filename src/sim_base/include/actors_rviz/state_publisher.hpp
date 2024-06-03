#include <rclcpp/rclcpp.hpp>

#include <cpp_types.hpp>

#include "sensor_msgs/msg/joint_state.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "ros_bridge/msg/sim_base.hpp"
#include "ros_bridge/msg/sim_joint.hpp"
#include "ros_bridge/msg/traj_visual.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
class StatePublisher : public rclcpp::Node
{
public:

    StatePublisher();
    void BaseCom(const ros_bridge::msg::SimBase msg);
    void CtrlCom(const ros_bridge::msg::SimJoint msg);

private:
    int count = 0;
    // 
    ros_bridge::msg::SimBase sim_base;
    ros_bridge::msg::SimJoint sim_joint;
    ros_bridge::msg::TrajVisual traj_visual;
    //ctrl 订阅者
    rclcpp::Subscription<ros_bridge::msg::SimJoint>::SharedPtr joint_sub;
    //base 订阅者
    rclcpp::Subscription<ros_bridge::msg::SimBase>::SharedPtr base_sub;
    //
    rclcpp::Publisher<ros_bridge::msg::TrajVisual>::SharedPtr traj_pub;
    sensor_msgs::msg::JointState joint_msg;
    geometry_msgs::msg::TransformStamped transform_stamped;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_publisher;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};