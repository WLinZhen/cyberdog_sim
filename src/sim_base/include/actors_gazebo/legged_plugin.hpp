#include <functional>
#include <memory>

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/sensors/sensors.hh>
#include "actuator.hpp"
#include "rclcpp/rclcpp.hpp"
#include <gazebo/physics/PhysicsTypes.hh>
#include "ros_bridge/msg/legged_command.hpp"
#include "ros_bridge/msg/sim_imu.hpp"
#include "ros_bridge/msg/sim_base.hpp"
#include "ros_bridge/msg/sim_joint.hpp"

// #include "sim_base/msg/legged_command.hpp"

namespace gazebo
{

  class LeggedPlugin : public ModelPlugin
  {
  public:
    LeggedPlugin();
    ~LeggedPlugin();
    /**
     * @brief 当 Gazebo 启动时调用一次。
     * @param _parent 模型是链接、关节和插件的集合
     * @param _sdf 插件在世界文件中的 SDF 元素的指针
     */
    void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);

    /**
     * @brief 由世界更新开始事件调用
     */
    void OnUpdate();

  private:
    /**
     * @brief 从 Gazebo 更新关节状态
     */
    void GetJointStates();

    /**
     * @brief 将共享内存数据发送到控制程序
     */
    void SendSMData();
    
    /**
     * @brief 设置 Gazebo 中机器人的关节命令
     */
    void SetJointCom();
    /**
     * @brief 接收ros关节命令
     */
    void RosCom(const ros_bridge::msg::LeggedCommand msg);
    // 指向模型的指针
    physics::ModelPtr model_;

    // 指向更新事件连接的指针
    event::ConnectionPtr update_connection_;

    // ros节点
    private: rclcpp::Node::SharedPtr node;

    // Gazebo 传感器
    gazebo::sensors::Sensor_V sensors_;

    // 附加到当前机器人的 Gazebo 传感器
    gazebo::sensors::Sensor_V sensors_attached_to_robot_;

    // IMU 传感器
    gazebo::sensors::ImuSensorPtr imu_sensor_;

    // Gazebo 关节名称向量
    std::vector<std::string> joint_names_;
    //
    // Gazebo 关节映射
    std::map<std::string, gazebo::physics::JointPtr> joint_map_;  
    // 控制命令订阅者
    rclcpp::Subscription<ros_bridge::msg::LeggedCommand>::SharedPtr cmd_sub;  
    // cmd;
    ros_bridge::msg::LeggedCommand cmd;
    ros_bridge::msg::LeggedCommand cmd_pre;
    //imu 发布者
    rclcpp::Publisher<ros_bridge::msg::SimImu>::SharedPtr imu_pub;
    //ctrl 发布者
    rclcpp::Publisher<ros_bridge::msg::SimJoint>::SharedPtr joint_pub;
    //base 发布者
    rclcpp::Publisher<ros_bridge::msg::SimBase>::SharedPtr base_pub;
    // 各种状态和控制数据的向量
    std::vector<double> q_;
    std::vector<double> dq_;
    std::vector<double> tau_;
    std::vector<double> q_ctrl_;
    std::vector<double> dq_ctrl_;
    std::vector<double> tau_ctrl_;
    Actuator motor_;

    // 脚部计数器和频率计数器
    int foot_counter_;
    int frequency_counter_;
    int count;
    // 身体姿态的四元数
    Eigen::Quaterniond q_body_;

    // 腿部映射数组
    uint kleg_map[4] = {1, 0, 3, 2};

    bool use_currentloop_response_;
    bool use_TNcurve_motormodel_;
    bool use_torque_response_;
  };
}
