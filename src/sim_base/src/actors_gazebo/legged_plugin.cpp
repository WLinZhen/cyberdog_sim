#include "actors_gazebo/legged_plugin.hpp"


namespace gazebo
{

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(LeggedPlugin)
    LeggedPlugin::LeggedPlugin()
    {
        this->node = std::make_shared<rclcpp::Node>("legged_plugin_node");
        count = 0;
    }
     // 在插件的析构函数中添加通知子线程退出的逻辑
    LeggedPlugin::~LeggedPlugin()
    {
        rclcpp::shutdown();
        // 通知子线程退出的逻辑
    }
    void LeggedPlugin::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
    {
        std::cout << "**************进入插件**************" << std::endl;
        // 存储对模型的指针
        model_ = _parent;
        
        std::cout << model_->GetName() << " 被导入" << std::endl;

        // 监听更新事件。此事件在每次模拟迭代时广播。
        update_connection_ = event::Events::ConnectWorldUpdateBegin(
            std::bind(&LeggedPlugin::OnUpdate, this));

        // 获取传感器列表
        sensors_ = gazebo::sensors::SensorManager::Instance()->GetSensors();

        // 如果模拟了多个机器人，则需要仅获取连接到我们机器人的传感器
        for (unsigned int i = 0; i < sensors_.size(); ++i)
        {
            if (sensors_[i]->ScopedName().find("::" + model_->GetName() + "::") != std::string::npos)
            {
                sensors_attached_to_robot_.push_back(sensors_[i]);
                std::cout << sensors_attached_to_robot_[i]->ScopedName() << std::endl;
            }
        }

        for (unsigned int i = 0; i < sensors_attached_to_robot_.size(); ++i)
        {
            if (sensors_attached_to_robot_[i]->Type().compare("imu") == 0)
            {
                imu_sensor_ = std::static_pointer_cast<gazebo::sensors::ImuSensor>(sensors_attached_to_robot_[i]);
                std::cout << "找到IMU传感器：" << imu_sensor_->Name() << std::endl;
            }
        }

        // 遍历Gazebo模型关节向量并将关节指针存储在映射中
        const gazebo::physics::Joint_V &joints = model_->GetJoints();
        for (unsigned int i = 0; i < joints.size(); i++)
        {
            std::string joint_name = joints[i]->GetName();
            joint_names_.push_back(joint_name);
            joint_map_[joint_name] = model_->GetJoint(joint_name);
            std::cout << "关节 # " << i << " - " << joint_name << std::endl;
        }

        q_.resize(joints.size());
        dq_.resize(joints.size());
        tau_.resize(joints.size());
        q_ctrl_.resize(joints.size());
        dq_ctrl_.resize(joints.size());
        tau_ctrl_.resize(joints.size());
        for (unsigned int i = 0; i < joint_names_.size(); i++)
        {
            unsigned int index = 0;
            std::string n = joint_names_[i];
            q_[i] = joint_map_[n]->Position(index);
            dq_[i] = joint_map_[n]->GetVelocity(index);
            tau_[i] = joint_map_[n]->GetForce(index);
        }

        for (uint i = 0; i < 4; i++)
        {
            q_ctrl_[3 * i] = q_[3 * i];
            q_ctrl_[3 * i + 1] = -q_[3 * i + 1];
            q_ctrl_[3 * i + 2] = -q_[3 * i + 2];
            dq_ctrl_[3 * i] = dq_[3 * i];
            dq_ctrl_[3 * i + 1] = -dq_[3 * i + 1];
            dq_ctrl_[3 * i + 2] = -dq_[3 * i + 2];
            tau_ctrl_[3 * i] = tau_[3 * i];
            tau_ctrl_[3 * i + 1] = -tau_[3 * i + 1];
            tau_ctrl_[3 * i + 2] = -tau_[3 * i + 2];
        }

        // 将Gazebo更新频率与控制程序频率匹配
        frequency_counter_ = 0;

        // Enable currentloop response limit of the motors
        use_currentloop_response_ = true;
        // Enable TN curve limit of the motors
        use_TNcurve_motormodel_ = true;

        // 计数用于将接触力传递到身体坐标
        foot_counter_ = 0;
        if (rclcpp::ok()) 
        {   
            // 创建imu发布者
            this->imu_pub = this->node->create_publisher<ros_bridge::msg::SimImu>("imu_data", 10);
            // 创建ctrl发布者
            this->joint_pub = this->node->create_publisher<ros_bridge::msg::SimJoint>("joint_data", 10);
            // 创建base发布者
            this->base_pub = this->node->create_publisher<ros_bridge::msg::SimBase>("base_data", 10);
            
            std::thread([this]() {
                
                // 创建关节命令订阅者
                this->cmd_sub = this->node->create_subscription<ros_bridge::msg::LeggedCommand>
                    ("legged_command", 10, std::bind(&LeggedPlugin::RosCom, this, std::placeholders::_1));
                if (this->cmd_sub) {
                    RCLCPP_INFO(this->node->get_logger(), "Subscriber created successfully.");
                } else {
                    RCLCPP_ERROR(this->node->get_logger(), "Failed to create subscriber.");
                    // 在这里可能采取适当的错误处理措施
                }
                // 检查节点是否已经添加到执行器
               
            }).detach();
            
        }
        RCLCPP_INFO(this->node->get_logger(), "Plugin created successfully.");
    } // LeggedPlugin::Load

    // 在世界更新开始事件时调用
    void LeggedPlugin::OnUpdate()
    {
        rclcpp::spin_some(this->node);
        // 匹配 gazebo 更新频率与控制程序频率
        frequency_counter_++;

        // 获取关节状态
        GetJointStates();
        
        // 如果还未达到控制程序频率的两倍，设置关节的质心并返回
        // if (frequency_counter_ < 2)
        // {
        //     SetJointCom();
        //     return;
        // }

        // ros2发送机器人状态数据到控制程序
        SendSMData();

        cmd = cmd_pre;
        //从控制程序接收并设置机器人的关节命令
        
        SetJointCom();


        // 重置计数器
        frequency_counter_ = 0;
    }

    void LeggedPlugin::RosCom(const ros_bridge::msg::LeggedCommand msg)
    {
        cmd_pre = msg;

    }

    // 获取关节状态
    void LeggedPlugin::GetJointStates()
    {
        for (unsigned int i = 0; i < joint_names_.size(); i++)
        {
            unsigned int index = 0;
            std::string n = joint_names_[i];
            // 获取关节位置、速度和力矩
            q_[i] = joint_map_[n]->Position(index);
            dq_[i] = joint_map_[n]->GetVelocity(index);
            tau_[i] = joint_map_[n]->GetForce(index);
        }

        // 将关节状态转换为控制程序需要的坐标系
        for (uint i = 0; i < 4; i++)
        {
            q_ctrl_[3 * i] = q_[3 * i];
            q_ctrl_[3 * i + 1] = q_[3 * i + 1];
            q_ctrl_[3 * i + 2] = q_[3 * i + 2];
            dq_ctrl_[3 * i] = dq_[3 * i];
            dq_ctrl_[3 * i + 1] = dq_[3 * i + 1];
            dq_ctrl_[3 * i + 2] = dq_[3 * i + 2];
            tau_ctrl_[3 * i] = tau_[3 * i];
            tau_ctrl_[3 * i + 1] = tau_[3 * i + 1];
            tau_ctrl_[3 * i + 2] = tau_[3 * i + 2];
        }
    }
    
    // 设置关节质心
    void LeggedPlugin::SetJointCom()
    {
        // 根据关节命令计算电机力矩
        for (int i = 0; i < 4; i++)
        {
            unsigned int index = 0;
            // PD控制器
            double abad_effort = cmd.kp_abad[i] * (cmd.q_des_abad[i] - q_ctrl_[kleg_map[i] * 3]) + cmd.kd_abad[i] * (cmd.qd_des_abad[i] - dq_ctrl_[kleg_map[i] * 3]) + cmd.tau_abad_ff[i];
            double hip_effort = (cmd.kp_hip[i] * (cmd.q_des_hip[i] - q_ctrl_[kleg_map[i] * 3 + 1]) + cmd.kd_hip[i] * (cmd.qd_des_hip[i] - dq_ctrl_[kleg_map[i] * 3 + 1]) + cmd.tau_hip_ff[i]);
            double knee_effort = (cmd.kp_knee[i] * (cmd.q_des_knee[i] - q_ctrl_[kleg_map[i] * 3 + 2]) + cmd.kd_knee[i] * (cmd.qd_des_knee[i] - dq_ctrl_[kleg_map[i] * 3 + 2]) + cmd.tau_knee_ff[i]);

            // TN 曲线电机模型
            if (use_TNcurve_motormodel_)
            {
                abad_effort = motor_.GetTorque(abad_effort, dq_[kleg_map[i] * 3]);
                hip_effort = motor_.GetTorque(hip_effort, dq_[kleg_map[i] * 3 + 1]);
                knee_effort = motor_.GetTorque(knee_effort, dq_[kleg_map[i] * 3 + 2]);
            }

            // 电流环响应
            if (use_currentloop_response_)
            {
                abad_effort = motor_.CerrentLoopResponse(abad_effort, dq_[kleg_map[i] * 3], kleg_map[i] * 3);
                hip_effort = motor_.CerrentLoopResponse(hip_effort, dq_[kleg_map[i] * 3 + 1], kleg_map[i] * 3 + 1);
                knee_effort = motor_.CerrentLoopResponse(knee_effort, dq_[kleg_map[i] * 3 + 2], kleg_map[i] * 3 + 2);
            }
            // RCLCPP_INFO(this->node->get_logger(), "%f, %f, %f",
            //         abad_effort,hip_effort,knee_effort);
            //设置关节力矩
            joint_map_[joint_names_[kleg_map[i] * 3]]->SetForce(index, abad_effort);
            joint_map_[joint_names_[kleg_map[i] * 3 + 1]]->SetForce(index, hip_effort);
            joint_map_[joint_names_[kleg_map[i] * 3 + 2]]->SetForce(index, knee_effort);
        }
    }

    void LeggedPlugin::SendSMData()
    {
        count++;

        ros_bridge::msg::SimImu sim_imu;

        // 读取IMU数据并归一化四元数
        Eigen::Quaterniond imu_quaternion(imu_sensor_->Orientation().W(), imu_sensor_->Orientation().X(), imu_sensor_->Orientation().Y(), imu_sensor_->Orientation().Z());
        imu_quaternion.normalize();

        // 更新消息中的四元数
        sim_imu.quat_w = imu_quaternion.w();
        sim_imu.quat_x = imu_quaternion.x();
        sim_imu.quat_y = imu_quaternion.y();
        sim_imu.quat_z = imu_quaternion.z();

        // 读取角速度和线性加速度
        sim_imu.gyro_x = imu_sensor_->AngularVelocity()[0];
        sim_imu.gyro_y = imu_sensor_->AngularVelocity()[1];
        sim_imu.gyro_z = imu_sensor_->AngularVelocity()[2];

        sim_imu.accelerometer_x = imu_sensor_->LinearAcceleration()[0];
        sim_imu.accelerometer_y = imu_sensor_->LinearAcceleration()[1];
        sim_imu.accelerometer_z = imu_sensor_->LinearAcceleration()[2];

        // 发布消息
        this->imu_pub->publish(sim_imu);

        ros_bridge::msg::SimJoint sim_joint;

        /************通过joint_pub发送到控制器************/

        sim_joint.q_abad_fl = q_ctrl_[0];
        sim_joint.q_abad_fr = q_ctrl_[3];
        sim_joint.q_abad_rl = q_ctrl_[6];
        sim_joint.q_abad_rr = q_ctrl_[9];

        sim_joint.q_hip_fl = q_ctrl_[1];
        sim_joint.q_hip_fr = q_ctrl_[4];
        sim_joint.q_hip_rl = q_ctrl_[7];
        sim_joint.q_hip_rr = q_ctrl_[10];

        sim_joint.q_knee_fl = q_ctrl_[2];
        sim_joint.q_knee_fr = q_ctrl_[5];
        sim_joint.q_knee_rl = q_ctrl_[8];
        sim_joint.q_knee_rr = q_ctrl_[11];

        sim_joint.qd_abad_fl = dq_ctrl_[0];
        sim_joint.qd_abad_fr = dq_ctrl_[3];
        sim_joint.qd_abad_rl = dq_ctrl_[6];
        sim_joint.qd_abad_rr = dq_ctrl_[9];

        sim_joint.qd_hip_fl = dq_ctrl_[1];
        sim_joint.qd_hip_fr = dq_ctrl_[4];
        sim_joint.qd_hip_rl = dq_ctrl_[7];
        sim_joint.qd_hip_rr = dq_ctrl_[10];

        sim_joint.qd_knee_fl = dq_ctrl_[2];
        sim_joint.qd_knee_fr = dq_ctrl_[5];
        sim_joint.qd_knee_rl = dq_ctrl_[8];
        sim_joint.qd_knee_rr = dq_ctrl_[11];

        sim_joint.tau_abad_fl = tau_ctrl_[0];
        sim_joint.tau_abad_fr = tau_ctrl_[3];
        sim_joint.tau_abad_rl = tau_ctrl_[6];
        sim_joint.tau_abad_rr = tau_ctrl_[9];

        sim_joint.tau_hip_fl = tau_ctrl_[1];
        sim_joint.tau_hip_fr = tau_ctrl_[4];
        sim_joint.tau_hip_rl = tau_ctrl_[7];
        sim_joint.tau_hip_rr = tau_ctrl_[10];

        sim_joint.tau_knee_fl = tau_ctrl_[2];
        sim_joint.tau_knee_fr = tau_ctrl_[5];
        sim_joint.tau_knee_rl = tau_ctrl_[8];
        sim_joint.tau_knee_rr = tau_ctrl_[11];

        // 发布消息
        this->joint_pub->publish(sim_joint);

        // 读取身体状态
        ignition::math::Pose3d base_pose;
        ignition::math::Vector3d base_vel;
        ignition::math::Vector3d base_omega;

        for (auto &it : model_->GetLinks())
        {
            if (it->GetName() == "base_link")
            {
                base_pose = it->WorldPose();
                base_vel = it->WorldLinearVel();
                base_omega = it->WorldAngularVel();
            }
        }


        ros_bridge::msg::SimBase sim_base;

        q_body_.w() = base_pose.Rot().W();
        q_body_.x() = base_pose.Rot().X(),
        q_body_.y() = base_pose.Rot().Y(),
        q_body_.z() = base_pose.Rot().Z(),

        q_body_.normalize();

        sim_base.position_x = base_pose.Pos()[0];
        sim_base.position_y = base_pose.Pos()[1];
        sim_base.position_z = base_pose.Pos()[2];
        sim_base.orientation_w = q_body_.w();
        sim_base.orientation_x = q_body_.x();
        sim_base.orientation_y = q_body_.y();
        sim_base.orientation_z = q_body_.z();
        

        //Eigen::Vector3d eulerAngle = q_body_.matrix().eulerAngles(2, 1, 0);


        sim_base.linear_velocity_x = base_vel[0];
        sim_base.linear_velocity_y = base_vel[1];
        sim_base.linear_velocity_z = base_vel[2];
        sim_base.angular_velocity_x = base_omega[0];
        sim_base.angular_velocity_y = base_omega[1];
        sim_base.angular_velocity_z = base_omega[2];
        
        this->base_pub->publish(sim_base);
        // sim_base.linear_velocity = q_body_.inverse()* sim_base.linear_velocity;
        // simToRobot.cheaterState.omegaBody = q.conjugate() * simToRobot.cheaterState.omegaBody;
    }

}

   