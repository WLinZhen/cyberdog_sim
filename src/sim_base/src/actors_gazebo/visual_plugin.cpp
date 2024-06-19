#include "actors_gazebo/visual_plugin.hpp"
namespace gazebo
{
    GZ_REGISTER_VISUAL_PLUGIN(TrajPlugin)
    TrajPlugin::TrajPlugin():baseline(NULL),flline(NULL),frline(NULL),rlline(NULL),rrline(NULL)
    {
        rclcpp::init(0,0);
        this->node = std::make_shared<rclcpp::Node>("traj_plugin_node");
    }
    
    TrajPlugin::~TrajPlugin()
    {
        
        this->visual->DeleteDynamicLine(this->baseline);
        this->visual->DeleteDynamicLine(this->flline);
        this->visual->DeleteDynamicLine(this->frline);
        this->visual->DeleteDynamicLine(this->rlline);
        this->visual->DeleteDynamicLine(this->rrline);
        rclcpp::shutdown();
    }
    void TrajPlugin::Load(rendering::VisualPtr _parent, sdf::ElementPtr _sdf )
    {
        std::cout << "**************进入可视化插件**************" << std::endl;
        this->visual = _parent;

        this->baseline = this->visual->CreateDynamicLine(rendering::RENDERING_LINE_STRIP);
        this->baseline->setMaterial("Gazebo/Blue");
        this->baseline->setVisibilityFlags(GZ_VISIBILITY_GUI);
        

        this->flline = this->visual->CreateDynamicLine(rendering::RENDERING_LINE_STRIP);
        this->flline->setMaterial("Gazebo/Purple");
        this->flline->setVisibilityFlags(GZ_VISIBILITY_GUI);


        this->frline = this->visual->CreateDynamicLine(rendering::RENDERING_LINE_STRIP);
        this->frline->setMaterial("Gazebo/Purple");
        this->frline->setVisibilityFlags(GZ_VISIBILITY_GUI);


        this->rlline = this->visual->CreateDynamicLine(rendering::RENDERING_LINE_STRIP);
        this->rlline->setMaterial("Gazebo/Purple");
        this->rlline->setVisibilityFlags(GZ_VISIBILITY_GUI);


        this->rrline = this->visual->CreateDynamicLine(rendering::RENDERING_LINE_STRIP);
        this->rrline->setMaterial("Gazebo/Green");
        this->rrline->setVisibilityFlags(GZ_VISIBILITY_GUI);
        this->visual->SetVisible(true);

        this->update_connection = event::Events::ConnectPreRender(boost::bind(&TrajPlugin::OnUpdate, this));
        if (rclcpp::ok())
        {
            this->traj_sub = this->node->create_subscription<ros_bridge::msg::TrajVisual>
                    ("traj_data", 10, std::bind(&TrajPlugin::RosCom, this, std::placeholders::_1));
        }
    }
    void TrajPlugin::RosCom(const ros_bridge::msg::TrajVisual msg)
    {
        baseX = msg.base_x;
        baseY = msg.base_y;
        baseZ = msg.base_z;

        flX = msg.fl_foot_x;
        flY = msg.fl_foot_y;
        flZ = msg.fl_foot_z;

        frX = msg.fr_foot_x;
        frY = msg.fr_foot_y;
        frZ = msg.fr_foot_z;

        rlX = msg.rl_foot_x;
        rlY = msg.rl_foot_y;
        rlZ = msg.rl_foot_z;

        rrX = msg.rr_foot_x;
        rrY = msg.rr_foot_y;
        rrZ = msg.rr_foot_z;
    }
    void TrajPlugin::OnUpdate()
    {
        rclcpp::spin_some(this->node);
        // this->line->SetPoint(1, ignition::math::Vector3d(Fx, Fy, Fz));
        this->baseline->AddPoint(ignition::math::Vector3d(baseX, baseY, baseZ), ignition::math::Color(0, 1, 1, 1.0));
        this->flline->AddPoint(ignition::math::Vector3d(flX,flY, flZ), ignition::math::Color(0.5, 0.2, 1, 1.0));
        // this->frline->AddPoint(ignition::math::Vector3d(frX, frY, frZ), ignition::math::Color(1, 0, 1, 1.0));
        // this->rlline->AddPoint(ignition::math::Vector3d(rlX, rlY, rlZ), ignition::math::Color(0, 2, 1, 1.0));
        this->rrline->AddPoint(ignition::math::Vector3d(rrX, rrY, rrZ), ignition::math::Color(0.2, 0.5, 1, 1.0));
        // ROS_INFO("Fx= %d, Fy= %d, Fz= %d", Fx, Fy, Fz);
    }
}