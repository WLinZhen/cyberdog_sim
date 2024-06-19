
#include <ignition/math/Color.hh>
#include <gazebo/common/Events.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/Node.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/gazebo.hh>
#include <boost/bind.hpp>
#include "gazebo/rendering/RenderTypes.hh"
#include "gazebo/rendering/Visual.hh"
#include "gazebo/rendering/Scene.hh"
#include <gazebo/rendering/rendering.hh>
#include <gazebo/msgs/visual.pb.h>
#include <gazebo/rendering/DynamicLines.hh>
#include "rclcpp/rclcpp.hpp"
#include "ros_bridge/msg/traj_visual.hpp"

namespace gazebo
{
    class TrajPlugin : public VisualPlugin
    {
        public:
            TrajPlugin();
            ~TrajPlugin();
            void Load(rendering::VisualPtr _parent, sdf::ElementPtr _sdf );
            void OnUpdate();
            void RosCom(const ros_bridge::msg::TrajVisual msg);
            
        private:
            rclcpp::Node::SharedPtr node;
            //base 发布者
            rclcpp::Subscription<ros_bridge::msg::TrajVisual>::SharedPtr traj_sub;
            rendering::VisualPtr        visual;
            rendering::DynamicLines     *baseline;
            rendering::DynamicLines     *flline;
            rendering::DynamicLines     *frline;
            rendering::DynamicLines     *rlline;
            rendering::DynamicLines     *rrline;
            double baseX = 0;
            double baseY = 0;
            double baseZ = 0;

            double flX = 0;
            double flY = 0;
            double flZ = 0;

            double frX = 0;
            double frY = 0;
            double frZ = 0;

            double rlX = 0;
            double rlY = 0;
            double rlZ = 0;

            double rrX = 0;
            double rrY = 0;
            double rrZ = 0;

            event::ConnectionPtr update_connection;
    };
}