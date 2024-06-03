#include "actors_rviz/state_publisher.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<StatePublisher>();

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}