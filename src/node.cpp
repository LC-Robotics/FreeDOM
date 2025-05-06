//
// Created by ZhiangQi on 25-5-5.
//
#include <FreeDOM-ROS2/freenode.h>

using namespace freedom;
int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<FreeNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}