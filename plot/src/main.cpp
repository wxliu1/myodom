// 2023-11-29

#include "myodom.h"

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<wx::CMyOdom>());
    rclcpp::shutdown();

    return 0;
}