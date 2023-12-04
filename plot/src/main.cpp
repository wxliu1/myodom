// 2023-11-29

#include "myodom.h"

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "plot_result");  // node name
    ros::MultiThreadedSpinner spinner(6);  // use 6 threads
    // ros::NodeHandle n("~");
    // CMyOdom node{n};
    liu::wx::CMyOdom node{ros::NodeHandle{"~"}};

    // ros::spin(); // comment 
    spinner.spin();  // add this line 2023-12-04

    // rclcpp::init(argc, argv);
    // rclcpp::spin(std::make_shared<wx::CMyOdom>());
    // rclcpp::shutdown();

    return 0;
}