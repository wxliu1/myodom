// created by wxliu on 2023-11-29

#ifndef _MY_ODOM_H_
#define _MY_ODOM_H_

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"

// #include <sensor_msgs/msg/imu.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
//#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/time_synchronizer.h>

#include <myodom/msg/my_odom.hpp>
#include <atp_info/msg/atp.hpp>
#include<myresult/msg/my_result.hpp>

// #include <Eigen/Dense>

using namespace std::chrono_literals;

using std::placeholders::_1;


namespace wx {

// using Vector2d = Eigen::Vector2d;
// using Vector3d = Eigen::Vector3d;
// using Matrix3d = Eigen::Matrix3d;
// using Quaterniond = Eigen::Quaterniond; 
// namespace cb = cv_bridge;
// namespace sm = sensor_msgs;
// namespace gm = geometry_msgs;
namespace mf = message_filters;

class CMyOdom : public rclcpp::Node
{
public:    
    CMyOdom();


private:    
    void OdomCb(const myodom::msg::MyOdom::SharedPtr &myodom_ptr,
        const atp_info::msg::Atp::SharedPtr &atp_ptr);

public:

private:
    mf::Subscriber<myodom::msg::MyOdom> sub_myodom_;
    mf::Subscriber<atp_info::msg::Atp> sub_atp_;

    // using SyncOdom = mf::TimeSynchronizer<myodom::msg::MyOdom, atp_info::msg::Atp>;
    // using SyncOdom = mf::ApproximateTimeSynchronizer<myodom::msg::MyOdom, atp_info::msg::Atp>;
    // std::optional<SyncOdom> sync_odom_;

    typedef message_filters::sync_policies::ApproximateTime<myodom::msg::MyOdom, atp_info::msg::Atp> SyncOdom;
    message_filters::Synchronizer<SyncOdom> *sync_odom_;

   
    // rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_my_odom_;
    rclcpp::Publisher<myresult::msg::MyResult>::SharedPtr pub_result_;

};

}

#endif
