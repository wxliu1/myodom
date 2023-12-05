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

#include <vector>

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

typedef struct _TBeacon
{
    int beacon_id;
    int beacon_odom;
}TBeacon;

typedef struct _TCompare
{
    _TCompare()
    {
        atp_count = 0;
        
    }

    void print()
    {
        double atp_velocity = (atp_calc_odom2 - atp_calc_odom1) / (100.0 * (atp_t2 - atp_t1));
        double our_velocity = (our_odom2 - our_odom1) / (our_t2 - our_t1);

        
        std::cout << "beacon_id: " << beacon_id << std::endl
            << "atp_calc_odom1: " << atp_calc_odom1 << std::endl
            << "atp_calc_odom2: " << atp_calc_odom2 << std::endl
            // << "our_odom1: " << our_odom1 << std::endl
            // << "our_odom2: " << our_odom2 << std::endl
            << "atp_count: " << atp_count << std::endl
            // << "atp_t1: " << atp_t1 << std::endl
            // << "atp_t2: " << atp_t2 << std::endl
            // << "our_t1: " << our_t1 << std::endl
            // << "our_t2: " << our_t2 << std::endl

            << "atp_delta_odom: " << (atp_calc_odom2 - atp_calc_odom1) / 100.0 << std::endl
            << "our_delta_odom: " << (our_odom2 - our_odom1) << std::endl
            << "atp_duration: " << (atp_t2 - atp_t1) << std::endl
            << "our_duration: " << (our_t2 - our_t1) << std::endl
            << "atp_velocity: " << atp_velocity << std::endl
            << "our_velocity: " << our_velocity << std::endl
            << "---THE END---" << std::endl << std::endl;

    }

    int beacon_id;
    int atp_calc_odom1;
    int atp_calc_odom2;
    double our_odom1;
    double our_odom2;
    int atp_count;
    double atp_t1;
    double atp_t2;
    double our_t1;
    double our_t2;
}TCompare;

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

    std::vector<TBeacon> vec_beacon_;
    std::vector<TCompare> vec_compare_;

};

}

#endif
