// created by wxliu on 2023-11-29

#ifndef _MY_ODOM_H_
#define _MY_ODOM_H_

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <thread>

// #include "rclcpp/rclcpp.hpp"

// #include <sensor_msgs/msg/imu.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
//#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/time_synchronizer.h>

// #include <myodom/MyOdom.h>
#include <myodom/MyOdom.h>
#include <atp_info/atp.h>
#include<myresult/MyResult.h>

// #include <Eigen/Dense>

using namespace std::chrono_literals;

using std::placeholders::_1;


namespace liu::wx {

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

    void reset()
    {
        beacon_id = 0;
        atp_calc_odom1 = 0;
        atp_calc_odom2 = 0;
        our_odom1 = 0;
        our_odom2 = 0;
        atp_count = 0;
        atp_t1 = 0;
        atp_t2 = 0;
        our_t1 = 0;
        our_t2 = 0;

        atp_odom = 0;
        our_odom = 0;
        atp_duration = 0;
        our_duration = 0;
    }

    void print()
    {
#if 0        
        double atp_velocity = (atp_calc_odom2 - atp_calc_odom1) / (100.0 * (atp_t2 - atp_t1));
        double our_velocity = (our_odom2 - our_odom1) / (our_t2 - our_t1);

        
        std::cout << "--- Compare ---" << std::endl
            // << "beacon_id: " << beacon_id << std::endl
            // << "atp_calc_odom1: " << atp_calc_odom1 << std::endl
            // << "atp_calc_odom2: " << atp_calc_odom2 << std::endl
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
            << "Percentage of precision: " << fabs(our_velocity - atp_velocity) * 100 / atp_velocity << "%" << std::endl
            << "---THE END---" << std::endl << std::endl;
#else        
        double atp_velocity = atp_odom / atp_duration;
        double our_velocity = our_odom / our_duration;
        std::cout << "--- Compare ---" << std::endl
            << "atp_count: " << atp_count << std::endl
            << "atp_odom: " << atp_odom << std::endl
            << "our_odom: " << our_odom << std::endl
            << "atp_duration: " << atp_duration << std::endl
            << "our_duration: " << our_duration << std::endl
            << "atp_velocity: " << atp_velocity << std::endl
            << "our_velocity: " << our_velocity << std::endl
            // << "Percentage of precision: " << fabs(our_velocity - atp_velocity) * 100 / atp_velocity << "%" << std::endl
            << "Precision percentage of the veloctiy: " << fabs(our_velocity - atp_velocity) * 100 / atp_velocity << "%" << std::endl
            << "------" << std::endl << std::endl
            << "atp_total_count: " << atp_total_count << std::endl
            << "atp_total_odom: " << atp_total_odom << std::endl
            << "our_total_odom: " << our_total_odom << std::endl
            << "atp_total_duration: " << atp_total_duration << std::endl
            << "our_total_duration: " << our_total_duration << std::endl
            << "atp_mean_velocity: " << atp_total_odom / atp_total_duration << std::endl
            << "our_mean_velocity: " << our_total_odom / our_total_duration << std::endl
            << "Precision percentage of the odom: " << fabs(our_total_odom - atp_total_odom) * 100 / atp_total_odom << "%" << std::endl
            << "---THE END---" << std::endl << std::endl;

#endif        

    }

    int beacon_id { 0 };
    int atp_calc_odom1 { 0 };
    int atp_calc_odom2 { 0 };
    double our_odom1 { 0 };
    double our_odom2 { 0 };
    int atp_count { 0 };
    double atp_t1 { 0 };
    double atp_t2 { 0 };
    double our_t1 { 0 };
    double our_t2 { 0 };

    double atp_odom { 0 };
    double our_odom { 0 };
    double atp_duration { 0 };
    double our_duration { 0 };

    int atp_total_count { 0 };
    double atp_total_odom { 0 };
    double our_total_odom { 0 };
    double atp_total_duration { 0 };
    double our_total_duration { 0 };


}TCompare;

class CMyOdom// : public rclcpp::Node
{
public:    
    CMyOdom(const ros::NodeHandle& pnh);
    ~CMyOdom();


private:    
    void OdomCb(const myodom::MyOdomConstPtr &myodom_ptr,
        // const atp_info::AtpConstPtr &atp_ptr);
        const atp_info::atpConstPtr &atp_ptr);

    void command();    

public:

private:
    ros::NodeHandle pnh_;
    mf::Subscriber<myodom::MyOdom> sub_myodom_;
    // mf::Subscriber<atp_info::Atp> sub_atp_;
    mf::Subscriber<atp_info::atp> sub_atp_;

    // using SyncOdom = mf::TimeSynchronizer<myodom::msg::MyOdom, atp_info::msg::Atp>;
    // using SyncOdom = mf::ApproximateTimeSynchronizer<myodom::msg::MyOdom, atp_info::msg::Atp>;
    // std::optional<SyncOdom> sync_odom_;

    // typedef message_filters::sync_policies::ApproximateTime<myodom::MyOdom, atp_info::Atp> SyncOdom;
    typedef message_filters::sync_policies::ApproximateTime<myodom::MyOdom, atp_info::atp> SyncOdom;
    message_filters::Synchronizer<SyncOdom> *sync_odom_;

   
    // rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_my_odom_;
    // rclcpp::Publisher<myresult::MyResult>::SharedPtr pub_result_;
    ros::Publisher pub_result_;

    std::vector<TBeacon> vec_beacon_;
    std::vector<TCompare> vec_compare_;
    std::thread keyboard_command_process;

};

}

#endif
