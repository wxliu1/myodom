// created by wxliu on 2023-11-29

#include "myodom.h"
#include <boost/circular_buffer.hpp>

using namespace wx;

CMyOdom::CMyOdom()
  : Node("CMyOdom")
  , sub_myodom_(this, "/my_odom")
  , sub_atp_(this, "/atp_info")
{
  // sync_odom_.emplace(sub_myodom_, sub_atp_, 5);
  // sync_odom_->registerCallback(&CMyOdom::OdomCb, this);

  sync_odom_ = new message_filters::Synchronizer<SyncOdom> (SyncOdom(10), sub_myodom_, sub_atp_);
  // sync_odom_->registerCallback(boost::bind(&CMyOdom::OdomCb, this, _1, _2));
  // sync_odom_->registerCallback(std::bind(&CMyOdom::OdomCb, this, std::placeholders::_1, std::placeholders::_2));
  sync_odom_->registerCallback(&CMyOdom::OdomCb, this);

  // pub_my_odom_ = this->create_publisher<sensor_msgs::msg::Imu>("/plot", 10);
  pub_result_ = this->create_publisher<myresult::msg::MyResult>("/plot", 10);
 
}

#define _PUBLISH_VELOCITY_DIRECTLY_

void CMyOdom::OdomCb(const myodom::msg::MyOdom::SharedPtr &myodom_ptr,
        const atp_info::msg::Atp::SharedPtr &atp_ptr)
{
  // std::cout << "received msgs.\n";
  static double atp_calc_odom = 0.0;
  if(atp_calc_odom < 1e-6)
  {
    atp_calc_odom = (atp_ptr->atp_calc_odom / 100.0);
  }

#ifdef _PUBLISH_VELOCITY_DIRECTLY_
  myresult::msg::MyResult result_msg;
  result_msg.header = myodom_ptr->header;
  result_msg.our_velocity = myodom_ptr->velocity;
  result_msg.atp_velocity = atp_ptr->atp_speed / 100.0;
  result_msg.velocity_error = result_msg.our_velocity - result_msg.atp_velocity;

  if(result_msg.our_velocity > 0.5 && result_msg.atp_velocity > 0.5)
    // odom_msg.angular_velocity.x = odom_msg.linear_acceleration.z / odom_msg.linear_acceleration.y * 100;
    result_msg.velocity_err_ratio = result_msg.velocity_error / result_msg.atp_velocity;// * 10 or * 100

  result_msg.our_period_odom = myodom_ptr->period_odom;
  result_msg.atp_period_odom = atp_ptr->atp_period_odom / 100.0;

  result_msg.our_total_odom = myodom_ptr->total_odom;
  result_msg.atp_total_odom = (atp_ptr->atp_calc_odom / 100.0) - atp_calc_odom;

  // publish velocity, period odom and total odom.
  pub_result_->publish(result_msg);

/*
  sensor_msgs::msg::Imu odom_msg;
  odom_msg.header = myodom_ptr->header;

  odom_msg.linear_acceleration.x = myodom_ptr->velocity;
  odom_msg.linear_acceleration.y = atp_ptr->atp_speed / 100.0;
  odom_msg.linear_acceleration.z = odom_msg.linear_acceleration.x - odom_msg.linear_acceleration.y;

  if(odom_msg.linear_acceleration.x > 0.5 && odom_msg.linear_acceleration.y > 0.5)
    odom_msg.angular_velocity.x = odom_msg.linear_acceleration.z / odom_msg.linear_acceleration.y * 100;
  // odom_msg.angular_velocity.y;
  // odom_msg.angular_velocity.z;

  // publish velocity, period odom and total odom.
  pub_my_odom_->publish(odom_msg);
  */
#else

  static constexpr int CNT = 5;
  static int count = 0;
  static double velocity1 = 0.0;
  static double velocity2 = 0.0;
  static double odom1 = 0.0;
  static double odom2 = 0.0;
  
  count++;
  velocity1 +=  myodom_ptr->velocity;
  velocity2 +=  (atp_ptr->atp_speed / 100.0);
  odom1 += myodom_ptr->period_odom;
  odom2 += (atp_ptr->atp_period_odom / 100.0);
  if(count == CNT)
  {
    velocity1 = velocity1 / CNT;
    velocity2 = velocity2 / CNT;

    odom1 = odom1 / CNT;
    odom2 = odom2 / CNT;

    count = 0;
  }

  if(count == 0)
  {
    // sensor_msgs::msg::Imu odom_msg;
    // odom_msg.header = myodom_ptr->header;

    myresult::msg::MyResult result_msg;
    result_msg.header = myodom_ptr->header;
    // float32 our_velocity
    // float32 atp_velocity
    // float32 velocity_error
    // float32 velocity_err_ratio
    // float32 our_period_odom
    // float32 atp_period_odom
    // float32 our_total_odom
    // float32 atp_total_odom

    result_msg.our_velocity = velocity1;
    result_msg.atp_velocity = velocity2;
    result_msg.velocity_error = result_msg.our_velocity - result_msg.atp_velocity;

    if(result_msg.our_velocity > 0.5 && result_msg.atp_velocity > 0.5)
      // odom_msg.angular_velocity.x = odom_msg.linear_acceleration.z / odom_msg.linear_acceleration.y * 100;
      result_msg.velocity_err_ratio = result_msg.velocity_error / atp_velocity;// * 10 or * 100

    result_msg.our_period_odom = odom1;
    result_msg.atp_period_odom = odom2;

    result_msg.our_total_odom = myodom_ptr->total_odom;
    result_msg.atp_total_odom = (atp_ptr->atp_calc_odom / 100.0) - atp_calc_odom;

    // publish velocity, period odom and total odom.
    pub_result_->publish(result_msg);


    velocity1 = 0.0;
    velocity2 = 0.0;
    odom1 = 0.0;
    odom2 = 0.0;
  }

#endif
  
}