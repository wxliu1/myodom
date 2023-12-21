// created by wxliu on 2023-11-29

#include "myodom.h"
#include <boost/circular_buffer.hpp>
#include <cmath>

// #define _PRINT_BEACON_
#define _CONFIDENCE_LEVEL_

#define _PUBLISH_VELOCITY_DIRECTLY_

using namespace liu::wx;

static TCompare compare;

CMyOdom::CMyOdom(const ros::NodeHandle& pnh)
  // : Node("CMyOdom")
  : pnh_(pnh)
  , sub_myodom_(pnh_, "/my_odom", 5)
  , sub_atp_(pnh_, "/atp_info", 5)
{
  // sync_odom_.emplace(sub_myodom_, sub_atp_, 5);
  // sync_odom_->registerCallback(&CMyOdom::OdomCb, this);

  sync_odom_ = new message_filters::Synchronizer<SyncOdom> (SyncOdom(10), sub_myodom_, sub_atp_);
  // sync_odom_->registerCallback(boost::bind(&CMyOdom::OdomCb, this, _1, _2));
  // sync_odom_->registerCallback(std::bind(&CMyOdom::OdomCb, this, std::placeholders::_1, std::placeholders::_2));
  sync_odom_->registerCallback(&CMyOdom::OdomCb, this);

  // pub_my_odom_ = this->create_publisher<sensor_msgs::msg::Imu>("/plot", 10);
  // pub_result_ = this->create_publisher<myresult::msg::MyResult>("/plot", 10);
  // pub_result_ = pnh_.subscribe("/plot", 10, &CMyOdom::cb_func, this);
  pub_result_ = pnh_.advertise<myresult::MyResult>("/plot", 10);
 
  keyboard_command_process = std::thread(&CMyOdom::command, this);
}

CMyOdom::~CMyOdom()
{
  keyboard_command_process.detach();
}

void CMyOdom::command()
{
  while (1)
  {

    char c = getchar();
    if (c == 's')
    {
      std::cout << "press 's'" << std::endl;
      if(compare.atp_count > 0)
      {
        std::cout << "print the last comparison.\n";
        compare.print();
        compare.reset();
      }
    }


    std::chrono::milliseconds dura(500);
    std::this_thread::sleep_for(dura);
  }
}



// void CMyOdom::OdomCb(const myodom::msg::MyOdom::SharedPtr &myodom_ptr,
//         const atp_info::msg::Atp::SharedPtr &atp_ptr)
void CMyOdom::OdomCb(const myodom::MyOdomConstPtr &myodom_ptr,
        const atp_info::atpConstPtr &atp_ptr)   
{
  // std::cout << "received msgs.\n";

#ifdef _CONFIDENCE_LEVEL_
  // static TCompare compare;
  bool confidence_coefficient = myodom_ptr->confidence_coefficient;
  if(!confidence_coefficient)
  {
    // print odom
    if(compare.atp_count > 0)
    {
      compare.print();
      compare.reset();
    }
  }
  else
  {
  #if 0  
    if(compare.atp_count == 0)
    {
      compare.atp_count = 1;
      compare.atp_calc_odom1 = atp_ptr->atp_calc_odom;
      compare.our_odom1 = myodom_ptr->total_odom;
      compare.atp_t1 = atp_ptr->header.stamp.sec + atp_ptr->header.stamp.nsec * (1e-9);
      compare.our_t1 = myodom_ptr->header.stamp.sec + myodom_ptr->header.stamp.nsec * (1e-9);
    }
    else
    {
      compare.atp_count ++;
      compare.atp_calc_odom2 = atp_ptr->atp_calc_odom;
      compare.our_odom2 = myodom_ptr->total_odom;
      compare.atp_t2 = atp_ptr->header.stamp.sec + atp_ptr->header.stamp.nsec * (1e-9);
      compare.our_t2 = myodom_ptr->header.stamp.sec + myodom_ptr->header.stamp.nsec * (1e-9);
    }
  #endif

    compare.atp_count ++;
    compare.atp_odom += atp_ptr->atp_period_odom / 100.0;
    compare.our_odom += myodom_ptr->period_odom;

    // compare.atp_duration += atp_ptr->atp_period_odom * 1.0 / atp_ptr->atp_speed; //if speed is 0 than NaN come out.
    compare.atp_duration += 0.2;
    compare.our_duration += myodom_ptr->delta_time;

    // if(std::isnan(atp_ptr->atp_period_odom * 1.0 / atp_ptr->atp_speed) || std::isnan(compare.atp_duration))
    // {
    //   std::cout << "atp_speed :" << atp_ptr->atp_speed << " atp_ptr->atp_period_odom : " << atp_ptr->atp_period_odom << std::endl;
    // }

    compare.atp_total_count ++;
    compare.atp_total_odom += atp_ptr->atp_period_odom / 100.0;
    compare.our_total_odom += myodom_ptr->period_odom;

    // compare.atp_duration += atp_ptr->atp_period_odom * 1.0 / atp_ptr->atp_speed; //if speed is 0 than NaN come out.
    compare.atp_total_duration += 0.2;
    compare.our_total_duration += myodom_ptr->delta_time;

    
  }

  return;
#endif


#ifdef _PRINT_BEACON_
  //if(1)
  {
    static int prev_beacon_id = 0;
    TBeacon beacon;
    beacon.beacon_id = atp_ptr->beacon_id;
    beacon.beacon_odom = atp_ptr->beacon_odom;
    bool bl = false;
    // std::vector<TBeacon>::iterator iter = vec_beacon_.begin();
    for(auto element : vec_beacon_)
    {
      if(element.beacon_id == beacon.beacon_id)
      {
        bl = true;
        break ;
      }
    }

    if(!bl)
    {
      // found new beacon.
      vec_beacon_.emplace_back(beacon);

      TCompare compare;
      compare.beacon_id = beacon.beacon_id;
      compare.atp_count = 1;
      compare.atp_calc_odom1 = atp_ptr->atp_calc_odom;
      compare.our_odom1 = myodom_ptr->total_odom;
      compare.atp_t1 = atp_ptr->header.stamp.sec + atp_ptr->header.stamp.nsec * (1e-9);
      compare.our_t1 = myodom_ptr->header.stamp.sec + myodom_ptr->header.stamp.nsec * (1e-9);
      vec_compare_.emplace_back(compare);

      if(prev_beacon_id > 0)
      {
        for(auto element : vec_compare_)
        {
          if(element.beacon_id == prev_beacon_id)
          {
            element.print();
            break ;
          }
        }

      }

      prev_beacon_id = beacon.beacon_id;

    }
    else
    {
      // find element in vec_compare_
      for(auto &element : vec_compare_)
      {
        if(element.beacon_id == beacon.beacon_id)
        {
          element.atp_count ++;
          element.atp_calc_odom2 = atp_ptr->atp_calc_odom;
          element.our_odom2 = myodom_ptr->total_odom;
          element.atp_t2 = atp_ptr->header.stamp.sec + atp_ptr->header.stamp.nsec * (1e-9);
          element.our_t2 = myodom_ptr->header.stamp.sec + myodom_ptr->header.stamp.nsec * (1e-9);
          break ;
        }
      }
      
    }

    return ;
  }
#endif

  static double atp_calc_odom = 0.0;
  if(atp_calc_odom < 1e-6)
  {
    atp_calc_odom = (atp_ptr->atp_calc_odom / 100.0);
  }

#ifdef _PUBLISH_VELOCITY_DIRECTLY_
  myresult::MyResult result_msg;
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
  // pub_result_->publish(result_msg);
  pub_result_.publish(result_msg);

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