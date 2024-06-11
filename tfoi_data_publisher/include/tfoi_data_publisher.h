#ifndef __WEB_DATA_CLASS_H_
#define __WEB_DATA_CLASS_H_

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include "tfoi_data_publisher/bms_fb.h"
#include "tfoi_data_publisher/bms_flag_fb.h"

class DataPublisher
{

private :

    ros::NodeHandle nh;
    ros::Subscriber bms_fb_sub;
    ros::Publisher bms_fb_pub_voltage;
    ros::Publisher bms_fb_pub_current;
    ros::Publisher bms_fb_pub_capacity;
    ros::Publisher bms_fb_pub_battery;                  // 배터리 잔량 표시(계산)

    ros::Subscriber bms_flag_fb_sub;
    ros::Publisher bms_flag_fb_pub_ov;                  //  배터리 전압 허용 범위 초과
    ros::Publisher bms_flag_fb_pub_uv;                  //  배터리 전압 허용 범위 미만
    ros::Publisher bms_flag_fb_pub_charge_ot;           //  배터리 과온
    ros::Publisher bms_flag_fb_pub_charge_ut;           //  배터리 저온
    ros::Publisher bms_flag_fb_pub_hight_temp;          //  배터리의 높은 온도 상태
    ros::Publisher bms_flag_fb_pub_low_temp;            //  배터리의 낮은 온도 상태 
    ros::Publisher bms_flag_fb_pub_charge_flag;         //  배터리 충전서태 플래그
    ros::Publisher bms_flag_fb_pub_soc;                 //  배터리 잔량 표시(Can Data)
   


    nav_msgs::Odometry::ConstPtr last_odom_msg;
    ros::Subscriber cmd_vel_sub;
    ros::Subscriber odom_sub;
    ros::Publisher odom_pub_distance;

public :

    DataPublisher();   

    float total_distance;

    void BmsFbCallBack(const tfoi_data_publisher::bms_fb::ConstPtr& bms_fb_msg);

    void BmsFlagFbCallBack(const tfoi_data_publisher::bms_flag_fb::ConstPtr& bms_fb_flag_msg);
    
    void OdomCallBack(const nav_msgs::Odometry::ConstPtr& odom_msg);

    void CmdVelCallBack(const geometry_msgs::Twist::ConstPtr& cmd_vel_msg); 

    ~DataPublisher();
};

#endif