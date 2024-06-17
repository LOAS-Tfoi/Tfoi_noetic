#include "tfoi_data_publisher.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Bool.h"
#include "std_msgs/UInt8.h"

DataPublisher::DataPublisher()
{
    bms_fb_sub = nh.subscribe("/bms_fb", 1, &DataPublisher::BmsFbCallBack, this);    
    bms_fb_pub_voltage = nh.advertise<std_msgs::Float32>("/tfoi_voltage", 1);
    bms_fb_pub_current = nh.advertise<std_msgs::Float32>("/tfoi_current", 1);
    bms_fb_pub_capacity = nh.advertise<std_msgs::Float32>("/tfoi_capacity", 1);
    bms_fb_pub_battery = nh.advertise<std_msgs::UInt8>("/tfoi_battery", 1);

    bms_flag_fb_sub = nh.subscribe("/bms_flag_fb", 1, &DataPublisher::BmsFlagFbCallBack, this);
    bms_flag_fb_pub_ov = nh.advertise<std_msgs::Bool>("/tfoi_over_voltage", 1);
    bms_flag_fb_pub_charge_ot = nh.advertise<std_msgs::Bool>("/T1/charge_over_temperature", 1);
    bms_flag_fb_pub_hight_temp = nh.advertise<std_msgs::Float32>("/tfoi_temperature", 1);
    bms_flag_fb_pub_charge_flag = nh.advertise<std_msgs::Bool>("/tfoi_charge_flag", 1);
    bms_flag_fb_pub_soc = nh.advertise<std_msgs::UInt8>("/tfoi_state_of_charge",1);

    odom_sub = nh.subscribe("/odom",1,&DataPublisher::OdomCallBack, this);
    odom_pub_distance = nh.advertise<std_msgs::Float32>("/tfoi_distance", 1);
    cmd_vel_sub = nh.subscribe("/tfoi_smoother_cmd_vel", 1, &DataPublisher::CmdVelCallBack,this);
         
    total_distance = 0.0;
    // voltage = 0;
    // current = 0;
    // capacity = 0;
};

void DataPublisher::BmsFbCallBack(const tfoi_data_publisher::bms_fb::ConstPtr& bms_fb_msg)
{
    std_msgs::Float32 voltage_msg;
    voltage_msg.data = bms_fb_msg->bms_fb_voltage;
    bms_fb_pub_voltage.publish(voltage_msg);

    std_msgs::Float32 current_msg;
    current_msg.data = bms_fb_msg->bms_fb_current;
    bms_fb_pub_current.publish(current_msg);

    std_msgs::Float32 capacity_msg;
    capacity_msg.data = bms_fb_msg->bms_fb_remaining_capacity;
    bms_fb_pub_capacity.publish(capacity_msg);

    //std_msgs::UInt8 battery_msg;
    //float capacity_battery_diff_percent = 100 / (29.5 - 4.0);
    //float min_capacity = 4.0;
    //battery_msg.data = static_cast<int>((bms_fb_msg->bms_fb_remaining_capacity - min_capacity) * capacity_battery_diff_percent);  
	//bms_fb_pub_battery.publish(battery_msg); 

};

void DataPublisher::BmsFlagFbCallBack(const tfoi_data_publisher::bms_flag_fb::ConstPtr& bms_fb_flag_msg)
{
    std_msgs::Bool over_voltage_msg;
    over_voltage_msg.data = bms_fb_flag_msg->bms_flag_fb_ov;
    bms_flag_fb_pub_ov.publish(over_voltage_msg);

    std_msgs::Bool charge_over_temp_msg;
    charge_over_temp_msg.data = bms_fb_flag_msg->bms_flag_fb_charge_ot;
    bms_flag_fb_pub_charge_ot.publish(charge_over_temp_msg);

    std_msgs::Float32 hight_temp_msg;
    hight_temp_msg.data = bms_fb_flag_msg->bms_flag_fb_hight_temperature;
    bms_flag_fb_pub_hight_temp.publish(hight_temp_msg);

    std_msgs::Bool charge_flag_msg;
    charge_flag_msg.data = bms_fb_flag_msg->bms_flag_fb_charge_flag;
    bms_flag_fb_pub_charge_flag.publish(charge_flag_msg);

    //std_msgs::UInt8 charge_state_msg;
    //charge_state_msg.data = bms_fb_flag_msg->bms_flag_fb_soc;
    //bms_flag_fb_pub_soc.publish(charge_state_msg);

	std_msgs::UInt8 battery_msg;
	battery_msg.data = bms_fb_flag_msg->bms_flag_fb_soc;
	bms_fb_pub_battery.publish(battery_msg); 

};

void DataPublisher::OdomCallBack(const nav_msgs::Odometry::ConstPtr& odom_msg)
{
    std_msgs::Float32 distance_msg;
    if(last_odom_msg)
    {
        float delta_x =  odom_msg->pose.pose.position.x - last_odom_msg->pose.pose.position.x;
        float delta_y =  odom_msg->pose.pose.position.y - last_odom_msg->pose.pose.position.y;
        float delta_th = odom_msg->pose.pose.orientation.z;

        float delta_dis = sqrt(delta_x * delta_x + delta_y * delta_y);

        total_distance += delta_dis;
        distance_msg.data = total_distance;
    }
    last_odom_msg = odom_msg;
    odom_pub_distance.publish(distance_msg);
};

void DataPublisher::CmdVelCallBack(const geometry_msgs::Twist::ConstPtr& cmd_vel_msg)
{
    double linear_x =  cmd_vel_msg->linear.x;
    double angular_z = cmd_vel_msg->angular.z;
}

DataPublisher::~DataPublisher() {}
