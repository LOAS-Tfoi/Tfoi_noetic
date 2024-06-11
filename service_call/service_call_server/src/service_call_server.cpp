#include "ros/ros.h"
#include <chrono>
#include <geometry_msgs/Twist.h>
#include "service_call_server/RobotTurning.h"

using Request_T = service_call_server::RobotTurning::Request;
using Response_T = service_call_server::RobotTurning::Response;

class TfoiBot{
private:
    ros::NodeHandle nh;
    ros::ServiceServer control_service;
    ros::Publisher vel_pub;

    geometry_msgs::Twist cmd_vel;
    std::string name;

public:
    TfoiBot(const std::string &name_in = "tfoibot") : name(name_in) {
        control_service = nh.advertiseService("/turning_robot", &TfoiBot::servCallback, this);
        vel_pub = nh.advertise<geometry_msgs::Twist>("/smoother_cmd_vel", 5);
        ROS_INFO("Service Server and Publisher initialized");
        ROS_INFO("Waiting for request... ");
    }

    bool servCallback(Request_T &req, Response_T &res) {
        auto start = std::chrono::steady_clock::now();

        if (req.command == "rot_on") {
            ROS_INFO(" ==== Start Immediate 360 Turning ==== ");
            cmd_vel.angular.z = 0.5;
            vel_pub.publish(cmd_vel);
        }        
        else if (req.command == "rot_off")
        {
            ROS_INFO(" ==== rotation off ==== ");

            // cmd_vel.angular.z = 0.0; // 각속도를 0으로 설정하여 회전을 중지합니다.
            // vel_pub.publish(cmd_vel); // 회전을 중지하는 명령을 발행합니다.

            //ros::Duration(1.0).sleep(); // 1초 동안 대기합니다.
        }
        else
        {
            ROS_ERROR("Unknown command received.");
            return false;
        }
        return true; // 서비스 콜백이 성공적으로 실행되었음을 반환합니다.
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "service_call_server_node");
    TfoiBot myTfoiBot;

    ros::spin();

    return 0;
}
