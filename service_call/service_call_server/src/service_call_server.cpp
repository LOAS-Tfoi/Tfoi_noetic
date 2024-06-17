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
    ros::Time start_time_;
    ros::Rate loop_rate{10};

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
            ROS_INFO(" ==== Start 360 degree Turning ==== ");
            cmd_vel.angular.z = 0.442;
            vel_pub.publish(cmd_vel);
        }        
        else if (req.command == "rot_off")
        {
            ROS_INFO(" ==== rotation off ==== ");

            // cmd_vel.angular.z = 0.0; 
            // vel_pub.publish(cmd_vel); 
            //ros::Duration(1.0).sleep();
        }
        else if (req.command == "rotation_call")
        {
            ROS_INFO(" ==== service_call_rotation ==== ");
            start_time_ = ros::Time::now();

            // 회전 각속도 설정
            cmd_vel.angular.z = 0.5;

            ros::Rate loop_rate(100); // 초당 100번의 루프 실행 (10ms 간격)
            ros::Time current_time;

            // 시작 시간부터 일정 시간이 경과할 때까지 반복해서 회전 명령을 보냄
            while ((current_time = ros::Time::now()) - start_time_ < ros::Duration(11.0))
            {
                vel_pub.publish(cmd_vel);
                loop_rate.sleep(); // 10ms 간격으로 반복 실행

                // 7초가 지나면 루프를 빠져나감
                if ((current_time - start_time_).toSec() >= 11.0) {
                    break;
                    return false;
                    }               
            }
            
        }
        else
        {
            ROS_ERROR("Unknown command received.");
            return false;
        }
        return true; // 서비스 콜백이 성공적으로 실행되었음을 반환
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "service_call_server_node");
    TfoiBot myTfoiBot;

    ros::spin();

    return 0;
}
