#include "ros/ros.h"
#include "std_msgs/Int8.h"
#include "ros/callback_queue.h"
#include "service_call_client/RobotTurning.h"
#include <iostream>
#include <string>

std::string action_msgs;
void actionCallback(const std_msgs::Int8::ConstPtr& msg)
{
    if (msg->data == 1)
    {
        action_msgs = "rot_on";
    }
    else if(msg->data == 0)
    {
        action_msgs = "rot_off";
    }
    else
    {
        action_msgs = "";
    }
}


int main(int argc, char** argv) {
    ros::init(argc, argv, "service_call_client_node");
    ros::NodeHandle nh;

    // 서비스 클라이언트 객체 생성
    ros::ServiceClient client = nh.serviceClient<service_call_client::RobotTurning>("/turning_robot");

    service_call_client::RobotTurning srv;

    ROS_INFO("==== Service Client initialized ====");

    ros::Subscriber sub = nh.subscribe<std_msgs::Int8>("/action",10,actionCallback);

    ros::CallbackQueue queue;
    nh.setCallbackQueue(&queue);

    // Spin in a separate thread to handle callbacks
    ros::AsyncSpinner spinner(1, &queue);
    spinner.start();

    unsigned int turning_time;
    float angular_vel;
    std::string com;
    
    // 첫 번째 서비스 호출: 즉시 360도 회전
    // std::cout << "Enter the command : rot : 360_Rotation, DELAY_360, STOP_SERVICE" << std::endl;
    // std::cin >> com;
    // std::cin.clear(); std::cin.ignore(32767, '\n');
    while (ros::ok())
    {
      ros::spinOnce();
        com = action_msgs;
        if (!com.empty())
                {
                if (com == "rot_on")
                    {
                    srv.request.command = com;
                    //srv.request.turning_time = 14;  
                    if (client.call(srv))
                        {
                        if (srv.response.success)
                            {
                                ROS_INFO("Service Request Success : action_msgs : %s", action_msgs.c_str());
                            }
                        else
                            {
                                //ROS_WARN("Service Request ");
                            }
                        }
                    }
                else if (com == "rot_off")
                    {
                    srv.request.command = com;
                    // srv.request.turning_time = 1;  
                    if (client.call(srv))
                        {
                         if (srv.response.success)
                            {
                                ROS_INFO("Service Request Success : action_msgs : %s", action_msgs.c_str());
                            }
                      else
                            {
                                //ROS_WARN("Service Request ()");
                            }
                        }
                    }
                else
                   {
                        ROS_ERROR("Service Request (Wait)");
                        return false;
                   }                 
                }
                com.clear(); // action_msgs 수행후 초기화
            }
    return 0;
}
