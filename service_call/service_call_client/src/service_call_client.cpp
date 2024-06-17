#include "ros/ros.h"
#include "std_msgs/Int8.h"
#include "ros/callback_queue.h"
#include "service_call_client/RobotTurning.h"
#include "move_base_msgs/MoveBaseActionGoal.h"
#include <iostream>
#include <string>

std::string action_msgs;
std::string service_call_msgs;
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

void moveBaseGoalCallback(const move_base_msgs::MoveBaseActionGoal::ConstPtr& msg)
{
    // MoveBaseActionGoal 메시지로부터 header의 seq 값 가져오기
    uint32_t seq = msg->goal.target_pose.header.seq;
    // seq 값을 출력하여 확인
    ROS_INFO("Seq value: %u", seq);
    if((seq > 0) && (seq % 4 == 0))
    {
        //service_call_msgs = "rotation_call";
        seq += 1;
    }
    else
    {
        service_call_msgs = "";
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "service_call_client_node");
    ros::NodeHandle nh;

    // 서비스 클라이언트 객체 생성
    ros::ServiceClient client = nh.serviceClient<service_call_client::RobotTurning>("/turning_robot");

    service_call_client::RobotTurning srv;

    ROS_INFO("==== Service Client initialized ====");

    ros::Subscriber action_sub = nh.subscribe<std_msgs::Int8>("/action",10,actionCallback);
    ros::Subscriber goal_sub = nh.subscribe<move_base_msgs::MoveBaseActionGoal>("/move_base/goal", 10, moveBaseGoalCallback);


    ros::CallbackQueue queue;
    nh.setCallbackQueue(&queue);

    // Spin in a separate thread to handle callbacks
    ros::AsyncSpinner spinner(1, &queue);
    spinner.start();

    unsigned int turning_time;
    float angular_vel;
    std::string com_1, com_2;
    
    // 첫 번째 서비스 호출: 즉시 360도 회전
    // std::cout << "Enter the command : rot : 360_Rotation, DELAY_360, STOP_SERVICE" << std::endl;
    // std::cin >> com;
    // std::cin.clear(); std::cin.ignore(32767, '\n');
    while (ros::ok())
    {
      ros::spinOnce();
        com_1 = action_msgs;
        com_2 = service_call_msgs;
        if ((!com_1.empty() || !com_2.empty()))
                {
                if (com_1 == "rot_on")
                    {
                    srv.request.command = com_1;
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
                else if (com_1 == "rot_off")
                    {
                    srv.request.command = com_1;
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
                else if (com_2 == "rotation_call")
                    {
                    srv.request.command = com_2;
                    srv.request.turning_time = 14;  
                    if (client.call(srv))
                        {
                         if (srv.response.success)
                            {
                                ROS_INFO("Service Request Success : service_call_msgs : %s", service_call_msgs.c_str());
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
                com_1.clear();
                com_2.clear();
            }
    return 0;
}
