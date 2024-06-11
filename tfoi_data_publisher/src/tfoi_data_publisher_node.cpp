#include "tfoi_data_publisher.h"
#include <cmath>

int main(int argc, char**argv)
{
    ros::init(argc, argv, "tfoi_data_publisher_node");
    ros::NodeHandle nh;

    DataPublisher data_publisher;    
    ros::Rate loop_rate(1);
    while(ros::ok())
    {    
     loop_rate.sleep();      
     ros::spinOnce();        
    }
  
    //return 0;
}
