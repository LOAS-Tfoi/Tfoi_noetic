#include <ros/ros.h>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/CameraInfo.h>
#include <math.h>

using namespace std;
using namespace cv;

sensor_msgs::CameraInfo getCameraInfo(void){        // extract cameraInfo.
    sensor_msgs::CameraInfo cam;

    vector<double> D{0.090675, -0.171616, 0.029090, 0.000478, 0.000000};
    boost::array<double, 9> K = {
        567.895161, 0.000000, 336.564649, 0.000000, 574.029770, 299.759099, 0.000000, 0.000000, 1.000000  
    };
    
     boost::array<double, 12> P = {
        582.719116, 0.000000, 337.272246, 0.000000, 0.000000, 574.170898, 310.869070, 0.000000, 0.000000, 0.000000, 1.000000, 0.000000
    };
    boost::array<double, 9> r = {1.000000, 0.000000, 0.000000, 0.000000, 1.000000, 0.000000, 0.000000, 0.000000, 1.000000};

    cam.height = 480;
    cam.width = 640;
    cam.distortion_model = "plumb_bob";
    cam.D = D;
    cam.K = K;
    cam.P = P;
    cam.R = r;
    cam.binning_x = 0;
    cam.binning_y = 0;
    cam.header.frame_id = "camera";  //frame_id为camera，也就是相机名字
    cam.header.stamp = ros::Time::now();
    cam.header.stamp.nsec = 0;
    return cam;
}

int main(int argc, char** argv) {

  ros::init(argc, argv, "camera_info");  //初始化了一个节点，名字为camera_info
  ros::NodeHandle n;
  ros::Publisher pub = n.advertise<sensor_msgs::CameraInfo>("/camera/camera_info", 10);  
  sensor_msgs::CameraInfo camera_info_dyn;
  ros::Rate rate(1);  

  while (ros::ok())
  {
      camera_info_dyn = getCameraInfo();
      pub.publish(camera_info_dyn); 
      rate.sleep();
  }
    ros::spin();
    return 0;
}

