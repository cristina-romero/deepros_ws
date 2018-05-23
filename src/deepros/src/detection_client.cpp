#include "ros/ros.h"

#include <cv_bridge/cv_bridge.h>

#include <opencv2/opencv.hpp>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

#include "deepros_srvs/GetImageDetections.h"

int main(int argc, char** argv)
{
  // ROS init
  ros::init(argc, argv, "detection_client");
  ros::NodeHandle nh;
  
  // Check arguments
  if (argc < 2) {
    std::cerr << "Usage: " << argv[0] << " <image> [<args>]" << std::endl;
    return 1;
  }
  
  // Load image
  std::string image_file = argv[1];
  cv::Mat img = cv::imread(image_file, -1);
  if (img.empty()) 
  {
    ROS_ERROR("Unable to decode image %s", image_file.c_str ());
    return 1;
  }
  
  // Client
  ros::ServiceClient client = nh.serviceClient<deepros_srvs::GetImageDetections> ("/deepros/srv/detect");
  
  // Convert cv::Mat to sensor_msg::Image
  cv_bridge::CvImage ros_image;
  ros_image.header.stamp = ros::Time::now();
  ros_image.encoding = sensor_msgs::image_encodings::BGR8;
  ros_image.image = img;
  
  // Call service
  deepros_srvs::GetImageDetections srv;
  srv.request.image = *ros_image.toImageMsg();
  if (client.call (srv))
  {
    ROS_INFO ("Detections for image: %s", image_file.c_str ());
    for (int i = 0; i < srv.response.detections.size (); i++)
    {
      ROS_INFO ("%d: %f - %s (%d, %d - %d, %d)", i, srv.response.detections[i].confidence, srv.response.detections[i].label.c_str (),
        srv.response.detections[i].min_x, srv.response.detections[i].min_y, srv.response.detections[i].max_x, srv.response.detections[i].max_y);
    }
  }
  else
  {
    ROS_ERROR("Failed to call predict service.");
    return 1;
  }
  
  return 0;
}
