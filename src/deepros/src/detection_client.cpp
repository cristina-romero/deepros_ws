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
  ros::NodeHandle nh("~");
  
  // Check arguments
  if (argc < 2) {
    std::cerr << "Usage: " << argv[0] << " <image> [_output:=<output_file>] [<args>]" << std::endl;
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
  
  // Check for output file
  std::string out_file;
  bool save_output = nh.getParam ("output", out_file);
  if (save_output)
    nh.deleteParam ("output"); // Clean param for next usage
  
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
      
      cv::Point min_pt (srv.response.detections[i].min_x, srv.response.detections[i].min_y);
      cv::Point max_pt (srv.response.detections[i].max_x, srv.response.detections[i].max_y);
      cv::rectangle (img, min_pt, max_pt, cv::Scalar(0, 255, 0), 3);
    }
    
    if (save_output)
    {
      cv::imwrite (out_file, img);
      ROS_INFO ("Detections image saved in: %s", out_file.c_str ());
    }
  }
  else
  {
    ROS_ERROR("Failed to call predict service.");
    return 1;
  }
  
  return 0;
}
