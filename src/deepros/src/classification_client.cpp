#include "ros/ros.h"

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

#include "deepros_srvs/GetImagePredictions.h"

int main(int argc, char** argv)
{
  // ROS init
  ros::init(argc, argv, "classification_client");
  ros::NodeHandle nh;
  
  // Check arguments
  if (argc < 3) {
    std::cerr << "Usage: " << argv[0] << " <image> <n_predictions> [<args>]" << std::endl;
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
  
  // Number of predictions
  int n_predictions = atoi (argv[2]);
  
  // Client
  ros::ServiceClient client = nh.serviceClient<deepros_srvs::GetImagePredictions> ("/deepros/srv/predict");
  
  // Convert cv::Mat to sensor_msg::Image
  cv_bridge::CvImage ros_image;
  ros_image.header.stamp = ros::Time::now();
  ros_image.encoding = sensor_msgs::image_encodings::BGR8;
  ros_image.image = img;
  
  // Call service
  deepros_srvs::GetImagePredictions srv;
  srv.request.image = *ros_image.toImageMsg();
  srv.request.n = n_predictions;
  srv.request.sort = true;
  if (client.call (srv))
  {
    ROS_INFO ("Predictions for image: %s", image_file.c_str ());
    for (int i = 0; i < srv.response.predictions.size (); i++)
      ROS_INFO ("%d: %f - %s", i, srv.response.predictions[i].confidence, srv.response.predictions[i].label.c_str ());
  }
  else
  {
    ROS_ERROR("Failed to call predict service.");
    return 1;
  }
  
  return 0;
}
