#include "ros/ros.h"

#include <cv_bridge/cv_bridge.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>

#include "deepros_msgs/Detections.h"


ros::Publisher detections_pub_; // Detections publisher

void drawDetections (const sensor_msgs::ImageConstPtr& img_msg, const deepros_msgs::DetectionsConstPtr& det_msg)
{
  // Convert sensor_msg::Image to cv::Mat
  cv_bridge::CvImagePtr img = cv_bridge::toCvCopy (img_msg, sensor_msgs::image_encodings::BGR8);
  
  // Draw detection rectangles
  for (int i = 0; i < det_msg->detections.size (); i++)
  {
    cv::Point min_pt (det_msg->detections[i].min_x, det_msg->detections[i].min_y);
    cv::Point max_pt (det_msg->detections[i].max_x, det_msg->detections[i].max_y);
    cv::rectangle (img->image, min_pt, max_pt, cv::Scalar(0, 255, 0), 3);
  }
  
  // Publish
  detections_pub_.publish (img->toImageMsg());
}

int main (int argc, char** argv)
{
  // ROS init
  ros::init(argc, argv, "visualize_detections");
  ros::NodeHandle nh;
  
  // Advertise publisher
  detections_pub_ = nh.advertise<sensor_msgs::Image> ("/deepros/detections_visualization", 10);
  
  // Subscribe to synchronized topics
  message_filters::Subscriber<sensor_msgs::Image> image_sub (nh, "input", 1);
  message_filters::Subscriber<deepros_msgs::Detections> detections_sub (nh, "/deepros/detections", 1);
  
  typedef message_filters::sync_policies::ExactTime<sensor_msgs::Image, deepros_msgs::Detections> DetectionsPolicy;
  message_filters::Synchronizer<DetectionsPolicy> sync (DetectionsPolicy (10), image_sub, detections_sub);
  sync.registerCallback (boost::bind(&drawDetections, _1, _2));

  // Spin
  ros::spin ();
  
  return 0;
}
