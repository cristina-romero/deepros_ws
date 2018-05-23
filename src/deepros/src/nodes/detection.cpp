#include <cv_bridge/cv_bridge.h>

#include "deepros/nodes/detection.h"
#include "deepros_msgs/Detections.h"

///////////////////////////////////////////////////////////////////
deepros::nodes::Detection::Detection(ros::NodeHandle nh, deepros::detector::Base::Ptr &detector) :
  nh_ (nh),
  detector_ (detector)
{
  // Advertise topic for predictions stream
  detection_pub_ = nh_.advertise<deepros_msgs::Detections> ("/deepros/detections", 10, 
    boost::bind(&deepros::nodes::Detection::connectCallback, this),
    boost::bind(&deepros::nodes::Detection::disconnectCallback, this));
    
  // Advertise service
  detection_srv_ = nh_.advertiseService ("/deepros/srv/detect", 
    (boost::function <bool (deepros_srvs::GetImageDetections::Request&, deepros_srvs::GetImageDetections::Response&)>)
    boost::bind(&deepros::nodes::Detection::serviceCallback, this, _1, _2));
}

///////////////////////////////////////////////////////////////////
void
deepros::nodes::Detection::connectCallback ()
{
  if (detection_pub_.getNumSubscribers() == 1 && !((void *) image_sub_))
  {
    // Someone is listening -> Subscribe to camera
    image_sub_ = nh_.subscribe<sensor_msgs::Image> ("input", 1, 
//      (boost::function <void (const sensor_msgs::ImageConstPtr&)>)
      boost::bind(&deepros::nodes::Detection::streamCallback, this, _1));
    ROS_INFO ("Starting predictions stream.");
  }
}

///////////////////////////////////////////////////////////////////
void
deepros::nodes::Detection::disconnectCallback ()
{
  if (detection_pub_.getNumSubscribers() == 0 && ((void *) image_sub_))
  {
    // No one is listening -> Unsubscribe to camera
    image_sub_.shutdown ();
    ROS_INFO ("Stopping predictions stream.");
  }
}

///////////////////////////////////////////////////////////////////
void
deepros::nodes::Detection::streamCallback (const sensor_msgs::ImageConstPtr& input)
{
  // Prepare message
  deepros_msgs::Detections msg;
  msg.header = input->header;
  
  // Get image predictions
  msg.detections = processImage (input);
  
  // Publish
  detection_pub_.publish (msg);
}

///////////////////////////////////////////////////////////////////
bool 
deepros::nodes::Detection::serviceCallback (deepros_srvs::GetImageDetections::Request &req,
  deepros_srvs::GetImageDetections::Response &res)
{
  // Get image predictions
  sensor_msgs::ImageConstPtr image_ptr = boost::make_shared<sensor_msgs::Image> (req.image);
  res.detections = processImage (image_ptr);
  
  return res.detections.size ();
}

///////////////////////////////////////////////////////////////////
std::vector<deepros_msgs::Detection>
deepros::nodes::Detection::processImage (const sensor_msgs::ImageConstPtr& input)
{
  // convert to cv image
  cv_bridge::CvImagePtr img = cv_bridge::toCvCopy(input, sensor_msgs::image_encodings::BGR8);
  
  // Predict
  std::vector<deepros::detector::Detection> detections = detector_->detect (img->image);
  
  // Store predictions
  std::vector<deepros_msgs::Detection> det;
  det.reserve (detections.size ());
  for (std::vector<deepros::detector::Detection>::iterator it = detections.begin (); it != detections.end (); ++it)
  {
    deepros_msgs::Detection detection_msg;
    detection_msg.label = it->label;
    detection_msg.confidence = it->confidence;
    detection_msg.min_x = it->min_x;
    detection_msg.min_y = it->min_y;
    detection_msg.max_x = it->max_x;
    detection_msg.max_y = it->max_y;
    det.push_back (detection_msg);
  }
  
  return det;
}
