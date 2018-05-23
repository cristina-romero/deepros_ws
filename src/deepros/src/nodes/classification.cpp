#include "deepros/nodes/classification.h"
#include "deepros_msgs/Predictions.h"

///////////////////////////////////////////////////////////////////
deepros::nodes::Classification::Classification(ros::NodeHandle nh, deepros::classifier::Base::Ptr &classifier, int n, bool sort) :
  nh_ (nh),
  classifier_ (classifier),
  n_predictions_ (n),
  sort_ (sort)
{
  // Advertise topic for predictions stream
  prediction_pub_ = nh_.advertise<deepros_msgs::Predictions> ("/deepros/predictions", 10, 
    boost::bind(&deepros::nodes::Classification::connectCallback, this),
    boost::bind(&deepros::nodes::Classification::disconnectCallback, this));
    
  // Advertise service
  prediction_srv_ = nh_.advertiseService ("/deepros/srv/predict", 
    (boost::function <bool (deepros_srvs::GetImagePredictions::Request&, deepros_srvs::GetImagePredictions::Response&)>)
    boost::bind(&deepros::nodes::Classification::serviceCallback, this, _1, _2));
}

///////////////////////////////////////////////////////////////////
void
deepros::nodes::Classification::connectCallback ()
{
  if (prediction_pub_.getNumSubscribers() == 1 && !((void *) image_sub_))
  {
    // Someone is listening -> Subscribe to camera
    image_sub_ = nh_.subscribe<sensor_msgs::Image> ("input", 1, 
//      (boost::function <void (const sensor_msgs::ImageConstPtr&)>)
      boost::bind(&deepros::nodes::Classification::streamCallback, this, _1));
    ROS_INFO ("Starting predictions stream.");
  }
}

///////////////////////////////////////////////////////////////////
void
deepros::nodes::Classification::disconnectCallback ()
{
  if (prediction_pub_.getNumSubscribers() == 0 && ((void *) image_sub_))
  {
    // No one is listening -> Unsubscribe to camera
    image_sub_.shutdown ();
    ROS_INFO ("Stopping predictions stream.");
  }
}

///////////////////////////////////////////////////////////////////
void
deepros::nodes::Classification::streamCallback (const sensor_msgs::ImageConstPtr& input)
{
  // Prepare message
  deepros_msgs::Predictions msg;
  msg.header = input->header;
  
  // Get image predictions
  msg.predictions = processImage (input, n_predictions_, sort_);
  
  // Publish
  prediction_pub_.publish (msg);
}

///////////////////////////////////////////////////////////////////
bool 
deepros::nodes::Classification::serviceCallback (deepros_srvs::GetImagePredictions::Request &req,
  deepros_srvs::GetImagePredictions::Response &res)
{
  // Get image predictions
  sensor_msgs::ImageConstPtr image_ptr = boost::make_shared<sensor_msgs::Image> (req.image);
  res.predictions = processImage (image_ptr, req.n, req.sort);
  
  return res.predictions.size () == req.n;
}

///////////////////////////////////////////////////////////////////
std::vector<deepros_msgs::Prediction>
deepros::nodes::Classification::processImage (const sensor_msgs::ImageConstPtr& input, int n, bool sort)
{
  // convert to cv image
  cv_bridge::CvImagePtr img = cv_bridge::toCvCopy(input, sensor_msgs::image_encodings::BGR8);
  
  // Predict
  std::vector<deepros::classifier::Prediction> predictions = classifier_->classify (img->image, n, sort);
  
  // Store predictions
  std::vector<deepros_msgs::Prediction> pred;
  pred.reserve (predictions.size ());
  for (std::vector<deepros::classifier::Prediction>::iterator it = predictions.begin (); it != predictions.end (); ++it)
  {
    deepros_msgs::Prediction prediction_msg;
    prediction_msg.label = (*it).label;
    prediction_msg.confidence = (*it).confidence;
    pred.push_back (prediction_msg);
  }
  
  return pred;
}
