#include <ros/ros.h>
#include <ros/package.h>

#include <boost/filesystem.hpp>

#include <deepros_msgs/Labels.h>

#include "deepros/nodes/detection.h"
#include "deepros/detector/darknet.h"

int main(int argc, char** argv)
{
  // ROS init
  ros::init(argc, argv, "detector_darknet");
  ros::NodeHandle nh;
  
  // Check arguments
  if (argc < 2) {
    std::cerr << "Usage: " << argv[0] << " <model_id> [<args>]" << std::endl;
    return 1;
  }
  
  // Darknet model files
  std::string pkg_path = ros::package::getPath("deepros");
  ROS_INFO ("Load model %s from %s/models/", argv[1], pkg_path.c_str ());
  
  std::string model_id = argv[1];
  std::string model_file = pkg_path + "/models/darknet/" + model_id + "/" + model_id + ".cfg";
  std::string trained_file = pkg_path + "/models/darknet/" + model_id + "/" + model_id + ".weights";
  std::string label_file = pkg_path + "/models/darknet/" + model_id + "/" + model_id + ".labels";
  
  if (!boost::filesystem::exists (model_file) ||
      !boost::filesystem::exists (trained_file) ||
      !boost::filesystem::exists (label_file))
  {
    ROS_ERROR ("%s model not found. Please, check that the following files exist:", model_id.c_str ());
    ROS_ERROR (" - %s", model_file.c_str ());
    ROS_ERROR (" - %s", trained_file.c_str ());
    ROS_ERROR (" - %s", label_file.c_str ());
    ROS_ERROR ("You can also use the download_model.py script in '%s' to download darknet pretrained models.", (pkg_path + "/models/darknet").c_str ());
    return 1;
  }
  
  // Detector
  deepros::detector::Base::Ptr detector (new deepros::detector::Darknet (model_file, trained_file, label_file));
  
  // Detection node -> Advertise/subscribe to topics in constructor
  deepros::nodes::Detection dn (nh, detector);
  
  // Publish labels
  ros::Publisher labels_pub = nh.advertise<deepros_msgs::Labels> ("/deepros/labels", 1, true);
  deepros_msgs::Labels labels_msg;
  labels_msg.labels = detector->getLabels ();
  labels_pub.publish (labels_msg);
  
  // Return control to ROS
  ros::spin();
  
  return 0;
}
