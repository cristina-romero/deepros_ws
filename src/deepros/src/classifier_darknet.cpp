#include <ros/ros.h>
#include <ros/package.h>

#include <boost/filesystem.hpp>

#include "deepros/nodes/classification.h"
#include "deepros/classifier/darknet.h"

int main(int argc, char** argv)
{
  // ROS init
  ros::init(argc, argv, "classifier_darknet");
  ros::NodeHandle nh;
  
  // Check arguments
  if (argc < 3) {
    std::cerr << "Usage: " << argv[0] << " <model_id> <n_predictions> [<args>]" << std::endl;
    return 1;
  }
  
  // Darknet classifier files
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
  
  // Number of predictions
  int n_predictions = atoi (argv[2]);
  
  // Classifier
  deepros::classifier::Base::Ptr classifier (new deepros::classifier::Darknet (model_file, trained_file, label_file));
  
  // Prediction node -> Advertise/subscribe to topics in constructor
  deepros::nodes::Classification pn (nh, classifier, n_predictions);
  
  // Return control to ROS
  ros::spin();
  
  return 0;
}
