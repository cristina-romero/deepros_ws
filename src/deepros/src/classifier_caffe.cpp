#include <ros/ros.h>
#include <ros/package.h>

#include "deepros/nodes/classification.h"
#include "deepros/classifier/caffe.h"

int main(int argc, char** argv)
{
  // ROS init
  ros::init(argc, argv, "classifier_caffe");
  ros::NodeHandle nh;
  
  // Check arguments
  if (argc < 3) {
    std::cerr << "Usage: " << argv[0] << " <model_id> <n_predictions> [<args>]" << std::endl;
    return 1;
  }
  
  // Caffe classifier files
  std::string pkg_path = ros::package::getPath("deepros");
  ROS_INFO ("Load model %s from %s/models/", argv[1], pkg_path.c_str ());
  
  std::string model_id = argv[1];
  std::string model_file = pkg_path + "/models/caffe/" + model_id + "/deploy.prototxt";
  std::string trained_file = pkg_path + "/models/caffe/" + model_id + "/" + model_id + ".caffemodel";
  std::string mean_file = pkg_path + "/models/caffe/" + model_id + "/mean.binaryproto";
  std::string label_file = pkg_path + "/models/caffe/" + model_id + "/synset_words.txt";
  
  if (!boost::filesystem::exists (model_file) ||
      !boost::filesystem::exists (trained_file) ||
      !boost::filesystem::exists (mean_file) ||
      !boost::filesystem::exists (label_file))
  {
    ROS_ERROR ("%s model not found. Please, check that the following files exist:", model_id.c_str ());
    ROS_ERROR (" - %s", model_file.c_str ());
    ROS_ERROR (" - %s", trained_file.c_str ());
    ROS_ERROR (" - %s", mean_file.c_str ());
    ROS_ERROR (" - %s", label_file.c_str ());
    ROS_ERROR ("You can also use the get_model.sh script in '%s' to download caffe pretrained models.", (pkg_path + "/models/").c_str ());
    return 1;
  }
  
  // Number of predictions
  int n_predictions = atoi (argv[2]);
  
  // Classifier
  deepros::classifier::Base::Ptr classifier (new deepros::classifier::Caffe (model_file, trained_file, mean_file, label_file));
  
  // Prediction node -> Advertise/subscribe to topics in constructor
  deepros::nodes::Classification pn (nh, classifier, n_predictions);
  
  // Return control to ROS
  ros::spin();
  
  return 0;
}
