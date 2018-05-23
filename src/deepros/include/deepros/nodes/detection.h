#ifndef DEEPROS_NODES_DETECTION_H_
#define DEEPROS_NODES_DETECTION_H_

#include "ros/ros.h"

#include <opencv2/opencv.hpp>

#include "deepros/detector/base.h"
#include "deepros_msgs/Detection.h"
#include "deepros_srvs/GetImageDetections.h"

namespace deepros
{
  namespace nodes
  {
    /** \brief Detection node (framework independent)
     */
    class Detection
    {
      public:
        typedef boost::shared_ptr<Detection> Ptr;
        
        /** \brief Constructor */
        Detection (ros::NodeHandle nh, deepros::detector::Base::Ptr &detector);
        
        /** \brief Destructor */
        ~Detection () { };
        
      protected:
        /* Detector object */
        deepros::detector::Base::Ptr detector_;
        
        /* Detections publisher */
        ros::Publisher detection_pub_;
        
        /* Image subscriber */
        ros::Subscriber image_sub_;
        
        /* Detection service */
        ros::ServiceServer detection_srv_;
        
        /* ROS node handle */
        ros::NodeHandle nh_;
        
        /** \brief Subscriber connected to detection topic */
        void
        connectCallback ();
        
        /** \brief Subscriber disconnected from detection topic */
        void
        disconnectCallback ();
        
        /** \brief RGB images stream callback */
        void
        streamCallback (const sensor_msgs::ImageConstPtr& input);
        
        /** \brief Service callback */
        bool
        serviceCallback (deepros_srvs::GetImageDetections::Request &req, deepros_srvs::GetImageDetections::Response &res);
        
        /** \brief Process RGB image to obtain array of detections */
        std::vector<deepros_msgs::Detection>
        processImage (const sensor_msgs::ImageConstPtr& input);
    };
  }
}

#endif // DEEPROS_NODES_DETECTION_H_
