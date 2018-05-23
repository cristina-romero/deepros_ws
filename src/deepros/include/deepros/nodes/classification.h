#ifndef DEEPROS_NODES_CLASSIFICATION_H_
#define DEEPROS_NODES_CLASSIFICATION_H_

#include "ros/ros.h"

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/Image.h>

#include "deepros/classifier/base.h"
#include "deepros_msgs/Prediction.h"
#include "deepros_srvs/GetImagePredictions.h"

namespace deepros
{
  namespace nodes
  {
    /** \brief Class prediction node (framework independent)
     */
    class Classification
    {
      public:
        typedef boost::shared_ptr<Classification> Ptr;
        
        /** \brief Constructor */
        Classification (ros::NodeHandle nh, deepros::classifier::Base::Ptr &classifier, int n = 5, bool sort = true);
        
        /** \brief Destructor */
        ~Classification () { };
        
      protected:
        /* Classifier object */
        deepros::classifier::Base::Ptr classifier_;
        
        /* Number of predictions */
        int n_predictions_;
        
        /* Sort predictions by probability */
        bool sort_;
        
        /* Predictions publisher */
        ros::Publisher prediction_pub_;
        
        /* Image subscriber */
        ros::Subscriber image_sub_;
        
        /* Prediction service */
        ros::ServiceServer prediction_srv_;
        
        /* ROS node handle */
        ros::NodeHandle nh_;
        
        /** \brief Subscriber connected to prediction topic */
        void
        connectCallback ();
        
        /** \brief Subscriber disconnected from prediction topic */
        void
        disconnectCallback ();
        
        /** \brief RGB images stream callback */
        void
        streamCallback (const sensor_msgs::ImageConstPtr& input);
        
        /** \brief Service callback */
        bool
        serviceCallback (deepros_srvs::GetImagePredictions::Request &req, deepros_srvs::GetImagePredictions::Response &res);
        
        /** \brief Process RGB image to obtain array of predictions */
        std::vector<deepros_msgs::Prediction>
        processImage (const sensor_msgs::ImageConstPtr& input, int n, bool sort);
    };
  }
}

#endif // DEEPROS_NODES_CLASSIFICATION_H_
