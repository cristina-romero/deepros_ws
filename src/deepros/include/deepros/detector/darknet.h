#ifndef DEEPROS_DETECTOR_DARKNET_H_
#define DEEPROS_DETECTOR_DARKNET_H_

// Darknet
extern "C" {
#include <image.h>
#include <network.h>
}

#include "deepros/detector/base.h"

namespace deepros
{
  namespace detector
  {
    /** \brief Darknet classifiers. 
     */
    class Darknet : public Base
    {
      public:
        typedef boost::shared_ptr<Darknet> Ptr;
        
        /** \brief Constructor */
        Darknet (const std::string &cfg_file, const std::string &weigths_file, const std::string &labels_file);
        
        /** \brief Destructor */
        ~Darknet ();
      
        /** Return the detected elements in the image.
         * @param img Input image
         * @return Vector detections (confidence + label + bounding box)
         */
        virtual std::vector<Detection> 
        detect (const cv::Mat &img);
      
      protected:
        using Base::labels_;
        
        /** \brief Network structure and weights */
        boost::shared_ptr<network> net_;
        
        /** Confidence threshold */
        float threshold_;
        
        /** \brief Convert OpenCV image to darknet image struct
         * @param img OpenCV image
         * @return Darknet image
         */
        image
        fromOpenCVtoDarknet (cv::Mat img);
    };
  }
}

#endif // DEEPROS_DETECTOR_DARKNET_H_
