#ifndef DEEPROS_CLASSIFIER_DARKNET_H_
#define DEEPROS_CLASSIFIER_DARKNET_H_

// OpenCV
#include <opencv2/core/core.hpp>

// Darknet
extern "C" {
#include <image.h>
#include <network.h>
}

#include "deepros/classifier/base.h"

namespace deepros
{
  namespace classifier
  {
    /** \brief Darknet classifiers. 
     */
    class Darknet : public Base
    {
      public:
        typedef boost::shared_ptr<Darknet> Ptr;
        
        /** \brief Constructor */
        Darknet (const std::string &cfg_file, const std::string &weigths_file, const std::string &labels_file, int batch_size = 1);
        
        /** \brief Destructor */
        ~Darknet ();
      
      protected:
        using Base::batch_size_;
        using Base::labels_;
        
        /** \brief Network structure and weights */
        boost::shared_ptr<network> net_;
        
        /** \brief Network width */
        int net_width_;
        
        /** Return the top N predictions for a images.
         * @param img Input image
         * @param n Return top n predictions for each image
         * @return Vector with n predictions (confidence + label) for the image
         */
        virtual std::vector<Prediction> 
        process (const cv::Mat &img, int n, bool sort);
        
        /** Return the top N predictions for a batch of images.
         * @param imgs Input images
         * @param n Return top n predictions for each image
         * @return Vector with n predictions (confidence + label) for each image
         */
        virtual std::vector<std::vector<Prediction> > 
        batchProcess (const std::vector<cv::Mat> &imgs, int n = 5, bool sort = true);
        
        /** \brief C++ network struct */
//        struct darknet_network
//        {
//          network net;
//        };
        
        /** \brief Wrapper to avoid segmentation fault when loading the network. Don't know why it happens
         * @param cfgfile File name with network configuration
         * @param weightsfile File name with trained weights
         * @param batch_size Max batch size
         * @return Network struct
         */
//        darknet_network
//        loadDarknetNetwork (char *cfgfile, char *weightsfile, int batch_size = 1);
        
        /** \brief Convert OpenCV image to darknet image struct
         * @param img OpenCV image
         * @return Darknet image
         */
        image
        fromOpenCVtoDarknet (cv::Mat img);
    };
  }
}

#endif // DEEPROS_CLASSIFIER_DARKNET_H_
