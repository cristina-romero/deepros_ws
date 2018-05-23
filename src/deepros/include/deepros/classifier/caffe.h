#ifndef DEEPROS_CLASSIFIER_CAFFE_H_
#define DEEPROS_CLASSIFIER_CAFFE_H_

// #ifndef CPU_ONLY
// #define CPU_ONLY 1
// #endif

#include <caffe/caffe.hpp>

#include "deepros/classifier/base.h"

namespace deepros
{
  namespace classifier
  {
    class Caffe : public Base
    {
      public:
        typedef boost::shared_ptr<Caffe> Ptr;
        
        /** \brief Constructor */
        Caffe (const std::string &model_file, const std::string &trained_file, const std::string &mean_file, const std::string &label_file);
        
        /** \brief Empty destructor */
        inline
        ~Caffe () { };
        
      protected:
        using Base::labels_;
      
        /* Trained net */
        boost::shared_ptr<caffe::Net<float> > net_;
        
        /* Input image size for the net */
        cv::Size input_geometry_;
        
        /* Number of channels */
        int num_channels_;
        
        /* Mean image */
        cv::Mat mean_;
        
        /** Return the top N predictions.
         * @param img Input image
         * @param n Return top n predictions
         * @return Vector of top n predictions (confidence + label)
         */
        virtual std::vector<Prediction> 
        process (const cv::Mat &img, int n = 5, bool sort = true);
        
        /** Return the top N predictions for a batch of images.
         * @param imgs Input images
         * @param n Return top n predictions for each image
         * @return Vector with n predictions (confidence + label) for each image
         */
        virtual std::vector<std::vector<Prediction> > 
        batchProcess (const std::vector<cv::Mat> &imgs, int n = 5, bool sort = true);
        
        /** Load the mean file in binaryproto format.
         * @param mean_file Mean file in binaryproto format
         */
        void 
        setMean (const std::string &mean_file);
        
        std::vector<float> 
        predict (const cv::Mat &img);
        
        std::vector<float>
        batchPredict (const std::vector<cv::Mat> &imgs);
        
        /** Wrap the input layer of the network in separate cv::Mat objects
         * (one per channel). This way we save one memcpy operation and we
         * don't need to rely on cudaMemcpy2D. The last preprocessing
         * operation will write the separate channels directly to the input
         * layer.
         */
        void
        wrapInputLayer (std::vector<cv::Mat> *input_channels);
        
        void
        batchWrapInputLayer (std::vector<std::vector<cv::Mat> > &input_batch);
        
        void
        preprocess (const cv::Mat &img, std::vector<cv::Mat> *input_channels);
        
        void
        batchPreprocess (const std::vector<cv::Mat> &imgs, std::vector<std::vector<cv::Mat> > &input_batch);
        
        /* Return the indices of the top N values of vector v. */
        static std::vector<int> 
        argmax (const std::vector<float>& v, int N);
        
        inline static bool
        pairCompare (const std::pair<float, int>& lhs,
                     const std::pair<float, int>& rhs)
        {
          return lhs.first > rhs.first;
        };
    };
  }
}

#endif  // DEEPROS_CLASSIFIER_CAFFE_H_
