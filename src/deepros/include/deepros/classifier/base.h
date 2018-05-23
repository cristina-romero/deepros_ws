#ifndef DEEPROS_CLASSIFIER_BASE_H_
#define DEEPROS_CLASSIFIER_BASE_H_

#include <opencv2/core/core.hpp>

#include <boost/shared_ptr.hpp>

namespace deepros
{
  namespace classifier
  {
    /* Pair (label, confidence) representing a prediction. */
    struct Prediction
    {
      std::string label;
      float confidence;
      
      Prediction (std::string l, float c) :
        label (l),
        confidence (c)
      { };
      
      friend std::ostream& operator << (std::ostream& os, const Prediction& p);
    };
    
    inline std::ostream&
    operator << (std::ostream& os, const Prediction & p)
    {
      os << p.label << ": " << p.confidence;
      return os;
    };
    
    /** \brief Base class for classifiers. 
     */
    class Base
    {
      public:
        typedef boost::shared_ptr<Base> Ptr;
        
        /** \brief Empty constructor */
        inline
        Base () { };
        
        /** \brief Empty destructor */
        inline
        ~Base () { };

        /** Return the top N predictions.
         * @param img Input image
         * @param n Return top n predictions
         * @return Vector of top n predictions (confidence + label)
         */
        virtual std::vector<Prediction> 
        classify (const cv::Mat &img, int n = 5, bool sort = true);
        
        /** Return the top N predictions for a batch of images.
         * @param imgs Input images
         * @param n Return top n predictions for each image
         * @return Vector with n predictions (confidence + label) for each image
         */
        virtual std::vector<std::vector<Prediction> > 
        batchClassify (const std::vector<cv::Mat> &imgs, int n = 5, bool sort = true);
        
        /** Return the labels vector
         @return Labels vector
         */
        inline std::vector<std::string>&
        getLabels () { return labels_; };
        
        /** Set the maximum batch size
         @param batch_size The batch size
         */
        inline void
        setBatchSize (int batch_size) { batch_size_ = batch_size; };
        
        /** Return the maximum batch size
         @return Batch size
         */
        inline int
        getBatchSize () { return batch_size_; };
      
      protected:
        /* Output labels */
        std::vector<std::string> labels_;
        
        /* Batch size */
        int batch_size_;
        
        /** Return the top N predictions for a batch of images.
         * @param imgs Input images
         * @param n Return top n predictions for each image
         * @return Vector with n predictions (confidence + label) for each image
         */
        virtual std::vector<std::vector<Prediction> > 
        batchProcess (const std::vector<cv::Mat> &imgs, int n = 5, bool sort = true) = 0;
    };
  }
}

#endif // DEEPROS_CLASSIFIER_BASE_H_
