#ifndef DEEPROS_DETECTOR_BASE_H_
#define DEEPROS_DETECTOR_BASE_H_

#include <opencv2/core/core.hpp>

#include <boost/shared_ptr.hpp>

namespace deepros
{
  namespace detector
  {
    /* Prediction (label + confidence) with a bounding box. */
    struct Detection
    {
      std::string label;
      float confidence;
      
      int min_x, min_y;
      int max_x, max_y;
      
      Detection (std::string l, float c, int min_x, int min_y, int max_x, int max_y) :
        label (l),
        confidence (c),
        min_x (min_x),
        min_y (min_y),
        max_x (max_x),
        max_y (max_y)
      { };
      
      friend std::ostream& operator << (std::ostream& os, const Detection& d);
    };
    
    inline std::ostream&
    operator << (std::ostream& os, const Detection& d)
    {
      os << d.label << ": " << d.confidence << " (" << d.min_x << "," << d.min_y << " - " << d.max_x << "," << d.max_y << ")";
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

        /** Return the detected elements in the image.
         * @param img Input image
         * @return Vector detections (confidence + label + bounding box)
         */
        virtual std::vector<Detection> 
        detect (const cv::Mat &img) = 0;
        
        /** Return the labels vector
         @return Labels vector
         */
        inline std::vector<std::string>&
        getLabels () { return labels_; };
      
      protected:
        /* Possible output labels */
        std::vector<std::string> labels_;
    };
  }
}

#endif // DEEPROS_DETECTOR_BASE_H_
