#include <opencv2/imgproc/imgproc.hpp>

#include "deepros/classifier/base.h"

//////////////////////////////////////////////////////////////////////////////////////////////
std::vector<deepros::classifier::Prediction> 
deepros::classifier::Base::classify (const cv::Mat &img, int n, bool sort)
{
  std::vector<cv::Mat> batch_img;
  batch_img.push_back (img);
  std::vector<std::vector<deepros::classifier::Prediction> > batch_predictions = batchClassify (batch_img, n, sort);
  
  std::vector<deepros::classifier::Prediction> predictions;
  if (!batch_predictions.empty ())
    predictions = batch_predictions[0];
    
  return predictions;
}

//////////////////////////////////////////////////////////////////////////////////////////////
std::vector<std::vector<deepros::classifier::Prediction> > 
deepros::classifier::Base::batchClassify (const std::vector<cv::Mat> &imgs, int n, bool sort)
{
  std::vector<std::vector<deepros::classifier::Prediction> > predictions;
  predictions.reserve (imgs.size ());

  if (!batch_size_)
    batch_size_ = imgs.size ();

  int n_images_to_classify = imgs.size ();
  int n_images_classified = 0;
  while (n_images_to_classify != 0)
  {
    // Prepare batch
    int n_current_batch = std::min (n_images_to_classify, batch_size_);
    std::vector<cv::Mat> current_batch (imgs.begin () + n_images_classified, imgs.begin () + n_images_classified + n_current_batch);
//    std::cout << "Current batch size: " << current_batch.size () << std::endl;
    
    // Predict -> TODO: Change to use iterators
    std::vector<std::vector<deepros::classifier::Prediction> > batch_predictions = batchProcess (current_batch, n, sort);
    
    // Merge results
    predictions.insert (predictions.end (), batch_predictions.begin (), batch_predictions.end ());
    
    n_images_to_classify -= n_current_batch;
    n_images_classified += n_current_batch;
  }
  
  return predictions;
}
