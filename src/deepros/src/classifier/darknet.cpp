// Darknet definitions
#ifdef darknet_GPU // For compatibility with Caffe

#ifndef GPU
#define GPU
#endif

// C++ linkage
#include "cuda_runtime.h"
#include "curand.h"
#include "cublas_v2.h"

#ifdef darknet_CUDNN
#ifndef CUDNN
#define CUDNN
#endif
#include "cudnn.h"
#endif

#endif

#ifdef darknet_OPENCV
#ifndef OPENCV
#define OPENCV
#endif
#endif

// Darknet
extern "C" {
#include <data.h>
}

// OpenCV
#include <opencv2/core/core.hpp>

#include "deepros/classifier/darknet.h"

//////////////////////////////////////////////////////////////////////////////////////////////
deepros::classifier::Darknet::Darknet (const std::string &cfg_file, const std::string &weights_file, const std::string &labels_file, int batch_size)
{
  batch_size_ = batch_size;

  // Convert std::string to char*
  char *labels_str = new char[labels_file.length() + 1];
  strcpy(labels_str, labels_file.c_str());
  char *cfg_str = new char[cfg_file.length() + 1];
  strcpy(cfg_str, cfg_file.c_str());
  char *weights_str = new char[weights_file.length() + 1];
  strcpy(weights_str, weights_file.c_str());
  
  // Load labels
  char **names = get_labels(labels_str);
  
  size_t n_labels;
  for (n_labels = 0; names[n_labels] != NULL; n_labels++);
  labels_ = std::vector<std::string> (names, names + n_labels);
  
  // Load net config and weights
  network *net = load_network (cfg_str, weights_str, 0);
  set_batch_network(net, batch_size_);
  net_ = boost::shared_ptr<network> (net);
  
  // Save network width for resize
  net_width_ = net_->w;
  
  // Clean up
  delete[] weights_str;
  delete[] cfg_str;
  delete[] labels_str;
}

//////////////////////////////////////////////////////////////////////////////////////////////
deepros::classifier::Darknet::~Darknet ()
{
  free_network (net_.get ());
}

//////////////////////////////////////////////////////////////////////////////////////////////
//deepros::classifier::Darknet::darknet_network
//deepros::classifier::Darknet::loadDarknetNetwork (char *cfgfile, char *weightsfile, int batch_size)
//{
//  darknet_network dn;
//  dn.net = load_network (cfgfile, weightsfile, 0);
//  set_batch_network(&dn.net, batch_size);
//  return dn;
//}

//////////////////////////////////////////////////////////////////////////////////////////////
image
deepros::classifier::Darknet::fromOpenCVtoDarknet (cv::Mat img)
{
  // Copy cv::Mat data to darknet image (adapted from load_image_stb function in image.c file)
  cv::Size s = img.size ();
  int w = s.width;
  int h = s.height;
  int c = img.channels ();  
  image im = make_image (w, h, c);
  for(int k = 0; k < c; ++k){
    for(int j = 0; j < h; ++j){
      for(int i = 0; i < w; ++i){
        int dst_index = i + w * j + w * h * k;
        int src_index = k + c * i + c * w * j;
        im.data[dst_index] = (float) img.data[src_index]/255.;
      }
    }
  }
  rgbgr_image(im);
  
  return im;
}

//////////////////////////////////////////////////////////////////////////////////////////////
std::vector<deepros::classifier::Prediction> 
deepros::classifier::Darknet::process (const cv::Mat &img, int n, bool sort)
{
  // Code adapted from predict_classifier in classifier.c file

  // Preprocess image
  image im = fromOpenCVtoDarknet (img);
  image r = letterbox_image(im, net_->w, net_->h);
  
  // Predict
  float *predictions = network_predict (net_.get (), r.data);
  if(net_->hierarchy)
    hierarchy_predictions (predictions, net_->outputs, net_->hierarchy, 1, 1);
  
  // Prepare output
  std::vector<int> max_n (n);
  if (sort)
    top_predictions (net_.get (), n, max_n.data ());
  std::vector<deepros::classifier::Prediction> predictions_img;
  for(int i = 0; i < n; ++i)
  {
    int idx = (sort) ? max_n[i] : i;
    deepros::classifier::Prediction p (labels_[idx], predictions[idx]);
    predictions_img.push_back (p);
  }
  
  // Delete image
  free_image (r);
  free_image (im);

  return predictions_img;
}

//////////////////////////////////////////////////////////////////////////////////////////////
std::vector<std::vector<deepros::classifier::Prediction> > 
deepros::classifier::Darknet::batchProcess (const std::vector<cv::Mat> &imgs, int n, bool sort)
{
  // TODO: Batch processing
  std::vector<std::vector<deepros::classifier::Prediction> > out_predictions;
  for (int j = 0; j < imgs.size (); j++)
  {
    std::vector<deepros::classifier::Prediction> predictions_img = process (imgs[j], n, sort);
    out_predictions.push_back (predictions_img);
  } 
  
  return out_predictions;
}

