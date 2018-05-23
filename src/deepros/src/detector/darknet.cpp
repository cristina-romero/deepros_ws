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
#include "box.h"
#include "image.h"
#include "layer.h"
#include "region_layer.h"
#include "utils.h"
}

#include "deepros/detector/darknet.h"

//////////////////////////////////////////////////////////////////////////////////////////////
deepros::detector::Darknet::Darknet (const std::string &cfg_file, const std::string &weights_file, const std::string &labels_file) :
  threshold_ (0.5f)
{
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
  set_batch_network(net, 1);
  net_ = boost::shared_ptr<network> (net, free_network);
  
  // Clean up
  delete[] weights_str;
  delete[] cfg_str;
  delete[] labels_str;
}

//////////////////////////////////////////////////////////////////////////////////////////////
deepros::detector::Darknet::~Darknet ()
{
//  free_network (net_.get ());
}

//////////////////////////////////////////////////////////////////////////////////////////////
image
deepros::detector::Darknet::fromOpenCVtoDarknet (cv::Mat img)
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
std::vector<deepros::detector::Detection> 
deepros::detector::Darknet::detect (const cv::Mat &img)
{
  // Code adapted from test_yolo in yolo.c file

  std::vector<deepros::detector::Detection> detections;
  
  // Preprocess image
  image im = fromOpenCVtoDarknet (img);
  image sized = letterbox_image (im, net_->w, net_->h);
  
  // Some params
  float nms = 0.45f;
  int n_classes = labels_.size ();
  
  // Detection
  network_predict (net_.get (), sized.data);

  int n_boxes = 0;
  detection *dets = get_network_boxes(net_.get (), 1, 1, threshold_, 0, 0, 0, &n_boxes);
  if (nms)
    do_nms_sort(dets, n_boxes, n_classes, nms);
  
  // Prepare output
  for (int i = 0; i < n_boxes; i++)
  {
    // From draw_detections in image.c
    int class_id = max_index (dets[i].prob, n_classes);
    float prob = dets[i].prob[class_id];
    if (prob > threshold_)
    {
      // From draw_bbox in image.c
      int left = std::max (0.0f, (dets[i].bbox.x - dets[i].bbox.w / 2) * im.w);
      int right = std::min ((float) im.w, (dets[i].bbox.x + dets[i].bbox.w / 2) * im.w);
      int top = std::max (0.0f, (dets[i].bbox.y - dets[i].bbox.h / 2) * im.h);
      int bottom = std::min ((float) im.h, (dets[i].bbox.y + dets[i].bbox.h / 2) * im.h);
      
      deepros::detector::Detection d (labels_[class_id], prob, left, top, right, bottom);
      detections.push_back (d);
    }
  }
  
  // Clean up
  free_detections(dets, n_boxes);
  free_image (sized);
  free_image (im);
  
  return detections;
}

