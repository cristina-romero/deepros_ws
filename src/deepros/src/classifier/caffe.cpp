#include <opencv2/imgproc/imgproc.hpp>

#include "deepros/classifier/caffe.h"

//////////////////////////////////////////////////////////////////////////////////////////////
deepros::classifier::Caffe::Caffe (const std::string& model_file, const std::string& trained_file, const std::string& mean_file, const std::string& label_file)
{
#ifdef CPU_ONLY
  caffe::Caffe::set_mode (caffe::Caffe::CPU);
#else
  caffe::Caffe::set_mode (caffe::Caffe::GPU);
#endif

  /* Load the network. */
  net_.reset (new caffe::Net<float> (model_file, caffe::TEST));
  net_->CopyTrainedLayersFrom (trained_file);

  CHECK_EQ (net_->num_inputs (), 1) << "Network should have exactly one input.";
  CHECK_EQ (net_->num_outputs (), 1) << "Network should have exactly one output.";

  caffe::Blob<float>* input_layer = net_->input_blobs ()[0];
  num_channels_ = input_layer->channels ();
  CHECK (num_channels_ == 3 || num_channels_ == 1)
    << "Input layer should have 1 or 3 channels.";
  input_geometry_ = cv::Size (input_layer->width (), input_layer->height ());

  /* Load the binaryproto mean file. */
  setMean (mean_file);

  /* Load labels. */
  std::ifstream labels (label_file.c_str());
  CHECK (labels) << "Unable to open labels file " << label_file;
  std::string line;
  while (std::getline (labels, line))
    labels_.push_back (line);

  caffe::Blob<float>* output_layer = net_->output_blobs ()[0];
  CHECK_EQ (labels_.size (), output_layer->channels ()) << "Number of labels is different from the output layer dimension.";
}

//////////////////////////////////////////////////////////////////////////////////////////////
std::vector<deepros::classifier::Prediction> 
deepros::classifier::Caffe::process (const cv::Mat &img, int n, bool sort)
{
  std::vector<float> output = predict (img);

  n = std::min<int> (labels_.size(), n);
  std::vector<int> max_n;
  if (sort)
    max_n = argmax (output, n);
  std::vector<deepros::classifier::Prediction> predictions;
  for (int i = 0; i < n; ++i)
  {
    int idx = (sort) ? max_n[i] : i;
    deepros::classifier::Prediction p (labels_[idx], output[idx]);
    predictions.push_back (p);
  }

  return predictions;
}

//////////////////////////////////////////////////////////////////////////////////////////////
std::vector<std::vector<deepros::classifier::Prediction> > 
deepros::classifier::Caffe::batchProcess (const std::vector<cv::Mat> &imgs, int n, bool sort)
{
  std::vector<float> output = batchPredict (imgs);
  
  n = std::min<int> (labels_.size(), n);
  int num_classes = net_->output_blobs ()[0]->channels ();
  std::vector<std::vector<deepros::classifier::Prediction> > predictions;
  for (int j = 0; j < imgs.size (); j++)
  {
    std::vector<float> output_img (output.begin () + j * num_classes, output.begin () + (j + 1) * num_classes);
    std::vector<int> max_n;
    if (sort)
      max_n = argmax (output_img, n);
    std::vector<deepros::classifier::Prediction> predictions_img;
    for (int i = 0; i < n; ++i)
    {
      int idx = (sort) ? max_n[i] : i;
      deepros::classifier::Prediction p (labels_[idx], output_img[idx]);
      predictions_img.push_back (p);
    }
    predictions.push_back (predictions_img);
  }
  
  return predictions;
}

//////////////////////////////////////////////////////////////////////////////////////////////
void 
deepros::classifier::Caffe::setMean (const std::string &mean_file)
{
  caffe::BlobProto blob_proto;
  ReadProtoFromBinaryFileOrDie (mean_file.c_str(), &blob_proto);

  /* Convert from BlobProto to Blob<float> */
  caffe::Blob<float> mean_blob;
  mean_blob.FromProto (blob_proto);
  CHECK_EQ (mean_blob.channels (), num_channels_)
    << "Number of channels of mean file doesn't match input layer.";

  /* The format of the mean file is planar 32-bit float BGR or grayscale. */
  std::vector<cv::Mat> channels;
  float* data = mean_blob.mutable_cpu_data ();
  for (int i = 0; i < num_channels_; ++i) {
    /* Extract an individual channel. */
    cv::Mat channel (mean_blob.height (), mean_blob.width (), CV_32FC1, data);
    channels.push_back (channel);
    data += mean_blob.height () * mean_blob.width ();
  }

  /* Merge the separate channels into a single image. */
  cv::Mat mean;
  cv::merge (channels, mean);

  /* Compute the global mean pixel value and create a mean image
   * filled with this value. */
  cv::Scalar channel_mean = cv::mean (mean);
  mean_ = cv::Mat (input_geometry_, mean.type (), channel_mean);
}

//////////////////////////////////////////////////////////////////////////////////////////////
std::vector<float> 
deepros::classifier::Caffe::predict (const cv::Mat &img)
{
  caffe::Blob<float>* input_layer = net_->input_blobs ()[0];
  input_layer->Reshape (1, num_channels_, input_geometry_.height, input_geometry_.width);
  
  /* Forward dimension change to all layers. */
  net_->Reshape ();

  std::vector<cv::Mat> input_channels;
  wrapInputLayer (&input_channels);

  preprocess (img, &input_channels);

  net_->ForwardPrefilled ();

  /* Copy the output layer to a std::vector */
  caffe::Blob<float>* output_layer = net_->output_blobs ()[0];
  const float* begin = output_layer->cpu_data ();
  const float* end = begin + output_layer->channels ();
  return std::vector<float> (begin, end);
}

//////////////////////////////////////////////////////////////////////////////////////////////
std::vector<float>
deepros::classifier::Caffe::batchPredict (const std::vector<cv::Mat> &imgs)
{
  caffe::Blob<float>* input_layer = net_->input_blobs ()[0];
  
  input_layer->Reshape (imgs.size (), num_channels_, input_geometry_.height, input_geometry_.width);
  
  /* Forward dimension change to all layers. */
  net_->Reshape ();

  std::vector<std::vector<cv::Mat> > input_batch;
  batchWrapInputLayer (input_batch);

  batchPreprocess (imgs, input_batch);

  net_->ForwardPrefilled ();
  
  /* Copy the output layer to a std::vector */
  caffe::Blob<float>* output_layer = net_->output_blobs ()[0];
  const float* begin = output_layer->cpu_data ();
  const float* end = begin + output_layer->channels () * imgs.size ();
  return std::vector<float> (begin, end);
}

//////////////////////////////////////////////////////////////////////////////////////////////
void 
deepros::classifier::Caffe::wrapInputLayer (std::vector<cv::Mat> *input_channels)
{
  caffe::Blob<float>* input_layer = net_->input_blobs ()[0];

  int width = input_layer->width ();
  int height = input_layer->height ();
  float* input_data = input_layer->mutable_cpu_data ();
  for (int i = 0; i < input_layer->channels (); ++i)
  {
    cv::Mat channel (height, width, CV_32FC1, input_data);
    input_channels->push_back (channel);
    input_data += width * height;
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
void 
deepros::classifier::Caffe::batchWrapInputLayer (std::vector<std::vector<cv::Mat> > &input_batch)
{
  caffe::Blob<float>* input_layer = net_->input_blobs ()[0];

  int width = input_layer->width ();
  int height = input_layer->height ();
  int num = input_layer->num ();
  float* input_data = input_layer->mutable_cpu_data ();
  for (int j = 0; j < num; j++)
  {
    std::vector<cv::Mat> input_channels;
    for (int i = 0; i < input_layer->channels (); ++i)
    {
      cv::Mat channel (height, width, CV_32FC1, input_data);
      input_channels.push_back (channel);
      input_data += width * height;
    }
    input_batch.push_back (input_channels);
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
void 
deepros::classifier::Caffe::preprocess (const cv::Mat& img, std::vector<cv::Mat> *input_channels)
{
  /* Convert the input image to the input image format of the network. */
  cv::Mat sample;
  if (img.channels () == 3 && num_channels_ == 1)
    cv::cvtColor (img, sample, cv::COLOR_BGR2GRAY);
  else if (img.channels () == 4 && num_channels_ == 1)
    cv::cvtColor (img, sample, cv::COLOR_BGRA2GRAY);
  else if (img.channels () == 4 && num_channels_ == 3)
    cv::cvtColor(img, sample, cv::COLOR_BGRA2BGR);
  else if (img.channels () == 1 && num_channels_ == 3)
    cv::cvtColor (img, sample, cv::COLOR_GRAY2BGR);
  else
    sample = img;

  cv::Mat sample_resized;
  if (sample.size () != input_geometry_)
    cv::resize (sample, sample_resized, input_geometry_);
  else
    sample_resized = sample;

  cv::Mat sample_float;
  if (num_channels_ == 3)
    sample_resized.convertTo (sample_float, CV_32FC3);
  else
    sample_resized.convertTo (sample_float, CV_32FC1);

  cv::Mat sample_normalized;
  cv::subtract (sample_float, mean_, sample_normalized);

  /* This operation will write the separate BGR planes directly to the
   * input layer of the network because it is wrapped by the cv::Mat
   * objects in input_channels. */
  cv::split (sample_normalized, *input_channels);

  CHECK (reinterpret_cast<float*> (input_channels->at (0).data) == net_->input_blobs ()[0]->cpu_data ())
    << "Input channels are not wrapping the input layer of the network.";
}

//////////////////////////////////////////////////////////////////////////////////////////////
void 
deepros::classifier::Caffe::batchPreprocess (const std::vector<cv::Mat> &imgs, std::vector<std::vector<cv::Mat> > &input_batch)
{
  for (int i = 0; i < imgs.size (); i++)
  {
    cv::Mat img = imgs[i];
//    std::vector<cv::Mat> *input_channels = &((*input_batch)[i]);
    
    // Convert the input image to the input image format of the network.
    cv::Mat sample;
    if (img.channels () == 3 && num_channels_ == 1)
      cv::cvtColor (img, sample, cv::COLOR_BGR2GRAY);
    else if (img.channels () == 4 && num_channels_ == 1)
      cv::cvtColor (img, sample, cv::COLOR_BGRA2GRAY);
    else if (img.channels () == 4 && num_channels_ == 3)
      cv::cvtColor(img, sample, cv::COLOR_BGRA2BGR);
    else if (img.channels () == 1 && num_channels_ == 3)
      cv::cvtColor (img, sample, cv::COLOR_GRAY2BGR);
    else
      sample = img;

    cv::Mat sample_resized;
    if (sample.size () != input_geometry_)
      cv::resize (sample, sample_resized, input_geometry_);
    else
      sample_resized = sample;

    cv::Mat sample_float;
    if (num_channels_ == 3)
      sample_resized.convertTo (sample_float, CV_32FC3);
    else
      sample_resized.convertTo (sample_float, CV_32FC1);

    cv::Mat sample_normalized;
    cv::subtract (sample_float, mean_, sample_normalized);

    // This operation will write the separate BGR planes directly to the
    // input layer of the network because it is wrapped by the cv::Mat
    // objects in input_channels.
//    cv::split (sample_normalized, *input_channels);
    cv::split (sample_normalized, input_batch[i]);
  }
  CHECK (reinterpret_cast<float*> (input_batch.at (0).at (0).data) == net_->input_blobs ()[0]->cpu_data ()) << "Input channels are not wrapping the input layer of the network.";
}

//////////////////////////////////////////////////////////////////////////////////////////////
std::vector<int> 
deepros::classifier::Caffe::argmax(const std::vector<float>& v, int N)
{
  std::vector<std::pair<float, int> > pairs;
  for (size_t i = 0; i < v.size(); ++i)
    pairs.push_back(std::make_pair(v[i], i));
  std::partial_sort(pairs.begin(), pairs.begin() + N, pairs.end(), pairCompare);

  std::vector<int> result;
  for (int i = 0; i < N; ++i)
    result.push_back(pairs[i].second);
  return result;
}
