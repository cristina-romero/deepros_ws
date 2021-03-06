# deepros

Deep Learning + ROS. This ROS package integrates different Deep Learning frameworks, so pretrained models can be used for image classification and object detection.

The currently supported frameworks are:

* [Caffe](http://caffe.berkeleyvision.org/)
* [Darknet](https://pjreddie.com/darknet/)

## Prerequisites

This package is based on the use of pretrained models. Thus, you must have installed the corresponding framework and download the desired pretrained model(s). Please, follow these instructions to install and configure the framework:

* [Caffe: Install and configuration](models/README_Caffe.md)
* [Darknet: Install and configuration](models/README_Darknet.md)

**NOTE: Classification with Darknet has been temporarily disabled due to a bug in the framework.**

## Usage

The package provides a classifier and a detector node for each framework (if there are pretrained models for the task). Each of these nodes can work with a stream of images and/or with a single image.

### classifier\_&lt;framework&gt;

This node publishes the top `n` predictions for the input image received in the `input` topic. It also advertises a service `/deepros/srv/predictions` to obtain predictions for a single image.

For example, to obtain the top 5 predictions using the Caffe `bvlc_alexnet` model from a stream of images provided by the openni2\_launch package, you could use:

    rosrun deepros classifier_caffe bvlc_alexnet 5 input:=/camera/rgb/image_raw

_Arguments_

    rosrun deepros classifier_<framework> <model_id> <n_predictions> [<args>]

- `<model_id>`: Pretrained model to use for classification.
- `<n_pedictions>`: Number of predictions.
- `<args>` [Optional]: ROS additional arguments.

_Subscribed topics_

- `input` ([sensor\_msgs::Image](http://docs.ros.org/api/sensor_msgs/html/msg/Image.html)): Input image. To save resources, the node will only subscribe to this topics if there are subscriptors for the predictions topic.

_Published topics_

- `/deepros/predictions` (deepros\_msgs::Predictions): Top predictions. Please, see the deepros\_msgs package to check the message fields.

_Services_

- `/deepros/srv/predict` (deepros\_srvs::GetImagePredictions): It gets predictions for a single image provided as argument. Please, see the deepros\_srvs package to check the message fields.

### classification\_client

Example of client for the service `/deepros/srv/predict`. It can be used to obtain predictions for a single image, which will be printed to the standard output.

For example, while the classifier\_&lt;framework&gt; node is running, you can obtain the top 5 predictions for an image with:

    rosrun deepros classification_client <image> 5 

_Arguments_

    rosrun deepros classification_client <image> <n_predictions> [<args>]

- `<image>`: Image to classify.
- `<n_pedictions>`: Number of predictions.
- `<args>` [Optional]: ROS additional arguments.


### detector\_&lt;framework&gt;

This node publishes the detected objects for the input image received in the `input` topic. It also advertises a service `/deepros/srv/detections` to obtain detections for a single image.

For example, to obtain the detected objects using the Darknet YOLOv3 model from a stream of images provided by the openni2\_launch package, you could use:

    rosrun deepros detector_darknet yolov3 input:=/camera/rgb/image_raw

_Arguments_

    rosrun deepros detector_<framework> <model_id> [<args>]

- `<model_id>`: Pretrained model to use for detection.
- `<args>` [Optional]: ROS additional arguments.

_Subscribed topics_

- `input` ([sensor\_msgs::Image](http://docs.ros.org/api/sensor_msgs/html/msg/Image.html)): Input image. To save resources, the node will only subscribe to this topics if there are subscriptors for the detections topic.

_Published topics_

- `/deepros/detections` (deepros\_msgs::Prediction): Detections in the image. Please, see the deepros\_msgs package to check the message fields.

_Services_

- `/deepros/srv/detect` (deepros\_srvs::GetImageDetections): It gets detections for a single image provided as argument. Please, see the deepros\_srvs package to check the message fields.

### detection\_client

Example of client for the service `/deepros/srv/detect`. It can be used to obtain object detections for a single image, which will be printed to the standard output. Optionally (if the `output` argument is set), it can also draw the rectangles for the detections in the image and save it.

For example, while the detector\_&lt;framework&gt; node is running, you can obtain the detections for an image with:

    rosrun deepros detection_client <image>

_Arguments_

    rosrun deepros detection_client <image> [_output:=<output_file>] [<args>]

- `<image>`: Image to obtain detections from.
- `_output:=<output_file>` [Optional]: Output image file with drawn detections.
- `<args>` [Optional]: ROS additional arguments.


### visualize\_detections

This node subscribes to the `input` images topic and the obtained detections (`/deepros/detections`) to draw the bounding box of the detected elements in the image, and then it publishes the output image with the drawn detections in the `/deepros/detections_visualization` topic. It can be used to easily visualize the output of the detector\_&lt;framework&gt; node in `rviz`.

For example, while the openni2\_launch package is streaming the RGB images and the detector\_&lt;framework&gt; node is running, you can obtain the images with the bounding box drawn in them with:

    rosrun deepros visualize_detections input:=/camera/rgb/image_raw

_Arguments_

    rosrun deepros visualize_detections [<args>]

- `<args>` [Optional]: ROS additional arguments.

_Subscribed topics_

- `input` ([sensor\_msgs::Image](http://docs.ros.org/api/sensor_msgs/html/msg/Image.html)): Input images.
- `/deepros/detections` (deepros\_msgs::Prediction): Detections in the input image. Please, see the deepros\_msgs package to check the message fields.

_Published topics_

- `/deepros/detections_visualization` ([sensor\_msgs::Image](http://docs.ros.org/api/sensor_msgs/html/msg/Image.html)): Images with drawn detections.


## TODO

Integration with the following frameworks:
  * [TensorFlow](https://www.tensorflow.org/)
  * [Caffe2](https://caffe2.ai)

