# Darknet: Install and configuration

Here you will find the instructions to intall Darknet in your system, and to **configure the CMakeLists.txt file of the ROS node to use Darknet**. There is a CMakeLists.txt.template availabe in the deepros directory, copy this file and modify it.

    cp <deeros_dir>/CMakeLists.txt.template <deepros_dir>/CMakeLists.txt
    
**IMPORTANT:** The following instructions are designed for commit f6d861736038da22c9eb0739dca84003c5a5e275 of the darknet repo. If you have a newer version of the code, you can switch back with:
    
    git checkout f6d861736038da22c9eb0739dca84003c5a5e275

## Install and configuration

Please, follow the instructions in <https://pjreddie.com/darknet/install/> to compile the darknet framework (preferably with GPU support). **When you are finished, you will need to set the directory of your Darknet sources in the CMakeLists.txt file of the deepros package**. Please, add/modify the following line:

    set(darknet_DIR <darknet_dir>)
    
And uncomment the following lines:

    set(darknet_INCLUDE_DIRS ${darknet_DIR}/include ${darknet_DIR}/src)
    set(darknet_LIBRARIES ${darknet_DIR}/libdarknet.so glog)
    set(darknet_FOUND 1)

## Download models

The models/darknet folder will store the different pretrained models available. It contains the models configuration and the needed script to download additional files. These models are available in the [Darknet website](https://pjreddie.com/darknet/).

With the download\_model.py script you can download the different models and the auxiliar files. This script is a modified version of the download\_model\_binary.py file from the [Caffe](http://caffe.berkeleyvision.org) source code.

You can download the models with:

    python download_model.py <model_id>

Which will download the following files:

- <model\_id>/<model\_id>.weights
- <model\_id>/<model\_id>.cfg
- <model\_id>/<model\_id>.labels

For example, to download the YOLOv3 model you would run:

    python download_model.py yolov3
    
    
### Alternative models

The available models are:
* darknet-reference (Classification)
* extraction (Classification)
* yolov2 (Detection)
* yolov3 (Detection)

If you want to use a different model, you only need to add the weights file (<model\_id>.weights), architecture file (<model\_id>.cfg) and labels file (<model\_id>.labels) to the <model\_id> folder in models/darknet. Alternatively, you could create a config file <model\_id>/config.yaml with the URLs to download them. For example, the config.yaml file for the YOLOv3 network is:

    name: yolov3
    weights_url: https://pjreddie.com/media/files/yolov3.weights
    cfg_url: https://raw.githubusercontent.com/pjreddie/darknet/master/cfg/yolov3.cfg
    labels_url: https://raw.githubusercontent.com/pjreddie/darknet/master/data/coco.names

The pretrained models for Darknet can be found in:
* Classification: <https://pjreddie.com/darknet/imagenet/>
* Detection: <https://pjreddie.com/darknet/yolo/>

