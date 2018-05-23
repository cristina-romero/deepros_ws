# Deep Learning + ROS workspace

ROS workspace with different packages to use pretrained Deep Learning models.

## Install

1. Download the code

        git clone https://github.com/cristina-romero/deepros_ws.git

2. Compile

        cd deepros_ws/src
        catkin_init_workspace
        cd ..
        catkin_make
        source devel/setup.bash

3. Check the documentation of the different packages for usage/compilation instructions.

## Packages

### deepros\_msgs

Package that contains the message definitions for the deepros node.

### deepros\_srvs

Package that contains the service definitions for the deepros node.

### deepros

Deep Learning + ROS. This ROS package integrates different Deep Learning frameworks (currently, [Caffe](http://caffe.berkeleyvision.org/) and [Darknet](https://pjreddie.com/darknet/)) to use pretrained models for image classification and object detection.

