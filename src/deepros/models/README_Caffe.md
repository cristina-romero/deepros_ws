# Caffe: Install and configuration

Here you will find the instructions to intall Caffe in your system (compiling with either make or cmake), and to **configure the CMakeLists.txt file of the ROS node to use Caffe**. There is a CMakeLists.txt.template availabe in the deepros directory, copy this file and modify it.

    cp <deeros_dir>/CMakeLists.txt.template <deepros_dir>/CMakeLists.txt

## Install and configuration

### With cmake

These are the instructions to build and install Caffe. If you have already installed Caffe in your system, you can modify the Caffe\_DIR in the CMakeLists.txt and continue with the models download.

First, install some dependencies with:

    sudo apt-get install libprotobuf-dev libleveldb-dev libsnappy-dev libopencv-dev  libhdf5-serial-dev protobuf-compiler
    sudo apt-get install --no-install-recommends libboost-all-dev
    sudo apt-get install libatlas-base-dev
    sudo apt-get install libgflags-dev libgoogle-glog-dev liblmdb-dev

If you want to compile Caffe with GPU support, you will also need to install [CUDA](https://developer.nvidia.com/cuda-downloads). Then, download Caffe:

    git clone https://github.com/BVLC/caffe.git

and build and install it:

    mkdir build
    cd build
    cmake -DCMAKE_INSTALL_PREFIX=<install_dir> -DCMAKE_BUILD_TYPE=Release ..
    make all
    make install
    make runtest

where <install_dir> is a custom folder to install the library.

Finally, **you will need to modify the CMakeLists.txt of the deepros package to set the path to your CaffeConfig.cmake file in Caffe_DIR**. If you installed Caffe in a custom folder you might need to add/modify the following line:

    set(Caffe_DIR <install_dir>/share/Caffe)

And uncomment the following line:

    find_package(Caffe REQUIRED)

**Note:** If you have any problem installing Caffe, you should check the following page for more detailed instructions: <http://caffe.berkeleyvision.org/installation.html>

### With make

To compile with make, please, follow the instructions in <http://caffe.berkeleyvision.org/installation.html#compilation>. **When you are finished, you will need to set the directory of your Caffe sources in the CMakeLists.txt file of the deepros package**. Please, add/modify the following line:

    set(Caffe_DIR <caffe_dir>)
    
And uncomment the following lines:

    set(Caffe_INCLUDE_DIRS ${Caffe_DIR}/build/include ${Caffe_DIR}/distribute/include)
    set(Caffe_LIBRARIES ${Caffe_DIR}/build/lib/libcaffe.so)

## Download models

The models/caffe folder will store the different pretrained models available. It contains the models configuration and the needed scripts to download additional files. These models belong to either the [Caffe Model Zoo and are pretrained with ImageNet](http://caffe.berkeleyvision.org/model_zoo.html), or to [Places-CNNs and are pretrained with the Places205 dataset](http://places.csail.mit.edu/downloadCNN.html).

With the download\_model\_binary.py script you can download the different models and the auxiliar files. This script is a modified version of the same file from the Caffe source code, with added support to download models pretrained with the Places dataset.

You can download the models with:

    python download_model_binary.py <model_id>

Which will download the following files (if they do not already exist):

- <model\_id>/<model\_id>.caffemodel
- <model\_id>/deploy.prototxt
- <model\_id>/synset_words.txt
- <model\_id>/mean.binaryproto

For example, to download the bvlc\_reference\_caffenet model you would run:

    python download_model_binary.py bvlc_reference_caffenet
    
The available models are:
* bvlc\_alexnet
* bvlc\_googlenet
* bvlc\_reference\_caffenet
* places\_alexnet
* places\_googlenet
* places\_vgg
        
## Credits

The Caffe models are released for unrestricted use. See <http://caffe.berkeleyvision.org/model_zoo.html#bair-model-license>

Please, cite the following paper if you use the Places models (<http://places.csail.mit.edu/>):
* B. Zhou, A. Lapedriza, J. Xiao, A. Torralba, and A. Oliva. “Learning Deep Features for Scene Recognition using Places Database.” Advances in Neural Information Processing Systems 27 (NIPS), 2014

