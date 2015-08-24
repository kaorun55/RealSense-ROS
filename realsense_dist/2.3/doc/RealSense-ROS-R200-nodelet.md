# Intel&reg; RealSense&trade; R200 Nodelet

Streaming data from the Intel&reg; RealSense&trade; R200 (DS4) camera

## Subscribed Topics
    None


## Publisehd Topics

Depth camera

    camera/depth/image_raw (sensor_msgs/Image)
        uint16 depths in mm.
    camera/depth/points (sensor_msgs/PointCloud2)
        Registered XYZRGB point cloud.
    camera/depth/uv (std_msgs/Float32MultiArray)
        UV mapping for depth image.
    camera/depth/camera_info
        Calibration data

Color camera

    camera/color/image_raw (sensor_msgs/Image)
        Color rectified image. RGB format.
    camera/color/camera_info
        Calibration data

## Parameters
    cHeight(int, default: 480) 
        Specify the color camera height resolution.
    cWidth(int, default: 640)
        Specify the color camera width resolution.
    dHeight(int, default: 360)
        Specify the depth camera height resolution.
    dWidth(int, default: 480)
        Specify the depth camera width resolution.
    pcScale(float, default: 1)
        Specify a scale divisor for the point cloud. 1 is mm and 1000 is meters.
    enableDepth(bool, default: 1) 
        Specify if to enable or not the depth camera. 1 is true. 0 is false.
    enableColor(bool, default: 1) 
        Specify if to enable or not the color camera. 1 is true. 0 is false.

## Services
    None


## Version
    2.0

## Known issues:
* After every run, there is a need to physically unplug and re-plug the camera. If not, an Error message “Cannot determine firmware version!” will appear when trying to access the camera.
* Check Changelog.txt for more info


## Running the R200 nodelet
Simply type:

    roslaunch realsense realsense_r200_launch.launch

This will launch the camera nodelet. You will see the camera lights up.

A quick check will be to run the stock image viewer to see the color image:

    rosrun image_view image_view image:=/camera/color/image_raw

You can also open RVIZ and load the provided RVIZ configuration file: realsenseRvizConfiguration.rviz.
![](realsenseRvizScreenshot.png)

## Tech and dependencies 
* libDSAPI.so (version 1.14.16)
* libpng16
* GLIBCXX_3.4.20

Upgrade by:

        sudo add-apt-repository ppa:ubuntu-toolchain-r/test
        sudo apt-get update
        sudo apt-get upgrade
        sudo apt-get dist-upgrade

libpng16:
    http://www.linuxfromscratch.org/blfs/view/svn/general/libpng.html

    Download compressed file from here:
    http://sourceforge.net/projects/libpng/files/libpng16/1.6.16/

    unzip & run:
    ./configure --prefix=/usr --disable-static &&
    make

    sudo make install &&
    sudo mkdir -v /usr/share/doc/libpng-1.6.16 &&
    sudo cp -v README libpng-manual.txt /usr/share/doc/libpng-1.6.16

** See ```ds_dependencies.txt``` for more information

System:

* Linux 12.04+
* ROS Indigo
* R200 (DS4) camera

** The ROS integration has been tested on a 64bit machine with Linux 14.04.01 and ROS Indigo.

## Installation

#### Getting the camera to work on Linux
<b><i>Please make sure that you have a working camera and that the software stack is installed properly. This can be checked by connecting the camera (to a USB3 port) and running one of the provided samples.</b></i>

If this does not work, you should first fix this issue before continuing with the ROS integration.

In the package folder you will find ```r200_install``` folder.

Un-pack and follow the ReadMe file in order to install the R200 driver and API.

* The provided patch_ubuntu_uvc.sh has some changes to the script in the packed files that works with Ubuntu 14.04. !! Use this file instead in this case.

In the ```ds_dependencies.txt``` you will find instructions how to download other dependences if you don’t have them already.

After installation is complete, restart your machine and run ```[unpacked folder]/Bin/DSSimpleCaptureGL``` app in order to test. You should see the depth and color streams in separate windows.

#### Getting the nodelet to work

In the provided package, please run

    /realsense_dist/[ver]/install_realsense_ros.sh

<b> Please note that you need to to have ROS sourced since we are using the $ROS_DISTRO variable. </b>

This will copy the nodelets and different configuration files to the ```/opt/ros/indigo```.

You can find a sample nodelet in the folder under the “sample” folder. This shows the basic access to the camera streams.

