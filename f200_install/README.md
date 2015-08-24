# Intel&reg; RealSense&trade; F200 API
    
## Installation

#### Getting the camera to work on Linux

In the package folder you will find ```f200_install``` folder.

Simply run 

    sh install.sh [path to install]. 

For example – 

    sh install.sh ~/ivcam

* Run 

        ./[installed folder]/ivcamsample/ivcam_sample 

The app just opens the camera and reads few frames. It does not have any GUI. The output should be something like that:

    $$$printParameters************************
    $$$2047.000000
    $$$1.485674
    $$$0.000000
    $$$0.131815
    $$$-0.055717
    $$$1.886967
    $$$-0.029558
    $$$0.000000
    $$$0.002917
    $$$-0.003724
    $$$1.420024
    $$$1.420011
    $$$0.999993
    $$$24.606842
    $$$0.000000
    $$$0.000000
    $$$-0.013931
    $$$printParameters ########################
    Failed to read image
    Failed to read image
    Error in Laser Power - UVC_GET_LEN
    ........................................................................
    ................................................................................
    ........................................


If you see the dots apearing on the screen, this means that the camera data is streaming. Just ignore any error messages that may apear before. 

* You can build manually the provided sample in ```[installed folder]/ivcamsample/``` by running the build script: 

        sh [installed folder]/ivcamsample/build.sh

## Known issues:
* First few frames may return error – “Failed to read image”. Just ignore those.
* You may get UVC_LASER_POWER error. If the camera is still working, then just ignore it.
* Sometimes, the camera doesn’t initialize as expected and the nodelet is dying after few seconds. Simply restart the nodelet.
* Check Changelog.txt for more info

## Tech and dependencies 

System:

* Linux 12.04+
* F200 (IVCAM) camera
