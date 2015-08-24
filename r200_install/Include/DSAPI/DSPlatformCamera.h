/*******************************************************************************

INTEL CORPORATION PROPRIETARY INFORMATION
This software is supplied under the terms of a license agreement or nondisclosure
agreement with Intel Corporation and may not be copied or disclosed except in
accordance with the terms of that agreement
Copyright(c) 2014-2015 Intel Corporation. All Rights Reserved.

*******************************************************************************/

#pragma once

#include "DSAPITypes.h"
#include "DSCalibRectParameters.h"

/// @class DSPlatformCamera
/// Defines methods specific to an implementation that has a platform camera.
/// A "platform" camera is a camera separate from the DS module that is calibrated at integration time
//  Key parameter, in addition to calibration data, is a flag recorded in the module flash memory to tell
/// if platform camera is supported.  DSAPI function accessPlatformCamera() is used to
/// both determine if a PlatformCamera is supported and to return an instance of this class.
class DSPlatformCamera
{
public:
    /// Get number supported resolution modes
    virtual int getPlatformCameraNumberOfResolutionModes(bool rectified) = 0;
    /// For each index = 0 to getPlatformCameraNumberOfResolutionModes() - 1, will get the platform camera image width,  height
    virtual bool getPlatformCameraResolutionMode(bool rectified, int index, int & width, int & height) = 0;
    /// Select platform camera width, platform camera height.
    virtual bool setPlatformCameraResolutionMode(bool rectified, int width, int height) = 0;

    // Supported focus settings used at calibration and required for use with provided calibration data
    virtual bool getSupportedFocusSetting(int & val) = 0;

    /// Gets selected width of platform camera image.
    virtual int platformCameraWidth() = 0;
    /// Gets selected height of platform camera image.
    virtual int platformCameraHeight() = 0;

    /// Get calibration parameters for currently selected rectified mode
    virtual bool getCalibIntrinsicsRectPlatformCamera(DSCalibIntrinsicsRectified & intrinsics) = 0;
    virtual bool getCalibExtrinsicsZToRectPlatformCamera(double translation[3]) = 0;

    /// Get calibration parameters for currently selected non-rectified mode
    virtual bool getCalibIntrinsicsNonRectPlatformCamera(DSCalibIntrinsicsNonRectified & intrinsics) = 0;
    virtual bool getCalibExtrinsicsZToNonRectPlatformCamera(double rotation[9], double translation[3]) = 0;

    /// Get the rotation from the coordinate system of rectified platform camera to the coordinate system
    /// of non-rectified platform camera, defined such that Xnonrect = rotation * Xrect
    virtual bool getCalibExtrinsicsRectPlatformCameraToNonRectPlatformCamera(double rotation[9]) = 0;

protected:
    // Creation (and deletion) of an object of this
    // type is supported through the DSFactory functions.
    DSPlatformCamera(){};
    DSPlatformCamera(const DSPlatformCamera & other) DS_DELETED_FUNCTION;
    DSPlatformCamera(DSPlatformCamera && other) DS_DELETED_FUNCTION;
    DSPlatformCamera & operator=(const DSPlatformCamera & other) DS_DELETED_FUNCTION;
    DSPlatformCamera & operator=(DSPlatformCamera && other) DS_DELETED_FUNCTION;
    virtual ~DSPlatformCamera(){};
};
