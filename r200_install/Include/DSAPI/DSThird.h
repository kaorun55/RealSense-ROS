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

/// @class DSThird
/// Defines methods specific to an implementation that has a third camera. "Third" refers to a color camera that is part of the
/// module and calibrated with respect to the depth image (and Left and Right image).
//  DSAPI function accessThird() is used to return an instance of this class.
class DSThird
{
public:
    /// Get number supported resolution modes for third (includes pairings with framerates)
    virtual int getThirdNumberOfResolutionModes(bool rectified) = 0;
    /// For each index = 0 to getThirdNumberOfResolutionModes() - 1 will get the third width, third height and framerate.
    virtual bool getThirdResolutionMode(bool rectified, int index, int & thirdWidth, int & thirdHeight, int & thirdFps, DSPixelFormat & thirdPixelFormat) = 0;
    /// Set third width, third height and framerate.
    virtual bool setThirdResolutionMode(bool rectified, int thirdWidth, int thirdHeight, int thirdFps, DSPixelFormat thirdPixelFormat) = 0;

    /// Get current frame rate.
    virtual uint32_t getThirdFramerate() = 0;

    /// Returns time stamp of the current third image data.
    /// The double is representing time in seconds since 00:00 Coordinated Universal Time (UTC), January 1, 1970.
    /// If usePerformanceCounter is true, the double is representing performance time in seconds. Note that performance time is only valid when depth is streaming.
    virtual double getThirdFrameTime(bool usePerformanceCounter = false) = 0;

    /// Gets the current frame number for third. Currently not really a frame number, but will change for consecutive frames.
    virtual int getThirdFrameNumber() = 0;

    /// Returns the pixel format of the third imager.
    virtual DSPixelFormat getThirdPixelFormat() = 0;
    /// Returns true if pixel format is a native format for the third imager.
    virtual bool isThirdPixelFormatNative(DSPixelFormat pixelFormat) = 0;

    /// Returns true if rectification is enabled for the third imager.
    virtual bool isThirdRectificationEnabled() = 0;

    /// Enables the capture of images from the third imager.
    virtual bool enableThird(bool state) = 0;
    /// Returns true if capture of images from third imager is enabled.
    virtual bool isThirdEnabled() = 0;

    /// Returns a pointer to the image data from the third imager.
    /// If any conversions are necessary to provide the user with requested output format, they will be done in this call (not grab).
    virtual void * getThirdImage() = 0;

    /// Gets width of third image.
    virtual int thirdWidth() = 0;
    /// Gets height of third image.
    virtual int thirdHeight() = 0;

    /// Get calibration parameters for currently selected rectified mode
    virtual bool getCalibIntrinsicsRectThird(DSCalibIntrinsicsRectified & intrinsics) = 0;
    virtual bool getCalibExtrinsicsZToRectThird(double translation[3]) = 0;

    /// Get calibration parameters for currently selected non-rectified mode
    virtual bool getCalibIntrinsicsNonRectThird(DSCalibIntrinsicsNonRectified & intrinsics) = 0;
    virtual bool getCalibExtrinsicsZToNonRectThird(double rotation[9], double translation[3]) = 0;

    /// Get the rotation from the coordinate system of rectified third to the coordinate system
    /// of non-rectified third, defined such that Xnonrect = rotation * Xrect
    virtual bool getCalibExtrinsicsRectThirdToNonRectThird(double rotation[9]) = 0;

protected:
    // Creation (and deletion) of an object of this
    // type is supported through the DSFactory functions.
    DSThird(){};
    DSThird(const DSThird & other) DS_DELETED_FUNCTION;
    DSThird(DSThird && other) DS_DELETED_FUNCTION;
    DSThird & operator=(const DSThird & other) DS_DELETED_FUNCTION;
    DSThird & operator=(DSThird && other) DS_DELETED_FUNCTION;
    virtual ~DSThird(){};
};
