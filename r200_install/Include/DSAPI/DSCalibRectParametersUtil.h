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
#include <cmath>

/// @defgroup Helper functions for calibration parameters
/// Inline helper functions that show how to use the calibration data that are obtained via the DSAPI interface.
/// @{

/// From z image to z camera (right-handed coordinate system).
/// zImage is assumed to contain [z row, z column, z depth].
/// Get zIntrinsics via DSAPI getCalibIntrinsicsZ.
inline void DSTransformFromZImageToZCamera(const DSCalibIntrinsicsRectified & zIntrinsics, const float zImage[3], float zCamera[3])
{
    zCamera[0] = zImage[2] * (zImage[0] - zIntrinsics.rpx) / zIntrinsics.rfx;
    zCamera[1] = zImage[2] * (zImage[1] - zIntrinsics.rpy) / zIntrinsics.rfy;
    zCamera[2] = zImage[2];
}

inline void DSTransformFromZCameraToRectOtherCamera(const double translation[3], const float zCamera[3], float OtherCamera[3])
{
    OtherCamera[0] = static_cast<float>(zCamera[0] + translation[0]);
    OtherCamera[1] = static_cast<float>(zCamera[1] + translation[1]);
    OtherCamera[2] = static_cast<float>(zCamera[2] + translation[2]);
}


/// From z camera to non-rectified Other camera
/// Get rotation and translation via DSAPI getCalibExtrinsicsZToNonRectOther.
inline void DSTransformFromZCameraToNonRectOtherCamera(const double rotation[9], const double translation[3], const float zCamera[3], float OtherCamera[3])
{
    OtherCamera[0] = static_cast<float>(rotation[0] * zCamera[0] + rotation[1] * zCamera[1] + rotation[2] * zCamera[2] + translation[0]);
    OtherCamera[1] = static_cast<float>(rotation[3] * zCamera[0] + rotation[4] * zCamera[1] + rotation[5] * zCamera[2] + translation[1]);
    OtherCamera[2] = static_cast<float>(rotation[6] * zCamera[0] + rotation[7] * zCamera[1] + rotation[8] * zCamera[2] + translation[2]);
}

/// From Other camera to rectified Other image
/// Get OtherIntrinsics via DSAPI getCalibIntrinsicsRectOther.
inline void DSTransformFromOtherCameraToRectOtherImage(const DSCalibIntrinsicsRectified & OtherIntrinsics, const float OtherCamera[3], float OtherImage[2])
{
    OtherImage[0] = OtherCamera[0] / OtherCamera[2];
    OtherImage[1] = OtherCamera[1] / OtherCamera[2];

    OtherImage[0] = OtherIntrinsics.rfx * OtherImage[0] + OtherIntrinsics.rpx;
    OtherImage[1] = OtherIntrinsics.rfy * OtherImage[1] + OtherIntrinsics.rpy;
}

/// From Other camera to non-rectified Other image
/// Get OtherIntrinsics via DSAPI getCalibIntrinsicsNonRectOther.
inline void DSTransformFromOtherCameraToNonRectOtherImage(const DSCalibIntrinsicsNonRectified & OtherIntrinsics, const float OtherCamera[3], float OtherImage[2])
{
    float t[2];
    t[0] = OtherCamera[0] / OtherCamera[2];
    t[1] = OtherCamera[1] / OtherCamera[2];

    const float * k = OtherIntrinsics.k;
    float r2 = t[0] * t[0] + t[1] * t[1];
    float r = static_cast<float>(1 + r2 * (k[0] + r2 * (k[1] + r2 * k[4])));
    t[0] *= r;
    t[1] *= r;

    OtherImage[0] = static_cast<float>(t[0] + 2 * k[2] * t[0] * t[1] + k[3] * (r2 + 2 * t[0] * t[0]));
    OtherImage[1] = static_cast<float>(t[1] + 2 * k[3] * t[0] * t[1] + k[2] * (r2 + 2 * t[1] * t[1]));

    OtherImage[0] = OtherIntrinsics.fx * OtherImage[0] + OtherIntrinsics.px;
    OtherImage[1] = OtherIntrinsics.fy * OtherImage[1] + OtherIntrinsics.py;
}


/// From z image to rectified Other image
/// Get zIntrinsics via DSAPI getCalibIntrinsicsZ.
/// Get translation via DSAPI getCalibExtrinsicsZToRectOther.
/// Get OtherIntrinsics via DSAPI getCalibIntrinsicsRectOther.
inline void DSTransformFromZImageToRectOtherImage(const DSCalibIntrinsicsRectified & zIntrinsics, const double translation[3], const DSCalibIntrinsicsRectified & OtherIntrinsics, const float zImage[3], float OtherImage[2])
{
    float zCamera[3];
    float OtherCamera[3];
    DSTransformFromZImageToZCamera(zIntrinsics, zImage, zCamera);
    DSTransformFromZCameraToRectOtherCamera(translation, zCamera, OtherCamera);
    DSTransformFromOtherCameraToRectOtherImage(OtherIntrinsics, OtherCamera, OtherImage);
}

/// From z image to non-rectified Other image
/// Get zIntrinsics via DSAPI getCalibIntrinsicsZ.
/// Get rotation and translation via DSAPI getCalibExtrinsicsZToNonRectOther.
/// Get OtherIntrinsics via DSAPI getCalibIntrinsicsNonRectOther.
inline void DSTransformFromZImageToNonRectOtherImage(const DSCalibIntrinsicsRectified & zIntrinsics, const double rotation[9], const double translation[3], const DSCalibIntrinsicsNonRectified & OtherIntrinsics, const float zImage[3], float OtherImage[2])
{
    float zCamera[3];
    float OtherCamera[3];
    DSTransformFromZImageToZCamera(zIntrinsics, zImage, zCamera);
    DSTransformFromZCameraToNonRectOtherCamera(rotation, translation, zCamera, OtherCamera);
    DSTransformFromOtherCameraToNonRectOtherImage(OtherIntrinsics, OtherCamera, OtherImage);
}


/// From rect Other image to non-rectified Other image
/// Get OtherIntrinsicsRect via DSAPI getCalibIntrinsicsRectOther.
/// Get rotation via DSAPI getCalibExtrinsicsRectOtherToNonRectOther.
/// Get OtherIntrinsicsNonRect via DSAPI getCalibIntrinsicsNonRectOther.
inline void DSTransformFromRectOtherImageToNonRectOtherImage(const DSCalibIntrinsicsRectified & OtherIntrinsicsRect, const double rotation[9], const DSCalibIntrinsicsNonRectified & OtherIntrinsicsNonRect, const float rectImage[2], float nonRectImage[2])
{
    float rectCamera[3];
    rectCamera[0] = (rectImage[0] - OtherIntrinsicsRect.rpx) / OtherIntrinsicsRect.rfx;
    rectCamera[1] = (rectImage[1] - OtherIntrinsicsRect.rpy) / OtherIntrinsicsRect.rfy;
    rectCamera[2] = 1;

    float nonRectCamera[3];
    nonRectCamera[0] = static_cast<float>(rotation[0] * rectCamera[0] + rotation[1] * rectCamera[1] + rotation[2] * rectCamera[2]);
    nonRectCamera[1] = static_cast<float>(rotation[3] * rectCamera[0] + rotation[4] * rectCamera[1] + rotation[5] * rectCamera[2]);
    nonRectCamera[2] = static_cast<float>(rotation[6] * rectCamera[0] + rotation[7] * rectCamera[1] + rotation[8] * rectCamera[2]);
    DSTransformFromOtherCameraToNonRectOtherImage(OtherIntrinsicsNonRect, nonRectCamera, nonRectImage);
}

/// From z camera to world.
/// Get rotation and translation via DSAPI getCalibZToWorldTransform.
inline void DSTransformFromZCameraToWorld(const double rotation[9], const double translation[3], const float zCamera[3], float world[3])
{
    world[0] = static_cast<float>(rotation[0] * zCamera[0] + rotation[1] * zCamera[1] + rotation[2] * zCamera[2] + translation[0]);
    world[1] = static_cast<float>(rotation[3] * zCamera[0] + rotation[4] * zCamera[1] + rotation[5] * zCamera[2] + translation[1]);
    world[2] = static_cast<float>(rotation[6] * zCamera[0] + rotation[7] * zCamera[1] + rotation[8] * zCamera[2] + translation[2]);
}

/// Compute field of view angles in degrees from rectified intrinsics
/// Get intrinsics via DSAPI getCalibIntrinsicsZ, getCalibIntrinsicsRectLeftRight or getCalibIntrinsicsRectOther
inline void DSFieldOfViewsFromIntrinsicsRect(const DSCalibIntrinsicsRectified & intrinsics, float & horizontalFOV, float & verticalFOV)
{
    horizontalFOV = atan2(intrinsics.rpx + 0.5f, intrinsics.rfx) + atan2(intrinsics.rw - intrinsics.rpx - 0.5f, intrinsics.rfx);
    verticalFOV = atan2(intrinsics.rpy + 0.5f, intrinsics.rfy) + atan2(intrinsics.rh - intrinsics.rpy - 0.5f, intrinsics.rfy);

    // Convert to degrees
    const float pi = 3.14159265358979323846f;
    horizontalFOV = horizontalFOV * 180.0f / pi;
    verticalFOV = verticalFOV * 180.0f / pi;
}

/// Deprecated functions
DS_DEPRECATED(inline void DSTransformFromZCameraToRectThirdCamera(const double translation[3], const float zCamera[3], float OtherCamera[3]), DSTransformFromZCameraToRectOtherCamera);
inline void DSTransformFromZCameraToRectThirdCamera(const double translation[3], const float zCamera[3], float OtherCamera[3])
{
    OtherCamera[0] = static_cast<float>(zCamera[0] + translation[0]);
    OtherCamera[1] = static_cast<float>(zCamera[1] + translation[1]);
    OtherCamera[2] = static_cast<float>(zCamera[2] + translation[2]);
}

DS_DEPRECATED(inline void DSTransformFromZCameraToNonRectThirdCamera(const double rotation[9], const double translation[3], const float zCamera[3], float OtherCamera[3]), DSTransformFromZCameraToNonRectOtherCamera);
inline void DSTransformFromZCameraToNonRectThirdCamera(const double rotation[9], const double translation[3], const float zCamera[3], float OtherCamera[3])
{
    OtherCamera[0] = static_cast<float>(rotation[0] * zCamera[0] + rotation[1] * zCamera[1] + rotation[2] * zCamera[2] + translation[0]);
    OtherCamera[1] = static_cast<float>(rotation[3] * zCamera[0] + rotation[4] * zCamera[1] + rotation[5] * zCamera[2] + translation[1]);
    OtherCamera[2] = static_cast<float>(rotation[6] * zCamera[0] + rotation[7] * zCamera[1] + rotation[8] * zCamera[2] + translation[2]);
}

DS_DEPRECATED(inline void DSTransformFromThirdCameraToRectThirdImage(const DSCalibIntrinsicsRectified & OtherIntrinsics, const float OtherCamera[3], float OtherImage[2]), DSTransformFromOtherCameraToRectOtherImage);
inline void DSTransformFromThirdCameraToRectThirdImage(const DSCalibIntrinsicsRectified & OtherIntrinsics, const float OtherCamera[3], float OtherImage[2])
{
    OtherImage[0] = OtherCamera[0] / OtherCamera[2];
    OtherImage[1] = OtherCamera[1] / OtherCamera[2];

    OtherImage[0] = OtherIntrinsics.rfx * OtherImage[0] + OtherIntrinsics.rpx;
    OtherImage[1] = OtherIntrinsics.rfy * OtherImage[1] + OtherIntrinsics.rpy;
}

DS_DEPRECATED(inline void DSTransformFromThirdCameraToNonRectThirdImage(const DSCalibIntrinsicsNonRectified & OtherIntrinsics, const float OtherCamera[3], float OtherImage[2]), DSTransformFromOtherCameraToNonRectOtherImage);
inline void DSTransformFromThirdCameraToNonRectThirdImage(const DSCalibIntrinsicsNonRectified & OtherIntrinsics, const float OtherCamera[3], float OtherImage[2])
{
    float t[2];
    t[0] = OtherCamera[0] / OtherCamera[2];
    t[1] = OtherCamera[1] / OtherCamera[2];

    const float * k = OtherIntrinsics.k;
    float r2 = t[0] * t[0] + t[1] * t[1];
    float r = static_cast<float>(1 + r2 * (k[0] + r2 * (k[1] + r2 * k[4])));
    t[0] *= r;
    t[1] *= r;

    OtherImage[0] = static_cast<float>(t[0] + 2 * k[2] * t[0] * t[1] + k[3] * (r2 + 2 * t[0] * t[0]));
    OtherImage[1] = static_cast<float>(t[1] + 2 * k[3] * t[0] * t[1] + k[2] * (r2 + 2 * t[1] * t[1]));

    OtherImage[0] = OtherIntrinsics.fx * OtherImage[0] + OtherIntrinsics.px;
    OtherImage[1] = OtherIntrinsics.fy * OtherImage[1] + OtherIntrinsics.py;
}

DS_DEPRECATED(inline void DSTransformFromZImageToRectThirdImage(const DSCalibIntrinsicsRectified & zIntrinsics, const double translation[3], const DSCalibIntrinsicsRectified & OtherIntrinsics, const float zImage[3], float OtherImage[2]), DSTransformFromZImageToRectOtherImage);
inline void DSTransformFromZImageToRectThirdImage(const DSCalibIntrinsicsRectified & zIntrinsics, const double translation[3], const DSCalibIntrinsicsRectified & OtherIntrinsics, const float zImage[3], float OtherImage[2])
{
    float zCamera[3];
    float OtherCamera[3];
    DSTransformFromZImageToZCamera(zIntrinsics, zImage, zCamera);
    DSTransformFromZCameraToRectOtherCamera(translation, zCamera, OtherCamera);
    DSTransformFromOtherCameraToRectOtherImage(OtherIntrinsics, OtherCamera, OtherImage);
}

DS_DEPRECATED(inline void DSTransformFromZImageToNonRectThirdImage(const DSCalibIntrinsicsRectified & zIntrinsics, const double rotation[9], const double translation[3], const DSCalibIntrinsicsNonRectified & OtherIntrinsics, const float zImage[3], float OtherImage[2]), DSTransformFromZImageToNonRectOtherImage);
inline void DSTransformFromZImageToNonRectThirdImage(const DSCalibIntrinsicsRectified & zIntrinsics, const double rotation[9], const double translation[3], const DSCalibIntrinsicsNonRectified & OtherIntrinsics, const float zImage[3], float OtherImage[2])
{
    float zCamera[3];
    float OtherCamera[3];
    DSTransformFromZImageToZCamera(zIntrinsics, zImage, zCamera);
    DSTransformFromZCameraToNonRectOtherCamera(rotation, translation, zCamera, OtherCamera);
    DSTransformFromOtherCameraToNonRectOtherImage(OtherIntrinsics, OtherCamera, OtherImage);
}

DS_DEPRECATED(inline void DSTransformFromRectThirdImageToNonRectThirdImage(const DSCalibIntrinsicsRectified & OtherIntrinsicsRect, const double rotation[9], const DSCalibIntrinsicsNonRectified & OtherIntrinsicsNonRect, const float rectImage[2], float nonRectImage[2]), DSTransformFromRectThirdImageToNonRectOtherImage);
inline void DSTransformFromRectThirdImageToNonRectThirdImage(const DSCalibIntrinsicsRectified & OtherIntrinsicsRect, const double rotation[9], const DSCalibIntrinsicsNonRectified & OtherIntrinsicsNonRect, const float rectImage[2], float nonRectImage[2])
{
    float rectCamera[3];
    rectCamera[0] = (rectImage[0] - OtherIntrinsicsRect.rpx) / OtherIntrinsicsRect.rfx;
    rectCamera[1] = (rectImage[1] - OtherIntrinsicsRect.rpy) / OtherIntrinsicsRect.rfy;
    rectCamera[2] = 1;

    float nonRectCamera[3];
    nonRectCamera[0] = static_cast<float>(rotation[0] * rectCamera[0] + rotation[1] * rectCamera[1] + rotation[2] * rectCamera[2]);
    nonRectCamera[1] = static_cast<float>(rotation[3] * rectCamera[0] + rotation[4] * rectCamera[1] + rotation[5] * rectCamera[2]);
    nonRectCamera[2] = static_cast<float>(rotation[6] * rectCamera[0] + rotation[7] * rectCamera[1] + rotation[8] * rectCamera[2]);
    DSTransformFromOtherCameraToNonRectOtherImage(OtherIntrinsicsNonRect, nonRectCamera, nonRectImage);
}

/// @}
