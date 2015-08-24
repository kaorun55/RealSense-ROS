/*******************************************************************************

INTEL CORPORATION PROPRIETARY INFORMATION
This software is supplied under the terms of a license agreement or nondisclosure
agreement with Intel Corporation and may not be copied or disclosed except in
accordance with the terms of that agreement
Copyright(c) 2014-2015 Intel Corporation. All Rights Reserved.

*******************************************************************************/

#pragma once

#include "DSAPITypes.h"

/// DSDepthControlPreset specifies the level of outlier removal done in hardware, controlled by DSDepthControlParameters
enum DSDepthControlPreset
{
    DS_DEPTHCONTROL_PRESET_OFF,       /// Disable almost all hardware-based outlier removal
    DS_DEPTHCONTROL_PRESET_LOW,       /// Provide a depthmap with a lower number of outliers removed, which has minimal false negatives.
    DS_DEPTHCONTROL_PRESET_MEDIUM,    /// Provide a depthmap with a medium number of outliers removed, which has balanced approach.
    DS_DEPTHCONTROL_PRESET_HIGH,      /// Provide a depthmap with a higher number of outliers removed, which has minimal false positives.
    DS_DEPTHCONTROL_PRESET_OPTIMIZED, /// Provide a depthmap with a medium/high number of outliers removed. Derived from an optimization function.
    DS_DEPTHCONTROL_PRESET_DEFAULT,   /// Default settings on chip. Similiar to the medium setting and best for outdoors.
    DS_DEPTHCONTROL_PRESET_COUNT      /// Helper enum value for iteration
};

/// Returns a set of Depth Control Parameters corresponding to a preset.
/// The Depth Control Parameters control hardware-based confidence thresholds of the depth data. This function provides helpful presets for their configuration.
/// Lower settings discard less data, providing more data density while retaining more erroneous data (i.e. more false positives).
/// Higher settings discard more data, providing less data density while rejecting more erroneous data (i.e. more false negatives).
///
/// Usage is typically:
///		dsapi->accessHardware()->setDepthControlParameters(DSGetDepthControlPreset(DS_DEPTHCONTROL_PRESET_DEFAULT));
///
/// @param mode which set of depth control presets to return
/// @return requested set of control parameters
/// @see DSAPI methods getDepthControlParameters, setDepthControlParameters
/// @see DSAPI struct DSDepthControlParameters

inline DSDepthControlParameters DSGetDepthControlPreset(DSDepthControlPreset mode)
{
    DSDepthControlParameters depthControl;
    switch (mode)
    {
    case DS_DEPTHCONTROL_PRESET_OFF:
        depthControl.robinsMunroeMinusIncrement = 5;
        depthControl.robinsMunroePlusIncrement = 5;
        depthControl.medianThreshold = 0;
        depthControl.scoreMinimumThreshold = 0;
        depthControl.scoreMaximumThreshold = 1023;
        depthControl.textureCountThreshold = 0;
        depthControl.textureDifferenceThreshold = 0;
        depthControl.secondPeakThreshold = 0;
        depthControl.neighborThreshold = 0;
        depthControl.lrThreshold = 2047;
        break;
    case DS_DEPTHCONTROL_PRESET_LOW:
        depthControl.robinsMunroeMinusIncrement = 5;
        depthControl.robinsMunroePlusIncrement = 5;
        depthControl.medianThreshold = 115;
        depthControl.scoreMinimumThreshold = 1;
        depthControl.scoreMaximumThreshold = 512;
        depthControl.textureCountThreshold = 6;
        depthControl.textureDifferenceThreshold = 18;
        depthControl.secondPeakThreshold = 25;
        depthControl.neighborThreshold = 3;
        depthControl.lrThreshold = 24;
        break;
    case DS_DEPTHCONTROL_PRESET_MEDIUM:
        depthControl.robinsMunroeMinusIncrement = 5;
        depthControl.robinsMunroePlusIncrement = 5;
        depthControl.medianThreshold = 185;
        depthControl.scoreMinimumThreshold = 5;
        depthControl.scoreMaximumThreshold = 505;
        depthControl.textureCountThreshold = 6;
        depthControl.textureDifferenceThreshold = 35;
        depthControl.secondPeakThreshold = 45;
        depthControl.neighborThreshold = 45;
        depthControl.lrThreshold = 14;
        break;
    case DS_DEPTHCONTROL_PRESET_HIGH:
        depthControl.robinsMunroeMinusIncrement = 5;
        depthControl.robinsMunroePlusIncrement = 5;
        depthControl.medianThreshold = 235;
        depthControl.scoreMinimumThreshold = 27;
        depthControl.scoreMaximumThreshold = 420;
        depthControl.textureCountThreshold = 8;
        depthControl.textureDifferenceThreshold = 80;
        depthControl.secondPeakThreshold = 70;
        depthControl.neighborThreshold = 90;
        depthControl.lrThreshold = 12;
        break;
    case DS_DEPTHCONTROL_PRESET_OPTIMIZED:
        depthControl.robinsMunroeMinusIncrement = 5;
        depthControl.robinsMunroePlusIncrement = 5;
        depthControl.medianThreshold = 175;
        depthControl.scoreMinimumThreshold = 24;
        depthControl.scoreMaximumThreshold = 430;
        depthControl.textureCountThreshold = 6;
        depthControl.textureDifferenceThreshold = 48;
        depthControl.secondPeakThreshold = 47;
        depthControl.neighborThreshold = 24;
        depthControl.lrThreshold = 12;
        break;
    case DS_DEPTHCONTROL_PRESET_DEFAULT:
    default:
        depthControl.robinsMunroeMinusIncrement = 5;
        depthControl.robinsMunroePlusIncrement = 5;
        depthControl.medianThreshold = 192;
        depthControl.scoreMinimumThreshold = 1;
        depthControl.scoreMaximumThreshold = 512;
        depthControl.textureCountThreshold = 6;
        depthControl.textureDifferenceThreshold = 24;
        depthControl.secondPeakThreshold = 27;
        depthControl.neighborThreshold = 7;
        depthControl.lrThreshold = 24;
    }
    return depthControl;
}
