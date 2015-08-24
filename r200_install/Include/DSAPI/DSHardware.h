/*******************************************************************************

INTEL CORPORATION PROPRIETARY INFORMATION
This software is supplied under the terms of a license agreement or nondisclosure
agreement with Intel Corporation and may not be copied or disclosed except in
accordance with the terms of that agreement
Copyright(c) 2014-2015 Intel Corporation. All Rights Reserved.

*******************************************************************************/

#pragma once

#include "DSAPITypes.h"

/// @class DSHardware
/// Defines methods specific to a hardware implementation.
class DS_DECL DSHardware
{
public:
    /// Returns true if the hardware is active/ready.
    virtual bool isHardwareActive() = 0;

    /// Returns the version of DS ASIC used to compute data.
    virtual DSChipType getChipVersion() = 0;

    /// Turn autoexposure on or off. Must be called after the camera has been initialized and capture has begun.
    virtual bool setAutoExposure(DSWhichImager whichImager, bool state) = 0;
    /// Returns true if autoexposure is enabled.
    virtual bool getAutoExposure(DSWhichImager whichImager, bool & state) = 0;

    /// Sets imager gain factor when not in auto exposure mode.
    /// For left and right imagers, the unit is true gain multiplier factor, e.g. gain factor of 2.0 is twice as bright as 1.0.
    /// @param gain the gain factor
    /// @param which  which imager does the request apply to.
    /// @frameRate which frame rate does the request apply to. When -1 will use either current framerate (if streaming) or default framerate
    /// @return true for success, false on error.
    virtual bool setImagerGain(float gain, DSWhichImager whichImager, int frameRate = -1) = 0;

    /// Gets imager gain factor when not in auto exposure mode.
    /// @param[out] gain the gain factor.
    /// @param which which imager does the request apply to.
    /// @frameRate which frame rate does the request apply to. When -1 will use either current framerate (if streaming). Framerate required before streaming.
    /// @return true for success, false on error.
    virtual bool getImagerGain(float & gain, DSWhichImager whichImager, int frameRate = -1) = 0;

    /// Gets imager gain range when imager is in manual exposure mode.  No effect in autoexposure mode.
    /// For DSWhichImager = DS_LEFT_IMAGER, DS_RIGHT_IMAGER, or DS_BOTH_IMAGERS the gain value is a gain multiplier factor and frameRate argument is supported
    /// For other values, the gain units depend on the device. Lower values are darker, higher values are brighter. frameRate is also not supported.
    /// range returned is over all frame rates.
    /// @param[out] minGain the minimum value
    /// @param maxGain the maximum value
    /// @param whichImager  which imager does the request apply to.
    /// @frameRate which frame rate does the request apply to. When -1 will use either current framerate (if streaming). Framerate required before streaming.
    /// @return true for success, false on error.
    virtual bool getImagerMinMaxGain(float & minGain, float & maxGain, float & defaultValue, float & delta, DSWhichImager whichImager, int frameRate = -1) = 0;

    /// Sets imager exposure time (in ms) when not in auto exposure mode.
    /// For DSWhichImager = DS_LEFT_IMAGER, DS_RIGHT_IMAGER, or DS_BOTH_IMAGERS the exposure value is used directly to set exposure in millisec, precision is .1 ms
    /// For DSWhichImager = DS_THIRD_IMAGER, on Windows the value is converted to the standard units of Windows Media Foundation log2(exposureMillisec / 1000.0)
    /// @param exposureTime the exposure time.
    /// @param which which imager does the request apply to.
    /// @frameRate which frame rate does the request apply to. When -1 will use either current framerate (if streaming). Framerate required before streaming.
    /// @return true for success, false on error.
    virtual bool setImagerExposure(float exposure, DSWhichImager whichImager, int frameRate = -1) = 0;

    /// Gets imager exposure time (in ms) when not in auto exposure mode.
    /// @param[out] exposure the exposure time.
    /// @param which which imager does the request apply to.
    /// @frameRate which frame rate does the request apply to. When -1 will use either current framerate (if streaming). Framerate required before streaming.
    /// @return true for success, false on error.
    virtual bool getImagerExposure(float & exposure, DSWhichImager whichImager, int frameRate = -1) = 0;

    /// Gets imager exposure range (in ms) when not in auto exposure mode.
    /// For Left and Right imagers, the frame rate argument is supported.  For DS_THIRD_IMAGER imager the range returned is
    /// the range across all frame rates.
    /// @param[out] minExposure the minimum exposure time.
    /// @param which which imager does the request apply to.
    /// @frameRate which frame rate does the request apply to. This is used for Left and Right imagers only.
    /// @return true for success, false on error.
    virtual bool getImagerMinMaxExposure(float & minExposure, float & maxExposure, float & defaultValue, float & delta, DSWhichImager whichImager, int frameRate = -1) = 0;

    /// Sets Brightness. The brightness control is used to set the desired brightness of the scene when autoexposure mode is enabled.
    /// When autoexposure is disabled, any change will be rejected.
    /// Supported only for DSWhichImager =  DS_THIRD_IMAGER.
    virtual bool setBrightness(int val, DSWhichImager whichImager) = 0;
    virtual bool getBrightness(int & val, DSWhichImager whichImager) = 0;
    virtual bool getMinMaxBrightness(int & min, int & max, int & defaultValue, int & delta, DSWhichImager whichImager) = 0;

    /// Sets contrast, which is a value expressed as a gain factor multiplied by 100. Windows range is 0 - 10000, default is 100 (x1)
    /// Supported only for DSWhichImager =  DS_THIRD_IMAGER.
    virtual bool setContrast(int val, DSWhichImager whichImager) = 0;
    virtual bool getContrast(int & val, DSWhichImager whichImager) = 0;
    virtual bool getMinMaxContrast(int & min, int & max, int & defaultValue, int & delta, DSWhichImager whichImager) = 0;

    /// Sets saturation, which is a value expressed as a gain factor multiplied by 100. Windows range is 0 - 10000, default is 100 (x1)
    /// Supported only for DSWhichImager =  DS_THIRD_IMAGER.
    virtual bool setSaturation(int val, DSWhichImager whichImager) = 0;
    virtual bool getSaturation(int & val, DSWhichImager whichImager) = 0;
    virtual bool getMinMaxSaturation(int & min, int & max, int & defaultValue, int & delta, DSWhichImager whichImager) = 0;

    /// Sets hue, which is a value expressed as degrees multiplied by 100. Windows & UVC range is -18000 to 18000 (-180 to +180 degrees), default is 0
    /// Supported only for DSWhichImager =  DS_THIRD_IMAGER.
    virtual bool setHue(int val, DSWhichImager whichImager) = 0;
    virtual bool getHue(int & val, DSWhichImager whichImager) = 0;
    virtual bool getMinMaxHue(int & min, int & max, int & defaultValue, int & delta, DSWhichImager whichImager) = 0;

    /// Sets gamma, this is expressed as gamma multiplied by 100. Windows and UVC range is 1 to 500.
    /// Supported only for DSWhichImager =  DS_THIRD_IMAGER.
    virtual bool setGamma(int val, DSWhichImager whichImager) = 0;
    virtual bool getGamma(int & val, DSWhichImager whichImager) = 0;
    virtual bool getMinMaxGamma(int & min, int & max, int & defaultValue, int & delta, DSWhichImager whichImager) = 0;

    /// Sets white balance. Color temperature, in degrees Kelvin. Windows has no defined range.
    /// UVC range is 2800 (incandescent) to 6500 (daylight) but still needs to provide range. Only valid for DS_THIRD_IMAGER currently
    /// Supported only for DSWhichImager =  DS_THIRD_IMAGER.
    virtual bool setWhiteBalance(int val, DSWhichImager whichImager) = 0;
    virtual bool getWhiteBalance(int & val, DSWhichImager whichImager) = 0;
    virtual bool getMinMaxWhiteBalance(int & min, int & max, int & defaultValue, int & delta, DSWhichImager whichImager) = 0;

    /// Toggles auto white balance functions.
    /// Supported only for DSWhichImager =  DS_THIRD_IMAGER.
    virtual bool setUseAutoWhiteBalance(bool state, DSWhichImager whichImager) = 0;
    /// Gets current value of use auto white balance functions. Only valid for DS_THIRD_IMAGER currently
    virtual bool getUseAutoWhiteBalance(bool & state, DSWhichImager whichImager) = 0;

    /// Sets sharpness. Arbitrary units. UVC has no specified range (min sharpness means no sharpening), Windows required range must be 0 through 100. The default value must be 50.
    /// Supported only for DSWhichImager =  DS_THIRD_IMAGER.
    virtual bool setSharpness(int val, DSWhichImager whichImager) = 0;
    virtual bool getSharpness(int & val, DSWhichImager whichImager) = 0;
    virtual bool getMinMaxSharpness(int & min, int & max, int & defaultValue, int & delta, DSWhichImager whichImager) = 0;

    /// Sets back light compensation. A value of false indicates that the back-light compensation is disabled. The default value of true indicates that the back-light compensation is enabled.
    /// Supported only for DSWhichImager =  DS_THIRD_IMAGER.
    virtual bool setBacklightCompensation(int val, DSWhichImager whichImager) = 0;
    virtual bool getBacklightCompensation(int & val, DSWhichImager whichImager) = 0;
    virtual bool getMinMaxBacklightCompensation(int & min, int & max, int & defaultValue, int & delta, DSWhichImager whichImager) = 0;

    /// Sends an i2c command to the imager(s) designated by DSWhichImagers. Register regAddress is given value regValue.
    /// @param DSWhichImagers one of DS_LEFT_IMAGER, DS_RIGHT_IMAGER DS_BOTH_IMAGERS
    /// @param regAddress  the i2c register address
    /// @param regValue  the value to set register regAddress to. This value could be 2 bytes or 4 bytes for the third image and the overloaded versions are for this.
    /// @param noCheck if true, do not check whether the write occurred correctly.
    /// @return false on fail else true.
    virtual bool setImagerRegister(DSWhichImager whichImager, uint16_t regAddress, uint8_t regValue, bool noCheck = false) = 0;
    virtual bool setImagerRegister(DSWhichImager whichImager, uint16_t regAddress, uint16_t regValue, bool noCheck = false) = 0;
    virtual bool setImagerRegister(DSWhichImager whichImager, uint16_t regAddress, uint32_t regValue, bool noCheck = false) = 0;
    /// Sends an i2c command to the imager designated by whichImagers. regValue is set to the contents of Register regAddress.
    /// @param DSWhichImagers one of DS_LEFT_IMAGER, DS_RIGHT_IMAGER DS_BOTH_IMAGERS
    /// @param regAddress  the i2c register address
    /// @param[out] regValue  where to put the value of register regAddress. This value could be 2 bytes or 4 bytes for the third image and the overloaded versions are for this.
    /// @return false on fail else true.
    virtual bool getImagerRegister(DSWhichImager whichImager, uint16_t regAddress, uint8_t & regValue) = 0;
    virtual bool getImagerRegister(DSWhichImager whichImager, uint16_t regAddress, uint16_t & regValue) = 0;
    virtual bool getImagerRegister(DSWhichImager whichImager, uint16_t regAddress, uint32_t & regValue) = 0;

    /// Sets power line frequency.
    /// Supported only for DSWhichImager =  DS_THIRD_IMAGER.
    virtual bool setPowerLineFrequency(DSPowerLineFreqOption plf, DSWhichImager whichImager) = 0;
    /// Gets power line frequency.
    /// Supported only for DSWhichImager =  DS_THIRD_IMAGER.
    virtual bool getPowerLineFrequency(DSPowerLineFreqOption & plf, DSWhichImager whichImager) = 0;
    /// Gets power line frequency minimal, maximal, and default value.
    /// Supported only for DSWhichImager =  DS_THIRD_IMAGER.
    virtual bool getMinMaxPowerLineFequency(DSPowerLineFreqOption & min, DSPowerLineFreqOption & max, DSPowerLineFreqOption & defaultValue, DSWhichImager whichImager) = 0;

    /// Reads temperature to describe the current temperature of the camera module.
    /// @param curTemperature current temperature in Celsius.
    /// @param minTemperature minimal temperature in Celsius since last reset.
    /// @param maxTemperature maximal temperature in Celsius since last reset.
    /// @param minFaultThreshold minimal temperature allowed in Celsius for the module to operate.
    /// @return true for success, false on error.
    virtual bool getTemperature(int8_t & curTemperature, int8_t & minTemperature, int8_t & maxTemperature, int8_t & minFaultThreshold) = 0;
    /// Resets the min and max recorded temperature.
    virtual bool resetMinMaxRecordedTemperture() = 0;

    /// Set/get parameters that affect the stereo algorithm. See DSAPITypes.h for documentation.
    virtual bool setDepthControlParameters(const DSDepthControlParameters & parameters) = 0;
    virtual bool getDepthControlParameters(DSDepthControlParameters & parameters) = 0;

    /// Sets the disparity shift, which reduces both the minimum and maximum depth that can be computed.
    /// The disparity shift is applied to the right image before matching. This allows range to be computed
    /// for points in the near field which would otherwise be beyond the disparity search range.
    /// A non-zero shift also correspondingly reduces the maximum depth that can be measured.
    /// Units are pixels.
    // CURRENTLY NOT IMPLEMENTED IN FW
    virtual bool setDisparityShift(uint32_t shift) = 0;

    /// Get the 32 bit hardware status.
    /// @param status the 32 bit hardware status returned.
    /// @return true for success, false on error.
    virtual bool getHardwareStatus(uint32_t & status) = 0;

protected:
    // Creation (and deletion) of an object of this
    // type is supported through the DSFactory functions.
    DSHardware(){};
    DSHardware(const DSHardware & other) DS_DELETED_FUNCTION;
    DSHardware(DSHardware && other) DS_DELETED_FUNCTION;
    DSHardware & operator=(const DSHardware & other) DS_DELETED_FUNCTION;
    DSHardware & operator=(DSHardware && other) DS_DELETED_FUNCTION;
    virtual ~DSHardware(){};
};
