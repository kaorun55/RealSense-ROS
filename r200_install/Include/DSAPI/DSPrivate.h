/*******************************************************************************

INTEL CORPORATION PROPRIETARY INFORMATION
This software is supplied under the terms of a license agreement or nondisclosure
agreement with Intel Corporation and may not be copied or disclosed except in
accordance with the terms of that agreement
Copyright(c) 2014-2015 Intel Corporation. All Rights Reserved.

*******************************************************************************/

#pragma once

#include "DSAPITypes.h"

/// Private defines and constants

enum OEMID
{
    OEM_NONE = 0
};

enum DSLensType
{
    DS_LENS_UNKNOWN = 0,    ///< Lens either unknown or not needing special treatment
    DS_LENS_DSL103 = 1,     ///< Sunex DSL103: Internal standard
    DS_LENS_DSL821C = 2,    ///< Sunex DSL 821C
    DS_LENS_DSL202A = 3,    ///< Sunex DSL 202A
    DS_LENS_DSL203 = 4,     ///< Sunex DSL 203
    DS_LENS_PENTAX2514 = 5, ///< Pentax cmount lens 25mm
    DS_LENS_DSL924A = 6,    ///< Sunex DSL 924a
    DS_LENS_AZW58 = 7,      ///< 58 degree lenses on the AZureWave boards (DS-526)
    DS_LENS_Largan9386 = 8, ///< 50 HFOV 38 VFOV (60DFOV): CTM2/6 Module L&R
    DS_LENS_DS6100 = 9,     ///< Newmax 67.8 x 41.4 degs in 1080p
    DS_LENS_DS6177 = 10,    ///< Newmax 71.7 x 44.2 degs in 1080p
    DS_LENS_DS6237 = 11,    ///< Newmax 58.9 x 45.9 degs in VGA
    DS_LENS_DS6233 = 12,    ///< IR lens
    DS_LENS_DS917 = 13,     ///< RGB lens
    DS_LENS_COUNT = 14      ///< Just count
};
DS_DECL const char * DSLensTypeString(DSLensType lensType);

enum DSLensCoatingType
{
    DS_LENS_COATING_UNKNOWN = 0,
    DS_LENS_COATING_IR_CUT = 1,         ///< IR coating DS4: Innowave 670 cut off
    DS_LENS_COATING_ALL_PASS = 2,       ///< No IR coating
    DS_LENS_COATING_IR_PASS = 3,        ///< Visible-light block / IR pass:  center 860, width 25nm
    DS_LENS_COATING_IR_PASS_859_43 = 4, ///< Visible-light block / IR pass  center 859, width 43nm
    DS_LENS_COATING_COUNT = 5
};
DS_DECL const char * DSLensCoatingTypeString(DSLensCoatingType lensCoatingType);

enum DSEmitterType
{
    DS_EMITTER_NONE = 0,
    DS_EMITTER_LD2 = 1,  ///< Laser Driver 2, NO PWM Controls
    DS_EMITTER_LD3 = 2,  ///< Laser Driver 3
    DS_EMITTER_COUNT = 3 ///< Just count

};
DS_DECL const char * DSEmitterTypeString(DSEmitterType emitterType);

enum DSPlatformImagerType
{
    DS_IMAGER_UNKNOWN = 0, ///< Imager unknown
    DS_IMAGER_OVT8858 = 1,
    DS_IMAGER_COUNT = 2
};
DS_DECL const char * DSPlatformImagerTypeString(DSPlatformImagerType imagerType);

/// @class DSPrivate
/// Defines private methods used only in internal projects.
class DS_DECL DSPrivate
{
public:
    /// Sets emitter type for this camera
    virtual bool setEmitterType(DSEmitterType et, bool writeThrough = true) = 0;
    /// Gets emitter type for this camera
    virtual bool getEmitterType(DSEmitterType & et) = 0;

    /// Sets lens type for this camera
    virtual bool setCameraLensType(DSLensType lt, bool writeThrough = true) = 0;
    /// Gets lens type for this camera
    virtual bool getCameraLensType(DSLensType & lt) = 0;

    /// Sets lens type for this camera
    virtual bool setCameraLensCoatingType(DSLensCoatingType lct, bool writeThrough = true) = 0;
    /// Gets lens coating type for this camera
    virtual bool getCameraLensCoatingType(DSLensCoatingType & lct) = 0;

    /// Sets nominal baseline for this camera in mm (30, 60, 80, 140, 220, 330)
    virtual bool setCameraNominalBaseline(int nomBaseline, bool writeThrough = true) = 0;
    /// Gets nominal baseline for this camera in mm (30, 60, 80, 140, 220, 330)
    virtual bool getCameraNominalBaseline(int & nomBaselineMM) = 0;

    /// Sets lens type for this camera
    virtual bool setCameraLensTypeThird(DSLensType lt, bool writeThrough = true) = 0;
    /// Gets lens type for this camera
    virtual bool getCameraLensTypeThird(DSLensType & lt) = 0;

    /// Sets lens type for this camera
    virtual bool setCameraLensCoatingTypeThird(DSLensCoatingType lct, bool writeThrough = true) = 0;
    /// Gets lens coating type for this camera
    virtual bool getCameraLensCoatingTypeThird(DSLensCoatingType & lct) = 0;

    /// Sets error number
    virtual void setError(DSStatus e) = 0;

    /// Sets the entire calibration data struct in camera memory. Do not use this unless you really
    /// are re-calibrating your camera and know what you are doing. If permanent is true, the parameters
    /// will be stored permanently in camera memory. Otherwise the parameters are applied and used,
    /// but will revert to original parameters on restart of the camera.
    /// @return false on fail else true.
    virtual bool setCalibRectParameters(const DSCalibRectParameters & params, bool permanent) = 0;
    virtual bool setCalibZToWorldTransform(double rotation[9], double translation[3]) = 0;

    /// Set the camera module info data
    /// @return false on fail else true.
    virtual bool setCameraModuleInfo(const uint8_t data[2048]) = 0;

    /// Get the camera module info data
    /// @return false on fail else true.
    virtual bool getCameraModuleInfo(uint8_t data[2048]) = 0;

    /// Get the recovery calibration data
    virtual bool setRecoveryCalibRectParameters(const DSCalibRectParameters & params) = 0;

    /// Get the recovery calibration data
    /// @return false on fail else true.
    virtual bool getRecoveryCalibRectParameters(DSCalibRectParameters & param) = 0;

    /// Set the recovery camera module info data
    /// @return false on fail else true.
    virtual bool setRecoveryCameraModuleInfo(const uint8_t data[2048]) = 0;

    /// Get the recovery camera module info data
    /// @return false on fail else true.
    virtual bool getRecoveryCameraModuleInfo(uint8_t data[2048]) = 0;

    /// Specifies a path to read a "CalibrationRectParam.dat" file, in place of the parameters read from the camera head.
    DS_NOT_IMPLEMENTED virtual void setPathOfCalibrationRectParamsInsteadOfParametersFromCameraHead(const char * path) = 0;

    /// Gets use-null remap.
    DS_NOT_IMPLEMENTED virtual void setUseNullRemap(bool state) = 0;

    /// Sets nominal baseline for this camera in mm (30, 60, 80, 140, 220, 330)
    virtual bool setCameraNominalBaselineThird(int nomBaseline, bool writeThrough = true) = 0;
    /// Gets nominal baseline for this camera in mm (30, 60, 80, 140, 220, 330)
    virtual bool getCameraNominalBaselineThird(int & nomBaselineMM) = 0;

    /// Gets camera known info.
    DS_NOT_IMPLEMENTED virtual bool getCameraInfoKnown() = 0;

    /// Sets camera head contents version.
    virtual bool setCameraHeadContentsVersion(uint32_t version, bool writeThrough = true) = 0;
    /// Gets camera head contents version.
    virtual bool getCameraHeadContentsVersion(uint32_t & version) = 0;
    /// Gets current (latest) camera head contents version number
    virtual bool getCurrentCameraHeadContentsVersion(uint32_t & version) = 0;

    /// Sets (internal) model number.
    virtual bool setModelNumber(uint32_t value, bool writeThrough = true) = 0;
    /// Gets (internal) model number.
    virtual bool getModelNumber(uint32_t & value) = 0;

    /// Sets (internal) model number.
    virtual bool setModuleRevisionNumber(uint32_t value, bool writeThrough = true) = 0;
    /// Gets (internal) model number.
    virtual bool getModuleRevisionNumber(uint32_t & value) = 0;

    /// Sets module model number.
    virtual bool setModuleModelNumber(uint8_t moduleVer, uint8_t moduleMajorVer, uint8_t moduleMinorVer, uint8_t moduleSkewVer, bool writeThrough = true) = 0;
    /// Gets module model number.
    virtual bool getModuleModelNumber(uint8_t & moduleVer, uint8_t & moduleMajorVer, uint8_t & moduleMinorVer, uint8_t & moduleSkewVer) = 0;

    /// Sets camera serial number. The corresponding get method is in DSAPI.
    virtual bool setCameraSerialNumber(uint32_t number, bool writeThrough = true) = 0;

    /// Sets calibration date.
    virtual bool setCalibrationDate(double time, bool writeThrough = true) = 0;
    /// Gets calibration date.
    virtual bool getCalibrationDate(double & time) = 0;

    /// Convert the data.
    DS_NOT_IMPLEMENTED virtual void enableConvertData() = 0;
    /// Check if the data has been converted.
    DS_NOT_IMPLEMENTED virtual bool isConvertDataEnabled() = 0;

    /// Enable the using current time for FILE mode.
    DS_NOT_IMPLEMENTED virtual void enableUsingCurrentTimeForFileMode(bool state) = 0;
    /// Get the using current time for FILE mode.
    DS_NOT_IMPLEMENTED virtual bool isUsingCurrentTimeForFileModeEnabled() = 0;

    /// Write to generic register.
    /// @param regAddress  the register address
    /// @param regValue  the value to set register regAddress to.
    /// @param noCheck if true, do not check whether the write occurred correctly.
    /// @return false on fail else true.
    virtual bool setRegister(uint32_t regAddress, uint32_t regValue, bool noCheck = false) = 0;
    /// Read from generic regiter
    /// @param regAddress  the register address
    /// @param[out] regValue  where to put the value of register regAddress.
    /// @return false on fail else true.
    virtual bool getRegister(uint32_t regAddress, uint32_t & regValue) = 0;

    /// Set OEM ID to camera.
    /// @param id  the OEM ID to set.
    /// @return false on fail else true.
    virtual bool setOEMID(OEMID id, bool writeThrough = true) = 0;

    /// Get OEM ID from camera.
    /// @param id  the OEM ID received.
    /// @return false on fail else true.
    virtual bool getOEMID(OEMID & id) = 0;

    virtual bool setPlatformCameraSupport(uint8_t value, bool writeThrough = true) = 0;
    virtual bool getPlatformCameraSupport(uint8_t & value) = 0;
    virtual bool setPlatformCameraFocus(uint32_t value, bool writeThrough = true) = 0;
    virtual bool setNominalBaselinePlatform(const int32_t value[3], bool writeThrough = true) = 0;
    virtual bool setLensTypePlatform(DSLensType value, bool writeThrough = true) = 0;
    virtual bool setImagerTypePlatform(DSPlatformImagerType value, bool writeThrough = true) = 0;
    virtual bool getNominalBaselinePlatform(int32_t value[3]) = 0;
    virtual bool getLensTypePlatform(DSLensType & value) = 0;
    virtual bool getImagerTypePlatform(DSPlatformImagerType & value) = 0;

    /// Set and Get PRQ
    // @param value is 1 for PRQ and non-PRQ otherwise.
    virtual bool setPRQ(uint8_t value, bool writeThrough = true) = 0;
    virtual bool getPRQ(uint8_t & value) = 0;

    /// Set the times to try and the time in milliseconds between tries when devce is at busy state.
    /// @param tryTimes Times to try when device is at busy state
    /// @param sleepTime Time in milliseconds between tries when devce is at busy state
    /// @return true for success, false on error.
    virtual bool setControlRetry(int tryTimes = 16, int sleepTimeMS = 9) = 0;

    /// Reset the DS4 module.
    ///  After using this API succeessfully, it is mandatory to destroy DSDAPI and recreate DASPI if needed.
    virtual bool reset() = 0;

    /// Set LR auto exposure parameters
    virtual bool setLRAutoExposureParameters(float MeanIntensitySetPoint, float BrightRatioSetPoint, float KPGain, float KPExposure, float KPDarkThreshold, unsigned short ExposureTopEdge, unsigned short ExposureBottomEdge, unsigned short ExposureLeftEdge, unsigned short ExposureRightEdge) = 0;

    /// Get LR auto exposure parameters
    virtual bool getLRAutoExposureParameters(float & MeanIntensitySetPoint, float & BrightRatioSetPoint, float & KPGain, float & KPExposure, float & KPDarkThreshold, unsigned short & ExposureTopEdge, unsigned short & ExposureBottomEdge, unsigned short & ExposureLeftEdge, unsigned short & ExposureRightEdge) = 0;

protected:
    // Creation (and deletion) of an object of this
    // type is supported through the DSFactory functions.
    DSPrivate(){};
    DSPrivate(const DSPrivate & other) DS_DELETED_FUNCTION;
    DSPrivate(DSPrivate && other) DS_DELETED_FUNCTION;
    DSPrivate & operator=(const DSPrivate & other) DS_DELETED_FUNCTION;
    DSPrivate & operator=(DSPrivate && other) DS_DELETED_FUNCTION;
    virtual ~DSPrivate(){};
};
