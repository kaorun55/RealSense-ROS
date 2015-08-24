/*******************************************************************************

INTEL CORPORATION PROPRIETARY INFORMATION
This software is supplied under the terms of a license agreement or nondisclosure
agreement with Intel Corporation and may not be copied or disclosed except in
accordance with the terms of that agreement
Copyright(c) 2014-2015 Intel Corporation. All Rights Reserved.

*******************************************************************************/

#pragma once

#include "DSAPITypes.h"

class DSAPI;

/// Create an instance of DSAPI
/// @param p the requested runtime platform (DS4 device, DS5 device, or playback from a recording)
/// @return new DSAPI instance of the requested platform type
/// @see DSDestroy
DS_DECL DSAPI * DSCreate(DSPlatform p);

/// Destroys an instance of the DSAPI previously returned by DSCreate(...)
/// @param ds the DSAPI instance to destroy
/// @see DSCreate
DS_DECL void DSDestroy(DSAPI * ds);

/// Static API to get the number of DS cameras on the system.
/// @param refresh Force to refresh the camera list.
/// @return The number of DS cameras on the system.r.
DS_DECL int DSGetNumberOfCameras(bool refresh = false);

/// Static API to get the serial number for camera with specified index.
/// @param cameraIndex A zero based integer, up to the number of DS cameras on the system.
/// @return DS camera serial number.
DS_DECL uint32_t DSGetCameraSerialNumber(int cameraIndex);
