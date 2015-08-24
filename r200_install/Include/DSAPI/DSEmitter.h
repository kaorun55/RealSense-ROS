/*******************************************************************************

INTEL CORPORATION PROPRIETARY INFORMATION
This software is supplied under the terms of a license agreement or nondisclosure
agreement with Intel Corporation and may not be copied or disclosed except in
accordance with the terms of that agreement
Copyright(c) 2014-2015 Intel Corporation. All Rights Reserved.

*******************************************************************************/

#pragma once

#include "DSAPITypes.h"

/// @class DSEmitter
/// Defines methods specific to an implementation that has an emitter.
class DSEmitter
{
public:
    /// Turn emitter on or off.  When Z is enabled, emitter is on by default.
    /// @param enable if true, turn on emitter during streaming, if false, turn off.
    /// @return true on no error
    virtual bool enableEmitter(bool enable) = 0;

    /// Get the emitter status.  Note: Emitter is never on before streaming starts. When called before streaming starts, emitter status indicates
    /// whether the emitter will be on or off when streaming. Known issue: current the emitter takes .5sec to turn on - during which the status is false.
    /// @param enabled when called during streaming, will return true if the emitter is on, false if it is off.
    /// @return true if succeeded and false otherwise
    virtual bool getEmitterStatus(bool & enabled) = 0;

protected:
    // Creation (and deletion) of an object of this
    // type is supported through the DSFactory functions.
    DSEmitter(){};
    DSEmitter(const DSEmitter & other) DS_DELETED_FUNCTION;
    DSEmitter(DSEmitter && other) DS_DELETED_FUNCTION;
    DSEmitter & operator=(const DSEmitter & other) DS_DELETED_FUNCTION;
    DSEmitter & operator=(DSEmitter && other) DS_DELETED_FUNCTION;
    virtual ~DSEmitter(){};
};
