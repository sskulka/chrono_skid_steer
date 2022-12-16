#pragma once
#pragma once
// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Shubhankar Kulkarni
// =============================================================================
//
// Vehicle model with an articulated chassis.
//
// =============================================================================
#ifndef SKDSTR_DRVLINE_H
#define SKDSTR_DRVLINE_H

#include "chrono_vehicle/wheeled_vehicle/ChWheeledVehicle.h"
//#include <ChSimpleDriveline.h>
#include "chrono_vehicle/wheeled_vehicle/driveline/ChSimpleDriveline.h"
#include "chrono_models/vehicle/mtv/FMTV_SimpleDriveline.h"
#include "chrono_vehicle/ChSubsysDefs.h"

using namespace chrono;
using namespace chrono::vehicle;

class Skid_steer_driveline : public chrono::vehicle::fmtv::FMTV_SimpleDriveline {
public:
	Skid_steer_driveline(const std::string& name);

	~Skid_steer_driveline() {};

	/// Update the driveline subsystem: apply the specified motor torque.
	/// This represents the input to the driveline subsystem from the powertrain system.

private:
	bool m_connected;
};
#endif