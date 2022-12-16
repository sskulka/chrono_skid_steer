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
// Authors: Radu Serban
// =============================================================================
//
// Generic concrete rigid suspension subsystem.
//
// This concrete suspension subsystem is defined with respect to a right-handed
// frame with X pointing towards the front, Y to the left, and Z up (as imposed
// by the base class ChRigidSuspension) and origin in the chassis midplane.
//
// All point locations are provided for the left half of the suspension.
//
// =============================================================================

#include "Skid_steer_RigidPinnedAxle.h"

using namespace chrono;
using namespace chrono::vehicle;
using namespace chrono::vehicle::skdstr_rigidpinnedaxle;


// Static variables -- all in SI units
const double Skid_Steer_RigidPinnedAxle::m_spindleMass = 1.103;
const double Skid_Steer_RigidPinnedAxle::m_spindleRadius = 0.15;
const double Skid_Steer_RigidPinnedAxle::m_spindleWidth = 0.06;
const ChVector<> Skid_Steer_RigidPinnedAxle::m_spindleInertia(0.000478, 0.000496, 0.000478);
const double Skid_Steer_RigidPinnedAxle::m_axleInertia = 0.4;

Skid_Steer_RigidPinnedAxle::Skid_Steer_RigidPinnedAxle(const std::string& name) : ChRigidPinnedAxle(name) {}

const ChVector<> Skid_Steer_RigidPinnedAxle::getLocation(PointId which) {
    switch (which) {
    case SPINDLE:
        return ChVector<>(0, 1.100, 0);  // location of spindle center of mass
    default:
        return ChVector<>(0, 0, 0);
    }
}