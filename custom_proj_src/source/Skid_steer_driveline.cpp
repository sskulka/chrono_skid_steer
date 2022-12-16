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
// Simple driveline model. This template can be used to model a FWD, a RWD, or a
// 4WD driveline. It uses a constant front/rear torque split (a value between 0
// and 1 for 4WD, 0 for RWD, and 1 for FWD) and a simple model for a Torsen
// limited-slip differential (front and/or rear).
//
// =============================================================================

#include <cmath>

#include "Skid_steer_driveline.h"
#include "chrono_vehicle/ChSubsysDefs.h"

using namespace chrono;
using namespace chrono::vehicle;

// -----------------------------------------------------------------------------
// Construct a default 4WD simple driveline.
// -----------------------------------------------------------------------------
Skid_steer_driveline::Skid_steer_driveline(const std::string& name) : FMTV_SimpleDriveline(name), m_connected(true) {
    if (m_connected != true) {
        std::cout << "wrongly initialized" << std::endl;
    }
};

//void Skid_Steer_Driveline::Synchronize(double time, const DriverInputs& driver_inputs, double torque) {
//    if (!m_connected)
//        return;
//
//    // Enforce driveshaft speed 
//    double kp = 1.5;
//    double speed_front = 0.5 * (ChSimpleDriveline::m_front_left->GetPos_dt() + ChSimpleDriveline::m_front_right->GetPos_dt());
//    double speed_rear = 0.5 * (ChSimpleDriveline::m_rear_left->GetPos_dt() + ChSimpleDriveline::m_rear_right->GetPos_dt());
//    double alpha = ChSimpleDriveline::GetFrontTorqueFraction();
//    double driveshaft_speed = alpha * speed_front + (1 - alpha) * speed_rear;
//    ChSimpleDriveline::m_driveshaft->SetPos_dt(driveshaft_speed);
//
//    // Split the input torque front/back.
//    double torque_front = torque * ChSimpleDriveline::GetFrontTorqueFraction();
//    double torque_rear = torque - torque_front;
//
//    // Split the axle torques for the corresponding left/right wheels and apply
//    // them to the suspension wheel shafts.
//    double torque_left;
//    double torque_right;
//
//    distributeTorque(torque_front, 4, ChSimpleDriveline::m_front_left->GetPos_dt(), ChSimpleDriveline::m_front_right->GetPos_dt(), torque_left, torque_right);
//
//    ChSimpleDriveline::m_front_left->SetAppliedTorque(-torque_left);
//    ChSimpleDriveline::m_front_right->SetAppliedTorque(-torque_right);
//
//    ChSimpleDriveline::m_rear_left->SetAppliedTorque(-torque_left);
//    ChSimpleDriveline::m_rear_right->SetAppliedTorque(-torque_right);
//    /*differentialSplit(torque_front, GetFrontDifferentialMaxBias(), m_front_left->GetPos_dt(),
//        m_front_right->GetPos_dt(), torque_left, torque_right);
//    m_front_left->SetAppliedTorque(-torque_left);
//    m_front_right->SetAppliedTorque(-torque_right);
//
//    differentialSplit(torque_rear, GetRearDifferentialMaxBias(), m_rear_left->GetPos_dt(), m_rear_right->GetPos_dt(),
//        torque_left, torque_right);
//    m_rear_left->SetAppliedTorque(-torque_left);
//    m_rear_right->SetAppliedTorque(-torque_right);*/
//}
//
//void Skid_Steer_Driveline::distributeTorque(double torque,
//    double max_bias,
//    double speed_left,
//    double speed_right,
//    double& torque_left,
//    double& torque_right) {
//    double diff = std::abs(speed_left - speed_right);
//
//    // The bias grows from 1 at diff=0.25 to max_bias at diff=0.5
//    double bias = 1;
//    if (diff > 0.5)
//        bias = max_bias;
//    else if (diff > 0.25)
//        bias = 4 * (max_bias - 1) * diff + (2 - max_bias);
//
//    // Split torque to the slow and fast wheels.
//    double alpha = bias / (1 + bias);
//    double slow = alpha * torque;
//    double fast = torque - slow;
//
//    if (std::abs(speed_left) < std::abs(speed_right)) {
//        torque_left = slow;
//        torque_right = fast;
//    }
//    else {
//        torque_left = fast;
//        torque_right = slow;
//    }
//}
// end namespace vehicle
// end namespace chrono