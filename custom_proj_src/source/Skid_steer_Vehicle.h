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
#pragma once
#ifndef SKDSTR_VEHICLE_H
#define SKDSTR_VEHICLE_H

#include "chrono_vehicle/wheeled_vehicle/ChWheeledVehicle.h"
#include "chrono_vehicle/ChPowertrain.h"
#include "chrono_vehicle/powertrain/SimpleMapPowertrain.h"
#include "chrono_vehicle/ChSubsysDefs.h"
#include "Skid_steer_RigidPinnedAxle.h"
#include "chrono_models/vehicle/generic/Generic_RigidPinnedAxle.h"
#include "chrono_models/vehicle/generic/Generic_SimpleMapPowertrain.h"


namespace chrono {
    namespace vehicle {

        class Skid_steer_Vehicle : public chrono::vehicle::ChWheeledVehicle {
        public:
            Skid_steer_Vehicle(const bool fixed, chrono::ChContactMethod contactMethod = chrono::ChContactMethod::NSC);

            ~Skid_steer_Vehicle() {};

            virtual int GetNumberAxles() const override { return 2; };

            virtual double GetWheelbase() const override { return 1.0; };
            virtual double GetMinTurningRadius() const override { return 5.0; };
            virtual double GetMaxSteeringAngle() const override { return 0; };

            void Skid_steer_Vehicle::Initialize(const chrono::ChCoordsys<>& chassisPos, double chassisFwdVel = 0) override;

            void Skid_steer_Vehicle::Synchronize(double time, const DriverInputs& driver_inputs, const ChTerrain& terrain) override;

            void Skid_steer_Vehicle::InitializePowertrain(std::shared_ptr<generic::Generic_SimpleMapPowertrain> powertrain_l, std::shared_ptr<generic::Generic_SimpleMapPowertrain> powertrain_r);
           //std::shared_ptr<SimpleMapPowertrain> m_powertrain;
           //std::shared_ptr<Skid_Steer_Driveline> m_driveline;  ///< driveline subsystem
            std::shared_ptr<generic::Generic_SimpleMapPowertrain> m_powertrain; //left
            std::shared_ptr<generic::Generic_SimpleMapPowertrain> m_powertrain_r; //right
            std::shared_ptr<ChDrivelineWV> m_driveline_r;
        };
    }
}

#endif