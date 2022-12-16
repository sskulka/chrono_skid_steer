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
// Generic vehicle model with an articulated chassis.
//
// =============================================================================

#include "chrono/assets/ChSphereShape.h"
#include "chrono/assets/ChTriangleMeshShape.h"
#include "chrono/utils/ChUtilsInputOutput.h"
#include "chrono_vehicle/ChPowertrain.h"
#include "chrono_vehicle/ChVehicleModelData.h"

#include "Skid_steer_Vehicle.h"
#include "ACV_ChassisFront.h"
#include "ACV_ChassisRear.h"
#include "ACV_ChassisConnector.h"
#include "ACV_RigidSuspension.h"
#include "ACV_Wheel.h"
#include "ACV_Driveline2WD.h"
#include "ACV_BrakeSimple.h"
//#include <ChRigidPinnedAxle.h>
#include "chrono_vehicle/wheeled_vehicle/suspension/ChRigidPinnedAxle.h"
#include "Skid_steer_driveline.h"
#include "chrono_vehicle/wheeled_vehicle/driveline/ChSimpleDriveline.h"
//#include "chrono_models/vehicle/generic/Generic_RigidPinnedAxle.h"


using namespace chrono;
using namespace chrono::vehicle;
        // -----------------------------------------------------------------------------
        // -----------------------------------------------------------------------------
        Skid_steer_Vehicle::Skid_steer_Vehicle(const bool fixed, ChContactMethod contactMethod)
            : ChWheeledVehicle("GenericWV", contactMethod) {
            // Create the front and rear chassis subsystems
            m_chassis = chrono_types::make_shared<ACV_ChassisFront>("ChassisFront", fixed);
            m_chassis_rear.resize(1);
            m_chassis_rear[0] = chrono_types::make_shared<ACV_ChassisRear>("ChassisRear");

            // Create the actuated articulation between front and rear chassis
            m_chassis_connectors.resize(1);
            m_chassis_connectors[0] = chrono_types::make_shared<ACV_ChassisConnector>("ChassisConnector");

            // Create the axle subsystems (suspension + wheels + brakes)
            m_axles.resize(2);

            m_axles[0] = chrono_types::make_shared<ChAxle>();
            //m_axles[0]->m_suspension = chrono_types::make_shared<ChRigidSuspension>("FrontSusp");
            m_axles[0]->m_suspension = chrono_types::make_shared<ACV_RigidSuspension>("FrontSusp");
            m_axles[0]->m_wheels.resize(2);
            m_axles[0]->m_wheels[0] = chrono_types::make_shared<ACV_Wheel>("Wheel_FL");
            m_axles[0]->m_wheels[1] = chrono_types::make_shared<ACV_Wheel>("Wheel_FR");
            m_axles[0]->m_brake_left = chrono_types::make_shared<ACV_BrakeSimple>("Brake_FL");
            m_axles[0]->m_brake_right = chrono_types::make_shared<ACV_BrakeSimple>("Brake_FR");

            m_axles[1] = chrono_types::make_shared<ChAxle>();
            m_axles[1]->m_suspension = chrono_types::make_shared<generic::Generic_RigidPinnedAxle>("RearSusp");
            m_axles[1]->m_wheels.resize(2);
            m_axles[1]->m_wheels[0] = chrono_types::make_shared<ACV_Wheel>("Wheel_RL");
            m_axles[1]->m_wheels[1] = chrono_types::make_shared<ACV_Wheel>("Wheel_RR");
            m_axles[1]->m_brake_left = chrono_types::make_shared<ACV_BrakeSimple>("Brake_RL");
            m_axles[1]->m_brake_right = chrono_types::make_shared<ACV_BrakeSimple>("Brake_RR");

            // Create the driveline
            m_driveline = chrono_types::make_shared<Skid_steer_driveline>("Driveline_l");
            m_driveline_r = chrono_types::make_shared<Skid_steer_driveline>("Driveline_r");
        }

        // -----------------------------------------------------------------------------
        // -----------------------------------------------------------------------------
        void Skid_steer_Vehicle::Initialize(const ChCoordsys<>& chassisPos, double chassisFwdVel) {
            // Initialize the chassis subsystem
           // GetLog() << "Skid_steer_vehicle initialize 1" << "\n\n";
            m_chassis->Initialize(m_system, chassisPos, chassisFwdVel, WheeledCollisionFamily::CHASSIS);
           // GetLog() << "Skid_steer_vehicle initialize 2" << "\n\n";
            m_chassis_rear[0]->Initialize(m_chassis, WheeledCollisionFamily::CHASSIS);
           // GetLog() << "Skid_steer_vehicle initialize 3" << "\n\n";

            // Initialize the connection between front and rear chassis
            m_chassis_connectors[0]->Initialize(m_chassis, m_chassis_rear[0]);
           // GetLog() << "Skid_steer_vehicle initialize 4" << "\n\n";

            // Initialize the axle subsystems
            m_axles[0]->Initialize(m_chassis, nullptr, nullptr, ChVector<>(0.5, 0, 0), ChVector<>(0, 0, 0.0), 0.0);

           // GetLog() << "Skid_steer_vehicle initialize 5" << "\n\n";
            m_axles[1]->Initialize(m_chassis_rear[0], nullptr, nullptr, ChVector<>(-0.5, 0, 0), ChVector<>(0, 0, 0), 0.0);
           // GetLog() << "Skid_steer_vehicle initialize 6" << "\n\n";

            // Initialize the driveline subsystem (RWD)
            std::vector<int> driven_susp = { 0,1 };

            GetLog() << "Skid_steer_vehicle initialize 7" << "\n\n";
            m_driveline->Initialize(m_chassis, m_axles, driven_susp);
            std::string lDriveLinename = m_driveline->GetName();
            GetLog() << "left driveline name is" << lDriveLinename<< "\n\n";
            m_driveline_r->Initialize(m_chassis, m_axles, driven_susp); 
            std::string rDriveLinename = m_driveline_r->GetName();
            GetLog() << "right driveline name is" << rDriveLinename << "\n\n";

            GetLog() << "Skid_steer_vehicle initialize 8" << "\n\n";
            // Invoke base class method
            ChWheeledVehicle::Initialize(chassisPos, chassisFwdVel);

            GetLog() << "Skid_steer_vehicle initialize 9" << "\n\n";
            getchar();
        }

        void Skid_steer_Vehicle::Synchronize(double time, const DriverInputs& driver_inputs, const ChTerrain& terrain) {
            double powertrain_torque_l = 0;
            double powertrain_torque_r = 0;
            //if (ChWheeledVehicle::m_powertrain && m_driveline) {
            if (m_powertrain && m_driveline) {
                // Synchronize the associated powertrain system (pass throttle input).
                m_powertrain->Synchronize(time, driver_inputs, m_driveline->GetDriveshaft()->GetPos_dt());
                // Extract the torque from the powertrain.
                //powertrain_torque = ChWheeledVehicle::m_powertrain->GetOutputTorque();
                powertrain_torque_l = m_powertrain->GetOutputTorque();
            }
            // Apply powertrain torque to the driveline's input shaft.
            if (m_driveline) {
                powertrain_torque_l = powertrain_torque_l - 10;
                m_driveline->Synchronize(time, driver_inputs, powertrain_torque_l);
                GetLog() << "left side input torque is: " << powertrain_torque_l << "\n\n";
                m_driveline->GetSpindleTorque
            }

            if (m_powertrain_r && m_driveline_r) {
                // Synchronize the associated powertrain system (pass throttle input).
                m_powertrain_r->Synchronize(time, driver_inputs, m_driveline_r->GetDriveshaft()->GetPos_dt());
                // Extract the torque from the powertrain.
                //powertrain_torque = ChWheeledVehicle::m_powertrain->GetOutputTorque();
                powertrain_torque_r = m_powertrain_r->GetOutputTorque();
            }
            // Apply powertrain torque to the driveline's input shaft.
            if (m_driveline_r)

                //GetLog() << "right side driveline name: " << m_powertrain_r->m_driveline->GetName() << "\n\n";
                //GetLog() << "right side driveline exists" << "\n\n";
                m_driveline_r->Synchronize(time, driver_inputs, powertrain_torque_r);
                GetLog() << "right side input torque is: " << powertrain_torque_r << "\n\n";
        }

        void Skid_steer_Vehicle::InitializePowertrain(std::shared_ptr<generic::Generic_SimpleMapPowertrain> powertrain_l, std::shared_ptr<generic::Generic_SimpleMapPowertrain> powertrain_r) {
            m_powertrain = NULL; //left powertrain 
            m_powertrain_r = NULL; //left powertrain 
            m_powertrain = powertrain_l;
            m_powertrain_r = powertrain_r;
            if (!m_powertrain->m_driveline) {
                m_powertrain->m_driveline = m_driveline;
            }
            if (!m_powertrain_r->m_driveline) {
                m_powertrain_r->m_driveline = m_driveline_r;

            }
            m_powertrain->Initialize(m_chassis);
            m_powertrain_r->Initialize(m_chassis);
            //ChWheeledVehicle::InitializePowertrain(powertrain);
        }
    