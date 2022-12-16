// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2020 projectchrono.org
// All right reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Radu Serban
// =============================================================================
//
// Mechanism for a single-wheel testing rig co-simulated with a tire and a
// terrain system.
//
// The global reference frame has Z up, X towards the front of the vehicle, and
// Y pointing to the left.
//
// =============================================================================

#include <fstream>
#include <algorithm>
#include <set>
#include <vector>

#include "chrono/ChConfig.h"
#include "chrono/utils/ChUtilsCreators.h"

#include "chrono_vehicle/ChVehicleModelData.h"

#include "chrono_vehicle/cosim/mbs/ChVehicleCosimRigNode.h"

#include "chrono_thirdparty/rapidjson/filereadstream.h"
#include "chrono_thirdparty/rapidjson/istreamwrapper.h"

using std::cout;
using std::endl;

using namespace rapidjson;

namespace chrono {
namespace vehicle {

// =============================================================================

ChVehicleCosimRigNode::ChVehicleCosimRigNode()
    : ChVehicleCosimMBSNode(), m_toe_angle(0), m_total_mass(0) {}

ChVehicleCosimRigNode::~ChVehicleCosimRigNode() {}

// -----------------------------------------------------------------------------

void ChVehicleCosimRigNode::InitializeMBS(const std::vector<ChVector<>>& tire_info,
                                          const ChVector2<>& terrain_size,
                                          double terrain_height) {
    assert(m_num_tire_nodes == 1);
    assert(tire_info.size() == 1);
    double tire_mass = tire_info[0].x();
    double tire_radius = tire_info[0].y();
    ////double tire_width = tire_info[0].z();

    // A single-wheel test rig needs a DBP rig
    if (!m_DBP_rig) {
        cout << "\n\nERROR: Single-wheel test rig REQUIRES a drawbar-pull rig!\n\n" << endl;
        MPI_Abort(MPI_COMM_WORLD, 1);
    }

    // Calculate initial rig location and set linear velocity of all rig bodies
    ChVector<> origin(-terrain_size.x() / 2 + 1.5 * tire_radius, 0, terrain_height + tire_radius);

    // Distribute remaining mass (after subtracting tire mass) equally between chassis and spindle.
    if (m_total_mass - tire_mass < 2)
        m_total_mass = tire_mass + 2;
    double body_mass = (m_total_mass - tire_mass) / 2;

    if (m_verbose) {
        cout << "[Rig node    ] total mass = " << m_total_mass << endl;
        cout << "[Rig node    ] tire mass  = " << tire_mass << endl;
        cout << "[Rig node    ] body mass  = " << body_mass << endl;
    }

    // Construct the mechanical system
    ChVector<> chassis_inertia(0.1, 0.1, 0.1);
    ChVector<> upright_inertia(0.1, 0.1, 0.1);
    ChVector<> spindle_inertia(0.1, 0.1, 0.1);

    // Create the chassis body
    m_chassis = chrono_types::make_shared<ChBody>();
    m_chassis->SetMass(body_mass);
    m_chassis->SetInertiaXX(chassis_inertia);
    m_chassis->SetPos(origin);
    m_chassis->SetRot(QUNIT);
    m_chassis->SetPos_dt(VNULL);
    m_system->AddBody(m_chassis);

    // Create the spindle body
    m_spindle = chrono_types::make_shared<ChBody>();
    m_spindle->SetMass(body_mass);
    m_spindle->SetInertiaXX(spindle_inertia);
    m_spindle->SetPos(origin);
    m_spindle->SetRot(QUNIT);
    m_spindle->SetPos_dt(VNULL);
    m_spindle->SetWvel_loc(VNULL);
    m_system->AddBody(m_spindle);

    // Create revolute motor to impose angular speed on the spindle
    m_rev_motor = chrono_types::make_shared<ChLinkMotorRotationSpeed>();
    m_rev_motor->SetMotorFunction(chrono_types::make_shared<ChFunction_Const>(0));
    m_rev_motor->SetName("motor");
    m_rev_motor->Initialize(m_chassis, m_spindle, ChFrame<>(origin, Q_from_AngZ(m_toe_angle) * Q_from_AngX(CH_C_PI_2)));
    m_system->AddLink(m_rev_motor);

    // Write file with rig node settings
    std::ofstream outf;
    outf.open(m_node_out_dir + "/settings.info", std::ios::out);

    outf << "System settings" << endl;
    outf << "   Integration step size = " << m_step_size << endl;
    outf << "Rig body masses" << endl;
    outf << "   total equivalent mass = " << m_total_mass << endl;
    outf << "   individual body mass  = " << body_mass << endl;
    outf << endl;
}

// -----------------------------------------------------------------------------

void ChVehicleCosimRigNode::ApplySpindleForce(unsigned int i, const TerrainForce& spindle_force) {
    assert(i == 0);

    m_spindle->Empty_forces_accumulators();
    m_spindle->Accumulate_force(spindle_force.force, spindle_force.point, false);
    m_spindle->Accumulate_torque(spindle_force.moment, false);
}

BodyState ChVehicleCosimRigNode::GetSpindleState(unsigned int i) const {
    BodyState state;

    state.pos = m_spindle->GetPos();
    state.rot = m_spindle->GetRot();
    state.lin_vel = m_spindle->GetPos_dt();
    state.ang_vel = m_spindle->GetWvel_par();
    
    return state;
}

// -----------------------------------------------------------------------------

void ChVehicleCosimRigNode::OnInitializeDBPRig(std::shared_ptr<ChFunction> func) {
    m_rev_motor->SetMotorFunction(func);
}

// -----------------------------------------------------------------------------

void ChVehicleCosimRigNode::OnOutputData(int frame) {
    // Append to results output file
    if (m_outf.is_open()) {
        std::string del("  ");

        const ChVector<>& chassis_pos = m_chassis->GetPos();
        const ChVector<>& spindle_pos = m_spindle->GetPos();
        const ChVector<>& spindle_vel = m_spindle->GetPos_dt();
        const ChVector<>& spindle_angvel = m_spindle->GetWvel_loc();

        const ChVector<>& rfrc_motor = m_rev_motor->Get_react_force();
        const ChVector<>& rtrq_motor = m_rev_motor->GetMotorTorque();

        m_outf << m_system->GetChTime() << del;
        // Body states
        m_outf << spindle_pos.x() << del << spindle_pos.y() << del << spindle_pos.z() << del;
        m_outf << spindle_vel.x() << del << spindle_vel.y() << del << spindle_vel.z() << del;
        m_outf << spindle_angvel.x() << del << spindle_angvel.y() << del << spindle_angvel.z() << del;
        m_outf << chassis_pos.x() << del << chassis_pos.y() << del << chassis_pos.z() << del;
        // Joint reactions
        m_outf << rfrc_motor.x() << del << rfrc_motor.y() << del << rfrc_motor.z() << del;
        m_outf << rtrq_motor.x() << del << rtrq_motor.y() << del << rtrq_motor.z() << del;
        // Solver statistics (for last integration step)
        m_outf << m_system->GetTimerStep() << del << m_system->GetTimerLSsetup() << del << m_system->GetTimerLSsolve()
               << del << m_system->GetTimerUpdate() << del;
        if (m_int_type == ChTimestepper::Type::HHT) {
            m_outf << m_integrator->GetNumIterations() << del << m_integrator->GetNumSetupCalls() << del
                   << m_integrator->GetNumSolveCalls() << del;
        }
        m_outf << endl;
    }

    // Create and write frame output file.
    utils::CSV_writer csv(" ");
    csv << m_system->GetChTime() << endl;  // current time
    csv << m_chassis->GetIdentifier() << m_chassis->GetPos() << m_chassis->GetRot() << m_chassis->GetPos_dt()
        << m_chassis->GetRot_dt() << endl;
    csv << m_spindle->GetIdentifier() << m_spindle->GetPos() << m_spindle->GetRot() << m_spindle->GetPos_dt()
        << m_spindle->GetRot_dt() << endl;

    std::string filename = OutputFilename(m_node_out_dir, "data", "dat", frame + 1, 5);
    csv.write_to_file(filename);

    if (m_verbose)
        cout << "[Rig node    ] write output file ==> " << filename << endl;
}

}  // end namespace vehicle
}  // end namespace chrono
