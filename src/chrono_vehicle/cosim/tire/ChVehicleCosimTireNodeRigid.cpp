// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2021 projectchrono.org
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
// Definition of the vehicle co-simulation rigid TIRE NODE class.
// This type of tire communicates with the terrain node through a BODY
// communication interface.
//
// The global reference frame has Z up, X towards the front of the vehicle, and
// Y pointing to the left.
//
// =============================================================================

#include "chrono_vehicle/cosim/tire/ChVehicleCosimTireNodeRigid.h"
#include "chrono/utils/ChUtilsInputOutput.h"

using std::cout;
using std::endl;

namespace chrono {
namespace vehicle {

ChVehicleCosimTireNodeRigid::ChVehicleCosimTireNodeRigid(int index) : ChVehicleCosimTireNode(index) {}

void ChVehicleCosimTireNodeRigid::ConstructTire() {
    m_tire = chrono_types::make_shared<RigidTire>(m_tire_json);
    assert(m_tire->UseContactMesh());
}

void ChVehicleCosimTireNodeRigid::InitializeTire(std::shared_ptr<ChWheel> wheel) {
    // Initialize the rigid tire
    wheel->SetTire(m_tire);                                       
    std::static_pointer_cast<ChTire>(m_tire)->Initialize(wheel);  // hack to call protected virtual method
    m_tire->SetVisualizationType(VisualizationType::MESH);

    // Set mesh data (vertex positions in local frame)
    m_mesh_data.nv = m_tire->GetNumVertices();
    m_mesh_data.nn = m_tire->GetNumNormals();
    m_mesh_data.nt = m_tire->GetNumTriangles();
    m_mesh_data.verts = m_tire->GetMeshVertices();
    m_mesh_data.norms = m_tire->GetMeshNormals();
    m_mesh_data.idx_verts = m_tire->GetMeshConnectivity();
    m_mesh_data.idx_norms = m_tire->GetMeshNormalIndices();

    // Tire contact material
    m_contact_mat = std::static_pointer_cast<ChMaterialSurfaceSMC>(m_tire->GetContactMaterial());

    // Preprocess the tire mesh and store neighbor element information for each vertex.
    // Calculate mesh triangle areas.
    m_adjElements.resize(m_mesh_data.nv);
    std::vector<double> triArea(m_mesh_data.nt);
    for (unsigned int ie = 0; ie < m_mesh_data.nt; ie++) {
        int iv1 = m_mesh_data.idx_verts[ie].x();
        int iv2 = m_mesh_data.idx_verts[ie].y();
        int iv3 = m_mesh_data.idx_verts[ie].z();
        ChVector<> v1 = m_mesh_data.verts[iv1];
        ChVector<> v2 = m_mesh_data.verts[iv2];
        ChVector<> v3 = m_mesh_data.verts[iv3];
        triArea[ie] = 0.5 * Vcross(v2 - v1, v3 - v1).Length();
        m_adjElements[iv1].push_back(ie);
        m_adjElements[iv2].push_back(ie);
        m_adjElements[iv3].push_back(ie);
    }

    // Preprocess the tire mesh and store representative area for each vertex.
    m_vertexArea.resize(m_tire->GetNumVertices());
    for (unsigned int in = 0; in < m_tire->GetNumVertices(); in++) {
        double area = 0;
        for (unsigned int ie = 0; ie < m_adjElements[in].size(); ie++) {
            area += triArea[m_adjElements[in][ie]];
        }
        m_vertexArea[in] = area / m_adjElements[in].size();
    }
}

void ChVehicleCosimTireNodeRigid::ApplySpindleState(const BodyState& spindle_state) {
    m_spindle->SetPos(spindle_state.pos);
    m_spindle->SetRot(spindle_state.rot);
    m_spindle->SetPos_dt(spindle_state.lin_vel);
    m_spindle->SetWvel_par(spindle_state.ang_vel);
}

void ChVehicleCosimTireNodeRigid::ApplySpindleForce(const TerrainForce& spindle_force) {
    // Cache spindle force for reporting
    m_force = spindle_force;
}

void ChVehicleCosimTireNodeRigid::OnOutputData(int frame) {
    // Append to results output file
    if (m_outf.is_open()) {
        std::string del("  ");

        m_outf << frame << del;
        m_outf << m_force.point.x() << del << m_force.point.y() << del << m_force.point.z() << del;
        m_outf << m_force.force.x() << del << m_force.force.y() << del << m_force.force.z() << del;
        m_outf << m_force.moment.x() << del << m_force.moment.y() << del << m_force.moment.z() << del;
        m_outf << endl;
    }

    // Create and write frame output file.
    utils::CSV_writer csv(" ");
    WriteTireStateInformation(csv);
    WriteTireMeshInformation(csv);

    std::string filename = OutputFilename(m_node_out_dir + "/simulation", "data", "dat", frame + 1, 5);
    csv.write_to_file(filename);

    if (m_verbose)
        cout << "[Tire node   ] write output file ==> " << filename << endl;
}

void ChVehicleCosimTireNodeRigid::WriteTireStateInformation(utils::CSV_writer& csv) {
    // Write number of vertices
    unsigned int num_vertices = m_tire->GetNumVertices();
    csv << num_vertices << endl;

    // Write mesh vertex positions and velocities
    std::vector<ChVector<>> pos;
    std::vector<ChVector<>> vel;
    m_tire->GetMeshVertexStates(pos, vel);
    for (unsigned int in = 0; in < num_vertices; in++)
        csv << pos[in] << endl;
    for (unsigned int in = 0; in < num_vertices; in++)
        csv << vel[in] << endl;
}

void ChVehicleCosimTireNodeRigid::WriteTireMeshInformation(utils::CSV_writer& csv) {
    // Print tire mesh connectivity
    csv << m_tire->GetNumTriangles() << endl;

    const std::vector<ChVector<int>>& triangles = m_tire->GetMeshConnectivity();
    for (unsigned int ie = 0; ie < m_tire->GetNumTriangles(); ie++) {
        csv << triangles[ie] << endl;
    }
}

void ChVehicleCosimTireNodeRigid::OutputVisualizationData(int frame) {
    auto filename = OutputFilename(m_node_out_dir + "/visualization", "vis", "dat", frame, 5);
    utils::WriteVisualizationAssets(m_system, filename, true);
}

}  // namespace vehicle
}  // namespace chrono
