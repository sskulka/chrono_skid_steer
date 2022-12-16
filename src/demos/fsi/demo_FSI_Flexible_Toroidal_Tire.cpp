// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All right reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Author: Pei Li, Wei Hu
// =============================================================================

#include <assert.h>
#include <stdlib.h>
#include <ctime>

#include "chrono/physics/ChSystemSMC.h"

#ifdef CHRONO_PARDISO_MKL
    #include "chrono_pardisomkl/ChSolverPardisoMKL.h"
#endif

#include "chrono/solver/ChIterativeSolverLS.h"
#include "chrono/utils/ChUtilsCreators.h"
#include "chrono/utils/ChUtilsGenerators.h"
#include "chrono/utils/ChUtilsGeometry.h"

#include "chrono_fsi/ChSystemFsi.h"
#include "chrono_fsi/ChVisualizationFsi.h"

#include "chrono/fea/ChElementShellANCF_3423.h"
#include "chrono/fea/ChLinkDirFrame.h"
#include "chrono/fea/ChLinkPointFrame.h"
#include "chrono/fea/ChMesh.h"
#include "chrono/fea/ChMeshExporter.h"
#include "chrono/fea/ChBuilderBeam.h"

#include "chrono_thirdparty/filesystem/path.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/physics/ChLinkMotorRotationAngle.h"
#include "chrono/utils/ChUtilsGeometry.h"
#include "chrono_thirdparty/filesystem/path.h"

// Chrono namespaces
using namespace chrono;
using namespace chrono::fea;
using namespace chrono::collision;
using namespace chrono::fsi;

// Set the output directory
const std::string out_dir = GetChronoOutputPath() + "FSI_Flexible_Toroidal_Tire/";
std::string MESH_CONNECTIVITY = out_dir + "Flex_MESH.vtk";

// Dimension of the domain
double smalldis = 1.0e-9;
double bxDim = 5.0 + smalldis;
double byDim = 0.6 + smalldis;
double bzDim = 0.3 + smalldis;

// Dimension of the fluid domain
double fxDim = 5.0 + smalldis;
double fyDim = 0.6 + smalldis;
double fzDim = 0.2 + smalldis;
bool flexible_elem_1D = false;

// Size of the wheel
double wheel_radius = 0.35;
double wheel_slip = 0.0;
double wheel_AngVel = 1.0;
double total_mass = 105.22;

double m_rim_radius = 0.35;
double m_height = 0.195;
double m_thickness = 0.014;
int m_div_circumference = 60;
int m_div_width = 12;
double m_alpha = 0.15;

// Initial Position of wheel
ChVector<> wheel_IniPos(-bxDim / 2 + 1.5 * wheel_radius, 0.0, 1.5 * wheel_radius + bzDim);
ChVector<> wheel_IniVel(0.0, 0.0, 0.0);

// Simulation time and stepsize
double t_end = 10.0;
double dT = 2.5e-4;

// Output frequency
bool output = true;
double out_fps = 20;

// Enable/disable run-time visualization (if Chrono::OpenGL is available)
bool render = true;
float render_fps = 100;

// linear actuator and angular actuator
auto actuator = chrono_types::make_shared<ChLinkLinActuator>();
auto motor = chrono_types::make_shared<ChLinkMotorRotationAngle>();

std::vector<std::vector<int>> NodeNeighborElement_mesh;

void Create_MB_FE(ChSystemSMC& sysMBS, ChSystemFsi& sysFSI);

int main(int argc, char* argv[]) {
    // Create oputput directories
    if (!filesystem::create_directory(filesystem::path(out_dir))) {
        std::cerr << "Error creating directory " << out_dir << std::endl;
        return 1;
    }
    if (!filesystem::create_directory(filesystem::path(out_dir + "/particles"))) {
        std::cerr << "Error creating directory " << out_dir + "/particles" << std::endl;
        return 1;
    }
    if (!filesystem::create_directory(filesystem::path(out_dir + "/vtk"))) {
        std::cerr << "Error creating directory " << out_dir + "/vtk" << std::endl;
        return 1;
    }

    // Create a physics system and an FSI system
    ChSystemSMC sysMBS;
    ChSystemFsi sysFSI(sysMBS);

    // Use the default input file or you may enter your input parameters as a command line argument
    std::string inputJson = GetChronoDataFile("fsi/input_json/demo_FSI_Flexible_Elements_Explicit.json");
    if (argc == 1) {
        std::cout << "Use the default JSON file" << std::endl;
    } else if (argc == 2) {
        std::cout << "Use the specified JSON file" << std::endl;
        std::string my_inputJson = std::string(argv[1]);
        inputJson = my_inputJson;
    } else {
        std::cout << "usage: ./demo_FSI_Flexible_Elements <json_file>" << std::endl;
        return 1;
    }
    sysFSI.ReadParametersFromFile(inputJson);

    sysFSI.SetContainerDim(ChVector<>(bxDim, byDim, bzDim));

    auto initSpace0 = sysFSI.GetInitialSpacing();
    ChVector<> cMin = ChVector<>(-5 * bxDim, -byDim / 2.0 - initSpace0 / 2.0, -5 * bzDim );
    ChVector<> cMax = ChVector<>( 5 * bxDim,  byDim / 2.0 + initSpace0 / 2.0,  10 * bzDim );
    sysFSI.SetBoundaries(cMin, cMax);

    // Setup the output directory for FSI data
    sysFSI.SetOutputDirectory(out_dir);

    // Create SPH particles of fluid region
    chrono::utils::GridSampler<> sampler(initSpace0);
    ChVector<> boxCenter(-bxDim / 2 + fxDim / 2, 0, fzDim / 2 + 1 * initSpace0);
    ChVector<> boxHalfDim(fxDim / 2, fyDim / 2, fzDim / 2);
    chrono::utils::Generator::PointVector points = sampler.SampleBox(boxCenter, boxHalfDim);
    size_t numPart = points.size();
    for (int i = 0; i < numPart; i++) {
        sysFSI.AddSPHParticle(points[i]);
    }

    // Create solids
    Create_MB_FE(sysMBS, sysFSI);
    sysFSI.Initialize();
    auto my_mesh = sysFSI.GetFsiMesh();

    // Create a run-tme visualizer
    ChVisualizationFsi fsi_vis(&sysFSI);
    if (render) {
        fsi_vis.SetTitle("Chrono::FSI flexible element demo");
        fsi_vis.SetCameraPosition(ChVector<>(bxDim / 8, -3, 0.25), ChVector<>(bxDim / 8, 0.0, 0.25));
        fsi_vis.SetCameraMoveScale(1.0f);
        fsi_vis.EnableBoundaryMarkers(false);
        fsi_vis.Initialize();
    }

    // Set MBS solver
#ifdef CHRONO_PARDISO_MKL
    auto mkl_solver = chrono_types::make_shared<ChSolverPardisoMKL>();
    mkl_solver->LockSparsityPattern(true);
    sysMBS.SetSolver(mkl_solver);
#else
    auto solver = chrono_types::make_shared<ChSolverMINRES>();
    sysMBS.SetSolver(solver);
    solver->SetMaxIterations(2000);
    solver->SetTolerance(1e-10);
    solver->EnableDiagonalPreconditioner(true);
    solver->SetVerbose(false);
    sysMBS.SetSolverForceTolerance(1e-10);
#endif

    // Simulation loop
    double dT = sysFSI.GetStepSize();

    unsigned int output_steps = (unsigned int)(1 / (out_fps * dT));
    unsigned int render_steps = (unsigned int)(1 / (render_fps * dT));

    double time = 0.0;
    int current_step = 0;

    ChTimer<> timer;
    timer.start();
    while (time < t_end) {
        std::cout << current_step << " time: " << time << std::endl;

        if (output && current_step % output_steps == 0) {
            std::cout << "-------- Output" << std::endl;
            sysFSI.PrintParticleToFile(out_dir + "/particles");
            static int counter = 0;
            std::string filename = out_dir + "/vtk/flex_body." + std::to_string(counter++) + ".vtk";
            fea::ChMeshExporter::writeFrame(my_mesh, (char*)filename.c_str(), MESH_CONNECTIVITY);
        }

        // Render SPH particles
        if (render && current_step % render_steps == 0) {
            if (!fsi_vis.Render())
                break;
        }

        sysFSI.DoStepDynamics_FSI();

        time += dT;
        current_step++;
    }
    timer.stop();
    std::cout << "\nSimulation time: " << timer() << " seconds\n" << std::endl;

    return 0;
}

//--------------------------------------------------------------------
// Create the objects of the MBD system. Rigid/flexible bodies, and if 
// fsi, their bce representation are created and added to the systems
void Create_MB_FE(ChSystemSMC& sysMBS, ChSystemFsi& sysFSI) {
    sysMBS.Set_G_acc(ChVector<>(0, 0, -9.81));
    sysFSI.Set_G_acc(ChVector<>(0, 0, -9.81));
    
    // Set common material Properties
    auto mysurfmaterial = chrono_types::make_shared<ChMaterialSurfaceSMC>();
    mysurfmaterial->SetYoungModulus(6e4);
    mysurfmaterial->SetFriction(0.3f);
    mysurfmaterial->SetRestitution(0.2f);
    mysurfmaterial->SetAdhesion(0);

    auto ground = chrono_types::make_shared<ChBody>();
    ground->SetIdentifier(-1);
    ground->SetBodyFixed(true);
    ground->SetCollide(true);

    ground->GetCollisionModel()->ClearModel();
    auto initSpace0 = sysFSI.GetInitialSpacing();

    // Bottom and top wall
    ChVector<> size_XY(bxDim / 2 + 3 * initSpace0, byDim / 2 + 3 * initSpace0, 2 * initSpace0);
    ChVector<> pos_zp(0, 0, bzDim + 2 * initSpace0);
    ChVector<> pos_zn(0, 0, -2 * initSpace0);

    // left and right Wall
    ChVector<> size_YZ(2 * initSpace0, byDim / 2 + 3 * initSpace0, bzDim / 2);
    ChVector<> pos_xp(bxDim / 2 + initSpace0, 0.0, bzDim / 2 + 1 * initSpace0);
    ChVector<> pos_xn(-bxDim / 2 - 3 * initSpace0, 0.0, bzDim / 2 + 1 * initSpace0);

    // Front and back Wall
    ChVector<> size_XZ(bxDim / 2, 2 * initSpace0, bzDim / 2);
    ChVector<> pos_yp(0, byDim / 2 + initSpace0, bzDim / 2 + 1 * initSpace0);
    ChVector<> pos_yn(0, -byDim / 2 - 3 * initSpace0, bzDim / 2 + 1 * initSpace0);

    // MBD representation of walls
    chrono::utils::AddBoxGeometry(ground.get(), mysurfmaterial, size_XY, pos_zn, QUNIT, true);
    chrono::utils::AddBoxGeometry(ground.get(), mysurfmaterial, size_YZ, pos_xp, QUNIT, true);
    chrono::utils::AddBoxGeometry(ground.get(), mysurfmaterial, size_YZ, pos_xn, QUNIT, true);
    // chrono::utils::AddBoxGeometry(ground.get(), mysurfmaterial, size_XZ, pos_yp, QUNIT, true);
    // chrono::utils::AddBoxGeometry(ground.get(), mysurfmaterial, size_XZ, pos_yn, QUNIT, true);
    sysMBS.AddBody(ground);

    // Fluid representation of walls
    sysFSI.AddBoxBCE(ground, pos_zn, QUNIT, size_XY, 12);
    // sysFSI.AddBoxBCE(ground, pos_zp, QUNIT, size_XY, 12);
    sysFSI.AddBoxBCE(ground, pos_xp, QUNIT, size_YZ, 23);
    sysFSI.AddBoxBCE(ground, pos_xn, QUNIT, size_YZ, 23);
    // sysFSI.AddBoxBCE(ground, pos_yp, QUNIT, size_XZ, 13);
    // sysFSI.AddBoxBCE(ground, pos_yn, QUNIT, size_XZ, 13);

    // ******************************* Rigid bodies ***********************************
    // set the abs orientation, position and velocity
    auto wheel = chrono_types::make_shared<ChBodyAuxRef>();
    ChQuaternion<> Body_rot = Q_from_Euler123(ChVector<double>(0, 0, 0));
    ChVector<> Body_pos = wheel_IniPos;
    ChVector<> Body_vel = wheel_IniVel;

    // Set inertia
    wheel->SetMass(total_mass * 1.0 / 2.0);
    wheel->SetInertiaXX(ChVector<>(60, 60, 60));
    wheel->SetPos_dt(Body_vel);
    wheel->SetWvel_loc(ChVector<>(0.0, 0.0, 0.0));  // set an initial anular velocity (rad/s)

    // Set the absolute position of the body:
    wheel->SetFrame_REF_to_abs(ChFrame<>(ChVector<>(Body_pos), ChQuaternion<>(Body_rot)));
    wheel->SetBodyFixed(false);
    wheel->SetCollide(false);
    sysMBS.AddBody(wheel);

    // Create the chassis 
    auto chassis = chrono_types::make_shared<ChBody>();
    chassis->SetMass(total_mass * 1.0 / 2.0);
    chassis->SetPos(wheel->GetPos());
    chassis->SetCollide(false);
    chassis->SetBodyFixed(false);
    sysMBS.AddBody(chassis);

    // Create the axle
    auto axle = chrono_types::make_shared<ChBody>();
    axle->SetMass(total_mass * 1.0 / 2.0);
    axle->SetPos(wheel->GetPos());
    axle->SetCollide(false);
    axle->SetBodyFixed(false);
    sysMBS.AddBody(axle);

    // Connect the chassis to the ground through a translational joint and create a linear actuator.
    auto prismatic1 = chrono_types::make_shared<ChLinkLockPrismatic>();
    prismatic1->Initialize(ground, chassis, ChCoordsys<>(chassis->GetPos(), Q_from_AngY(CH_C_PI_2)));
    prismatic1->SetName("prismatic_chassis_ground");
    sysMBS.AddLink(prismatic1);

    // double velocity = wheel_AngVel * wheel_radius * (1.0 - wheel_slip);
    // auto actuator_fun = chrono_types::make_shared<ChFunction_Ramp>(0.0, velocity);

    // actuator->Initialize(ground, chassis, false, ChCoordsys<>(chassis->GetPos(), QUNIT),
    //     ChCoordsys<>(chassis->GetPos() + ChVector<>(1, 0, 0), QUNIT));
    // actuator->SetName("actuator");
    // actuator->SetDistanceOffset(1);
    // actuator->SetActuatorFunction(actuator_fun);
    // sysMBS.AddLink(actuator);

    // Connect the axle to the chassis through a vertical translational joint.
    auto prismatic2 = chrono_types::make_shared<ChLinkLockPrismatic>();
    prismatic2->Initialize(chassis, axle, ChCoordsys<>(chassis->GetPos(), QUNIT));
    prismatic2->SetName("prismatic_axle_chassis");
    sysMBS.AddLink(prismatic2);

    // Connect the wheel to the axle through a engine joint.
    motor->SetName("engine_wheel_axle");
    motor->Initialize(wheel, axle, ChFrame<>(wheel->GetPos(), 
        chrono::Q_from_AngAxis(-CH_C_PI / 2.0, ChVector<>(1, 0, 0))));
    motor->SetAngleFunction(chrono_types::make_shared<ChFunction_Ramp>(0, wheel_AngVel));
    sysMBS.AddLink(motor);

    // ******************************* Flexible bodies ***********************************
    // Create a mesh, that is a container for groups of elements and their referenced nodes.
    auto my_mesh = chrono_types::make_shared<fea::ChMesh>();
    std::vector<std::vector<int>> _1D_elementsNodes_mesh;
    std::vector<std::vector<int>> _2D_elementsNodes_mesh;

    // Add the tire
    {
        auto mat = chrono_types::make_shared<ChMaterialShellANCF>(2000, 1.0e7, 0.3);

        // Create the mesh nodes.
        // The nodes are first created in the wheel local frame, assuming Y as the tire axis,
        // and are then transformed to the global frame.
        for (int i = 0; i < m_div_circumference; i++) {
            double phi = (CH_C_2PI * i) / m_div_circumference;
            for (int j = 0; j <= m_div_width; j++) {
                double theta = -CH_C_PI_2 + (CH_C_PI * j) / m_div_width;

                double x = (m_rim_radius + m_height * cos(theta)) * cos(phi) + wheel_IniPos.x();
                double y = m_height * sin(theta) + wheel_IniPos.y();
                double z = (m_rim_radius + m_height * cos(theta)) * sin(phi) + wheel_IniPos.z();
                ChVector<> loc = ChVector<>(x, y, z);

                double nx = cos(theta) * cos(phi);
                double ny = sin(theta);
                double nz = cos(theta) * sin(phi);
                ChVector<> dir = ChVector<>(nx, ny, nz);

                auto node = chrono_types::make_shared<ChNodeFEAxyzD>(loc, dir);
                node->SetMass(0.0);
                my_mesh->AddNode(node);

                // Fix the edge node on the wheel
                if (j == 0 || j == m_div_width){
                    auto mlink = chrono_types::make_shared<ChLinkPointFrame>();
                    mlink->Initialize(std::dynamic_pointer_cast<ChNodeFEAxyzD>(node), wheel);
                    sysMBS.Add(mlink);
                }
            }
        }

        int TotalNumElements = m_div_circumference * m_div_width; //my_mesh->GetNelements();
        int TotalNumNodes = my_mesh->GetNnodes();

        _2D_elementsNodes_mesh.resize(TotalNumElements);
        NodeNeighborElement_mesh.resize(TotalNumNodes);

        // Element dimensions
        double dz = m_thickness;
        double dx = CH_C_2PI * (m_rim_radius + m_height) / (1 * m_div_circumference);
        double dy = CH_C_PI * m_height / m_div_width;

        std::cout << dx  << " dx dy: " << dy << std::endl;

        // Create the ANCF shell elements
        int num_elem = 0;
        for (int i = 0; i < m_div_circumference; i++) {
            for (int j = 0; j < m_div_width; j++) {

                // Adjacent nodes
                int inode0, inode1, inode2, inode3;
                inode1 = j + i * (m_div_width + 1);
                inode2 = j + 1 + i * (m_div_width + 1);
                if (i == m_div_circumference - 1) {
                    inode0 = j;
                    inode3 = j + 1;
                } else {
                    inode0 = j + (i + 1) * (m_div_width + 1);
                    inode3 = j + 1 + (i + 1) * (m_div_width + 1);
                }

                auto node0 = std::dynamic_pointer_cast<ChNodeFEAxyzD>(my_mesh->GetNode(inode0));
                auto node1 = std::dynamic_pointer_cast<ChNodeFEAxyzD>(my_mesh->GetNode(inode1));
                auto node2 = std::dynamic_pointer_cast<ChNodeFEAxyzD>(my_mesh->GetNode(inode2));
                auto node3 = std::dynamic_pointer_cast<ChNodeFEAxyzD>(my_mesh->GetNode(inode3));

                _2D_elementsNodes_mesh[num_elem].push_back(inode0);
                _2D_elementsNodes_mesh[num_elem].push_back(inode1);
                _2D_elementsNodes_mesh[num_elem].push_back(inode2);
                _2D_elementsNodes_mesh[num_elem].push_back(inode3);
                NodeNeighborElement_mesh[inode0].push_back(num_elem);
                NodeNeighborElement_mesh[inode1].push_back(num_elem);
                NodeNeighborElement_mesh[inode2].push_back(num_elem);
                NodeNeighborElement_mesh[inode3].push_back(num_elem);

                // Create the element and set its nodes.
                auto element = chrono_types::make_shared<ChElementShellANCF_3423>();
                element->SetNodes(node0, node1, node2, node3);

                // Set element dimensions
                element->SetDimensions(dx, dy);

                // Add a single layers with a fiber angle of 0 degrees.
                element->AddLayer(dz, 0 * CH_C_DEG_TO_RAD, mat);

                // Set other element properties
                element->SetAlphaDamp(m_alpha);

                // Add element to mesh
                my_mesh->AddElement(element);

                ChVector<> center = 0.25 * (element->GetNodeA()->GetPos() + element->GetNodeB()->GetPos() +
                                            element->GetNodeC()->GetPos() + element->GetNodeD()->GetPos());
                std::cout << "Adding element" << num_elem << "  with center:  " << center.x() << " " << center.y()
                          << " " << center.z() << std::endl;

                num_elem++;
            }
        }
    }

    // Add the mesh to the system
    sysMBS.Add(my_mesh);

    // fluid representation of flexible bodies
    bool multilayer = true;
    bool removeMiddleLayer = true;
    bool add1DElem = flexible_elem_1D;
    bool add2DElem = !flexible_elem_1D;
    sysFSI.AddFEAmeshBCE(my_mesh, NodeNeighborElement_mesh, _1D_elementsNodes_mesh, 
        _2D_elementsNodes_mesh, add1DElem, add2DElem, multilayer, removeMiddleLayer, 0, 0);

    if (flexible_elem_1D)
        sysFSI.SetCableElementsNodes(_1D_elementsNodes_mesh);
    else
        sysFSI.SetShellElementsNodes(_2D_elementsNodes_mesh);

    sysFSI.SetFsiMesh(my_mesh);
    fea::ChMeshExporter::writeMesh(my_mesh, MESH_CONNECTIVITY);
}
