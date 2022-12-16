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
// Author: Wei Hu, Jason Zhou
// Chrono::FSI demo to show usage of VIPER rover models on SPH granular terrain
// This demo uses a plug-in VIPER rover model from chrono::models
// =============================================================================

#include "chrono_models/robot/viper/Viper.h"

#include "chrono/physics/ChSystemNSC.h"
#include "chrono/utils/ChUtilsCreators.h"
#include "chrono/utils/ChUtilsGenerators.h"
#include "chrono/utils/ChUtilsGeometry.h"
#include "chrono/utils/ChUtilsInputOutput.h"
#include "chrono/assets/ChBoxShape.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/physics/ChInertiaUtils.h"
#include "chrono/geometry/ChTriangleMeshConnected.h"

#include "chrono_fsi/ChSystemFsi.h"
#include "chrono_fsi/ChVisualizationFsi.h"

#include "chrono_thirdparty/filesystem/path.h"

/// Chrono namespaces
using namespace chrono;
using namespace chrono::fsi;
using namespace chrono::geometry;
using namespace chrono::viper;

/// output directories and settings
const std::string out_dir = GetChronoOutputPath() + "FSI_Viper/";
bool save_obj = false;  // if true, save as Wavefront OBJ; if false, save as VTK

/// Dimension of the space domain
double bxDim = 4.0;
double byDim = 2.0;
double bzDim = 0.1;

// Rover initial location
ChVector<> init_loc(-1, 0, 0.4);

// Simulation time and stepsize
double total_time = 20.0;
double dT = 2.5e-4;

// Save data as csv files to see the results off-line using Paraview
bool output = true;
int out_fps = 20;

// Enable/disable run-time visualization (if Chrono::OpenGL is available)
bool render = true;
float render_fps = 100;

// Pointer to store the VIPER instance
std::shared_ptr<Viper> rover;

std::shared_ptr<ChMaterialSurface> CustomWheelMaterial(ChContactMethod contact_method) {
    float mu = 0.4f;   // coefficient of friction
    float cr = 0.2f;   // coefficient of restitution
    float Y = 2e7f;    // Young's modulus
    float nu = 0.3f;   // Poisson ratio
    float kn = 2e5f;   // normal stiffness
    float gn = 40.0f;  // normal viscous damping
    float kt = 2e5f;   // tangential stiffness
    float gt = 20.0f;  // tangential viscous damping

    switch (contact_method) {
        case ChContactMethod::NSC: {
            auto matNSC = chrono_types::make_shared<ChMaterialSurfaceNSC>();
            matNSC->SetFriction(mu);
            matNSC->SetRestitution(cr);
            return matNSC;
        }
        case ChContactMethod::SMC: {
            auto matSMC = chrono_types::make_shared<ChMaterialSurfaceSMC>();
            matSMC->SetFriction(mu);
            matSMC->SetRestitution(cr);
            matSMC->SetYoungModulus(Y);
            matSMC->SetPoissonRatio(nu);
            matSMC->SetKn(kn);
            matSMC->SetGn(gn);
            matSMC->SetKt(kt);
            matSMC->SetGt(gt);
            return matSMC;
        }
        default:
            return std::shared_ptr<ChMaterialSurface>();
    }
}

// Forward declaration of helper functions
void SaveParaViewFiles(ChSystemFsi& sysFSI, ChSystemNSC& sysMBS, double mTime);
void CreateSolidPhase(ChSystemNSC& sysMBS, ChSystemFsi& sysFSI);

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
    if (!filesystem::create_directory(filesystem::path(out_dir + "/rover"))) {
        std::cerr << "Error creating directory " << out_dir + "/rover" << std::endl;
        return 1;
    }

    // Create a physical system and a corresponding FSI system
    ChSystemNSC sysMBS;
    ChSystemFsi sysFSI(sysMBS);

    ChVector<> gravity = ChVector<>(0, 0, -9.81);
    sysMBS.Set_G_acc(gravity);
    sysFSI.Set_G_acc(gravity);

    // Read JSON file with simulation parameters
    std::string inputJson = GetChronoDataFile("fsi/input_json/demo_FSI_Viper_granular_NSC.json");
    if (argc == 2) {
        inputJson = std::string(argv[1]);
    } else if (argc != 1) {
        std::cout << "usage: ./demo_ROBOT_Viper_SPH <json_file>" << std::endl;
        return 1;
    }

    sysFSI.ReadParametersFromFile(inputJson);

    sysFSI.SetStepSize(dT);

    sysFSI.SetContainerDim(ChVector<>(bxDim, byDim, bzDim));

    // Set SPH discretization type, consistent or inconsistent
    sysFSI.SetDiscreType(false, false);

    // Set wall boundary condition
    sysFSI.SetWallBC(BceVersion::ORIGINAL);

    // Setup the solver based on the input value of the prameters
    sysFSI.SetSPHMethod(FluidDynamics::WCSPH);

    // Set the periodic boundary condition
    double initSpace0 = sysFSI.GetInitialSpacing();
    ChVector<> cMin(-bxDim / 2 * 2, -byDim / 2 * 2, -bzDim * 10);
    ChVector<> cMax(bxDim / 2 * 2, byDim / 2 * 2, bzDim * 10);
    sysFSI.SetBoundaries(cMin, cMax);

    // Setup the output directory for FSI data
    sysFSI.SetOutputDirectory(out_dir);

    // Set simulation data output length
    sysFSI.SetOutputLength(0);

    // Create an initial box for the terrain patch
    chrono::utils::GridSampler<> sampler(initSpace0);
    ChVector<> boxCenter(0, 0, bzDim / 2);
    ChVector<> boxHalfDim(bxDim / 2, byDim / 2, bzDim / 2);
    std::vector<ChVector<>> points = sampler.SampleBox(boxCenter, boxHalfDim);

    // Add SPH particles from the sampler points to the FSI system
    auto gz = std::abs(gravity.z());
    int numPart = (int)points.size();
    for (int i = 0; i < numPart; i++) {
        double pre_ini = sysFSI.GetDensity() * gz * (-points[i].z() + bzDim);
        sysFSI.AddSPHParticle(points[i], sysFSI.GetDensity(), 0, sysFSI.GetViscosity(), sysFSI.GetKernelLength(),
                              ChVector<>(0),         // initial velocity
                              ChVector<>(-pre_ini),  // tauxxyyzz
                              ChVector<>(0)          // tauxyxzyz
        );
    }

    // Create MBD and BCE particles for the solid domain
    CreateSolidPhase(sysMBS, sysFSI);

    // Complete construction of the FSI system
    sysFSI.Initialize();

    // Get the body from the FSI system for visualization
    std::vector<std::shared_ptr<ChBody>>& FSI_Bodies = sysFSI.GetFsiBodies();
    auto Rover = FSI_Bodies[0];

    // Write position and velocity to file
    std::ofstream ofile;
    if (output)
        ofile.open(out_dir + "./body_position.txt");

    // Create a run-tme visualizer
    ChVisualizationFsi fsi_vis(&sysFSI);
    if (render) {
        fsi_vis.SetTitle("Viper on SPH terrain");
        fsi_vis.SetCameraPosition(ChVector<>(0, -3 * byDim, bzDim), ChVector<>(0, 0, 0));
        fsi_vis.SetCameraMoveScale(1.0f);
        fsi_vis.EnableBoundaryMarkers(false);
        fsi_vis.EnableRigidBodyMarkers(false);
        fsi_vis.AttachSystem(&sysMBS);
        fsi_vis.Initialize();
    }

    // Start the simulation
    unsigned int output_steps = (unsigned int)(1 / (out_fps * dT));
    unsigned int render_steps = (unsigned int)(1 / (render_fps * dT));
    double time = 0.0;
    int current_step = 0;

    auto body = sysMBS.Get_bodylist()[1];

    ChTimer<> timer;
    while (time < total_time) {
        std::cout << current_step << "  time: " << time << "  sim. time: " << timer() << std::endl; 

        rover->Update();

        std::cout << "  pos: " << body->GetPos() << std::endl;
        std::cout << "  vel: " << body->GetPos_dt() << std::endl;
        if (output) {
            ofile << time << "  " << body->GetPos() << "    " << body->GetPos_dt() << std::endl;
            if (current_step % output_steps == 0) {
                sysFSI.PrintParticleToFile(out_dir + "/particles");
                SaveParaViewFiles(sysFSI, sysMBS, time);
            }
        }

        // Render system
        if (render && current_step % render_steps == 0) {
            if (!fsi_vis.Render())
                break;
        }

        timer.start();
        sysFSI.DoStepDynamics_FSI();
        timer.stop();

        time += dT;
        current_step++;
    }

    if (output)
        ofile.close();

    return 0;
}

//------------------------------------------------------------------
// Create the objects of the MBD system. Rigid bodies and their
// BCE representations are created and added to the systems
//------------------------------------------------------------------
void CreateSolidPhase(ChSystemNSC& sysMBS, ChSystemFsi& sysFSI) {
    // Create a body for the rigid soil container
    auto box = chrono_types::make_shared<ChBodyEasyBox>(10, 10, 0.02, 1000, false, false);
    box->SetPos(ChVector<>(0, 0, 0));
    box->SetBodyFixed(true);
    sysMBS.Add(box);

    // Get the initial SPH particle spacing
    double initSpace0 = sysFSI.GetInitialSpacing();

    // Bottom wall
    ChVector<> size_XY(bxDim / 2 + 3 * initSpace0, byDim / 2 + 3 * initSpace0, 2 * initSpace0);
    ChVector<> pos_zn(0, 0, -3 * initSpace0);
    ChVector<> pos_zp(0, 0, bzDim + 2 * initSpace0);

    // Left and right wall
    ChVector<> size_YZ(2 * initSpace0, byDim / 2 + 3 * initSpace0, bzDim / 2);
    ChVector<> pos_xp(bxDim / 2 + initSpace0, 0.0, bzDim / 2 + 0 * initSpace0);
    ChVector<> pos_xn(-bxDim / 2 - 3 * initSpace0, 0.0, bzDim / 2 + 0 * initSpace0);

    // Front and back wall
    ChVector<> size_XZ(bxDim / 2, 2 * initSpace0, bzDim / 2);
    ChVector<> pos_yp(0, byDim / 2 + initSpace0, bzDim / 2 + 0 * initSpace0);
    ChVector<> pos_yn(0, -byDim / 2 - 3 * initSpace0, bzDim / 2 + 0 * initSpace0);

    // Fluid-Solid Coupling at the walls via BCE particles
    sysFSI.AddBoxBCE(box, pos_zn, QUNIT, size_XY, 12);
    sysFSI.AddBoxBCE(box, pos_xp, QUNIT, size_YZ, 23);
    sysFSI.AddBoxBCE(box, pos_xn, QUNIT, size_YZ, 23);
    sysFSI.AddBoxBCE(box, pos_yp, QUNIT, size_XZ, 13);
    sysFSI.AddBoxBCE(box, pos_yn, QUNIT, size_XZ, 13);

    auto driver = chrono_types::make_shared<ViperDCMotorControl>();
    rover = chrono_types::make_shared<Viper>(&sysMBS);
    rover->SetDriver(driver);
    rover->SetWheelContactMaterial(CustomWheelMaterial(ChContactMethod::NSC));
    rover->Initialize(ChFrame<>(init_loc, QUNIT));

    // Add BCE particles and mesh of wheels to the system
    for (int i = 0; i < 4; i++) {
        std::shared_ptr<ChBodyAuxRef> wheel_body;
        if (i == 0) {
            wheel_body = rover->GetWheel(ViperWheelID::V_LF)->GetBody();
        }
        if (i == 1) {
            wheel_body = rover->GetWheel(ViperWheelID::V_RF)->GetBody();
        }
        if (i == 2) {
            wheel_body = rover->GetWheel(ViperWheelID::V_LB)->GetBody();
        }
        if (i == 3) {
            wheel_body = rover->GetWheel(ViperWheelID::V_RB)->GetBody();
        }

        sysFSI.AddFsiBody(wheel_body);
        std::string BCE_path = GetChronoDataFile("fsi/demo_BCE/BCE_viperWheel.txt");
        if (i == 0 || i == 2) {
            sysFSI.AddFileBCE(wheel_body, BCE_path, ChVector<>(0), Q_from_AngZ(CH_C_PI), 1.0, true);
        } else {
            sysFSI.AddFileBCE(wheel_body, BCE_path, ChVector<>(0), QUNIT, 1.0, true);
        }
    }
}

//------------------------------------------------------------------
// Function to save the povray files of the MBD
//------------------------------------------------------------------
void SaveParaViewFiles(ChSystemFsi& sysFSI, ChSystemNSC& sysMBS, double mTime) {
    std::string rover_dir = out_dir + "/rover";
    std::string filename;
    static int frame_number = -1;
    frame_number++;

    // save the VIPER body to obj/vtk files
    for (int i = 0; i < 1; i++) {
        auto body = rover->GetChassis()->GetBody();
        ChFrame<> body_ref_frame = body->GetFrame_REF_to_abs();
        ChVector<> body_pos = body_ref_frame.GetPos();      // body->GetPos();
        ChQuaternion<> body_rot = body_ref_frame.GetRot();  // body->GetRot();

        auto mmesh = chrono_types::make_shared<ChTriangleMeshConnected>();
        std::string obj_path = (GetChronoDataFile("robot/viper/obj/viper_chassis.obj"));
        double scale_ratio = 1.0;
        mmesh->LoadWavefrontMesh(obj_path, false, true);
        mmesh->Transform(ChVector<>(0, 0, 0), ChMatrix33<>(scale_ratio));  // scale to a different size
        mmesh->RepairDuplicateVertexes(1e-9);                              // if meshes are not watertight

        double mmass;
        ChVector<> mcog;
        ChMatrix33<> minertia;
        mmesh->ComputeMassProperties(true, mmass, mcog, minertia);
        mmesh->Transform(body_pos, ChMatrix33<>(body_rot));  // rotate the mesh based on the orientation of body

        if (save_obj) {  // save to obj file
            filename = rover_dir + "/body_" + std::to_string(frame_number) + ".obj";
            std::vector<geometry::ChTriangleMeshConnected> meshes = {*mmesh};
            geometry::ChTriangleMeshConnected::WriteWavefront(filename, meshes);
        } else {  // save to vtk file
            filename = rover_dir + "/body_" + std::to_string(frame_number) + ".vtk";
            std::ofstream file;
            file.open(filename);
            file << "# vtk DataFile Version 2.0" << std::endl;
            file << "VTK from simulation" << std::endl;
            file << "ASCII" << std::endl;
            file << "DATASET UNSTRUCTURED_GRID" << std::endl;
            auto nv = mmesh->getCoordsVertices().size();
            file << "POINTS " << nv << " float" << std::endl;
            for (auto& v : mmesh->getCoordsVertices())
                file << v.x() << " " << v.y() << " " << v.z() << std::endl;
            auto nf = mmesh->getIndicesVertexes().size();
            file << "CELLS " << nf << " " << 4 * nf << std::endl;
            for (auto& f : mmesh->getIndicesVertexes())
                file << "3 " << f.x() << " " << f.y() << " " << f.z() << std::endl;
            file << "CELL_TYPES " << nf << std::endl;
            for (size_t ii = 0; ii < nf; ii++)
                file << "5 " << std::endl;
            file.close();
        }
    }

    // save the wheels to obj/vtk files
    for (int i = 0; i < 4; i++) {
        std::shared_ptr<ChBodyAuxRef> body;
        if (i == 0) {
            body = rover->GetWheel(ViperWheelID::V_LF)->GetBody();
        }
        if (i == 1) {
            body = rover->GetWheel(ViperWheelID::V_RF)->GetBody();
        }
        if (i == 2) {
            body = rover->GetWheel(ViperWheelID::V_LB)->GetBody();
        }
        if (i == 3) {
            body = rover->GetWheel(ViperWheelID::V_RB)->GetBody();
        }

        ChFrame<> body_ref_frame = body->GetFrame_REF_to_abs();
        ChVector<> body_pos = body_ref_frame.GetPos();      // body->GetPos();
        ChQuaternion<> body_rot = body_ref_frame.GetRot();  // body->GetRot();
        if (i == 0 || i == 2) {
            body_rot.Cross(body_rot, Q_from_AngZ(CH_C_PI));
        }

        auto mmesh = chrono_types::make_shared<ChTriangleMeshConnected>();
        std::string obj_path = GetChronoDataFile("robot/viper/obj/viper_wheel.obj");
        double scale_ratio = 1.0;
        mmesh->LoadWavefrontMesh(obj_path, false, true);
        mmesh->Transform(ChVector<>(0, 0, 0), ChMatrix33<>(scale_ratio));  // scale to a different size
        mmesh->RepairDuplicateVertexes(1e-9);                              // if meshes are not watertight

        double mmass;
        ChVector<> mcog;
        ChMatrix33<> minertia;
        mmesh->ComputeMassProperties(true, mmass, mcog, minertia);
        mmesh->Transform(body_pos, ChMatrix33<>(body_rot));  // rotate the mesh based on the orientation of body

        if (save_obj) {  // save to obj file
            filename = rover_dir + "/wheel_" + std::to_string(i + 1) + "_" + std::to_string(frame_number) + ".obj";
            std::vector<geometry::ChTriangleMeshConnected> meshes = {*mmesh};
            geometry::ChTriangleMeshConnected::WriteWavefront(filename, meshes);
        } else {  // save to vtk file
            filename = rover_dir + "/wheel_" + std::to_string(i + 1) + "_" + std::to_string(frame_number) + ".vtk";
            std::ofstream file;
            file.open(filename);
            file << "# vtk DataFile Version 2.0" << std::endl;
            file << "VTK from simulation" << std::endl;
            file << "ASCII" << std::endl;
            file << "DATASET UNSTRUCTURED_GRID" << std::endl;
            auto nv = mmesh->getCoordsVertices().size();
            file << "POINTS " << nv << " float" << std::endl;
            for (auto& v : mmesh->getCoordsVertices())
                file << v.x() << " " << v.y() << " " << v.z() << std::endl;
            auto nf = mmesh->getIndicesVertexes().size();
            file << "CELLS " << nf << " " << 4 * nf << std::endl;
            for (auto& f : mmesh->getIndicesVertexes())
                file << "3 " << f.x() << " " << f.y() << " " << f.z() << std::endl;
            file << "CELL_TYPES " << nf << std::endl;
            for (size_t ii = 0; ii < nf; ii++)
                file << "5 " << std::endl;
            file.close();
        }
    }

    // save the steering rod to obj/vtk files
    for (int i = 0; i < 4; i++) {
        std::shared_ptr<ChBodyAuxRef> body;
        if (i == 0) {
            body = rover->GetUpright(ViperWheelID::V_LF)->GetBody();
        }
        if (i == 1) {
            body = rover->GetUpright(ViperWheelID::V_RF)->GetBody();
        }
        if (i == 2) {
            body = rover->GetUpright(ViperWheelID::V_LB)->GetBody();
        }
        if (i == 3) {
            body = rover->GetUpright(ViperWheelID::V_RB)->GetBody();
        }
        ChFrame<> body_ref_frame = body->GetFrame_REF_to_abs();
        ChVector<> body_pos = body_ref_frame.GetPos();      // body->GetPos();
        ChQuaternion<> body_rot = body_ref_frame.GetRot();  // body->GetRot();

        auto mmesh = chrono_types::make_shared<ChTriangleMeshConnected>();
        std::string obj_path = "";
        if (i == 0 || i == 2) {
            obj_path = GetChronoDataFile("robot/viper/obj/viper_L_steer.obj");
        }
        if (i == 1 || i == 3) {
            obj_path = GetChronoDataFile("robot/viper/obj/viper_R_steer.obj");
        }
        double scale_ratio = 1.0;
        mmesh->LoadWavefrontMesh(obj_path, false, true);
        mmesh->Transform(ChVector<>(0, 0, 0), ChMatrix33<>(scale_ratio));  // scale to a different size
        mmesh->RepairDuplicateVertexes(1e-9);                              // if meshes are not watertight

        double mmass;
        ChVector<> mcog;
        ChMatrix33<> minertia;
        mmesh->ComputeMassProperties(true, mmass, mcog, minertia);
        mmesh->Transform(body_pos, ChMatrix33<>(body_rot));  // rotate the mesh based on the orientation of body

        if (save_obj) {  // save to obj file
            filename = rover_dir + "/steerRod_" + std::to_string(i + 1) + "_" + std::to_string(frame_number) + ".obj";
            std::vector<geometry::ChTriangleMeshConnected> meshes = {*mmesh};
            geometry::ChTriangleMeshConnected::WriteWavefront(filename, meshes);
        } else {  // save to vtk file
            filename = rover_dir + "/steerRod_" + std::to_string(i + 1) + "_" + std::to_string(frame_number) + ".vtk";
            std::ofstream file;
            file.open(filename);
            file << "# vtk DataFile Version 2.0" << std::endl;
            file << "VTK from simulation" << std::endl;
            file << "ASCII" << std::endl;
            file << "DATASET UNSTRUCTURED_GRID" << std::endl;
            auto nv = mmesh->getCoordsVertices().size();
            file << "POINTS " << nv << " float" << std::endl;
            for (auto& v : mmesh->getCoordsVertices())
                file << v.x() << " " << v.y() << " " << v.z() << std::endl;
            auto nf = mmesh->getIndicesVertexes().size();
            file << "CELLS " << nf << " " << 4 * nf << std::endl;
            for (auto& f : mmesh->getIndicesVertexes())
                file << "3 " << f.x() << " " << f.y() << " " << f.z() << std::endl;
            file << "CELL_TYPES " << nf << std::endl;
            for (size_t ii = 0; ii < nf; ii++)
                file << "5 " << std::endl;
            file.close();
        }
    }

    // save the lower rod to obj/vtk files
    for (int i = 0; i < 4; i++) {
        std::shared_ptr<ChBodyAuxRef> body;
        if (i == 0) {
            body = rover->GetLowerArm(ViperWheelID::V_LF)->GetBody();
        }
        if (i == 1) {
            body = rover->GetLowerArm(ViperWheelID::V_RF)->GetBody();
        }
        if (i == 2) {
            body = rover->GetLowerArm(ViperWheelID::V_LB)->GetBody();
        }
        if (i == 3) {
            body = rover->GetLowerArm(ViperWheelID::V_RB)->GetBody();
        }
        ChFrame<> body_ref_frame = body->GetFrame_REF_to_abs();
        ChVector<> body_pos = body_ref_frame.GetPos();      // body->GetPos();
        ChQuaternion<> body_rot = body_ref_frame.GetRot();  // body->GetRot();

        auto mmesh = chrono_types::make_shared<ChTriangleMeshConnected>();
        std::string obj_path = "";
        if (i == 0 || i == 2) {
            obj_path = GetChronoDataFile("robot/viper/obj/viper_L_bt_sus.obj");
        }
        if (i == 1 || i == 3) {
            obj_path = GetChronoDataFile("robot/viper/obj/viper_R_bt_sus.obj");
        }
        double scale_ratio = 1.0;
        mmesh->LoadWavefrontMesh(obj_path, false, true);
        mmesh->Transform(ChVector<>(0, 0, 0), ChMatrix33<>(scale_ratio));  // scale to a different size
        mmesh->RepairDuplicateVertexes(1e-9);                              // if meshes are not watertight

        double mmass;
        ChVector<> mcog;
        ChMatrix33<> minertia;
        mmesh->ComputeMassProperties(true, mmass, mcog, minertia);
        mmesh->Transform(body_pos, ChMatrix33<>(body_rot));  // rotate the mesh based on the orientation of body

        if (save_obj) {  // save to obj file
            filename = rover_dir + "/lowerRod_" + std::to_string(i + 1) + "_" + std::to_string(frame_number) + ".obj";
            std::vector<geometry::ChTriangleMeshConnected> meshes = {*mmesh};
            geometry::ChTriangleMeshConnected::WriteWavefront(filename, meshes);
        } else {  // save to vtk file
            filename = rover_dir + "/lowerRod_" + std::to_string(i + 1) + "_" + std::to_string(frame_number) + ".vtk";
            std::ofstream file;
            file.open(filename);
            file << "# vtk DataFile Version 2.0" << std::endl;
            file << "VTK from simulation" << std::endl;
            file << "ASCII" << std::endl;
            file << "DATASET UNSTRUCTURED_GRID" << std::endl;
            auto nv = mmesh->getCoordsVertices().size();
            file << "POINTS " << nv << " float" << std::endl;
            for (auto& v : mmesh->getCoordsVertices())
                file << v.x() << " " << v.y() << " " << v.z() << std::endl;
            auto nf = mmesh->getIndicesVertexes().size();
            file << "CELLS " << nf << " " << 4 * nf << std::endl;
            for (auto& f : mmesh->getIndicesVertexes())
                file << "3 " << f.x() << " " << f.y() << " " << f.z() << std::endl;
            file << "CELL_TYPES " << nf << std::endl;
            for (size_t ii = 0; ii < nf; ii++)
                file << "5 " << std::endl;
            file.close();
        }
    }

    // save the upper rod to obj/vtk files
    for (int i = 0; i < 4; i++) {
        std::shared_ptr<ChBodyAuxRef> body;
        if (i == 0) {
            body = rover->GetUpperArm(ViperWheelID::V_LF)->GetBody();
        }
        if (i == 1) {
            body = rover->GetUpperArm(ViperWheelID::V_RF)->GetBody();
        }
        if (i == 2) {
            body = rover->GetUpperArm(ViperWheelID::V_LB)->GetBody();
        }
        if (i == 3) {
            body = rover->GetUpperArm(ViperWheelID::V_RB)->GetBody();
        }
        ChFrame<> body_ref_frame = body->GetFrame_REF_to_abs();
        ChVector<> body_pos = body_ref_frame.GetPos();      // body->GetPos();
        ChQuaternion<> body_rot = body_ref_frame.GetRot();  // body->GetRot();

        auto mmesh = chrono_types::make_shared<ChTriangleMeshConnected>();
        std::string obj_path = "";
        if (i == 0 || i == 2) {
            obj_path = GetChronoDataFile("robot/viper/obj/viper_L_up_sus.obj");
        }
        if (i == 1 || i == 3) {
            obj_path = GetChronoDataFile("robot/viper/obj/viper_R_up_sus.obj");
        }

        double scale_ratio = 1.0;
        mmesh->LoadWavefrontMesh(obj_path, false, true);
        mmesh->Transform(ChVector<>(0, 0, 0), ChMatrix33<>(scale_ratio));  // scale to a different size
        mmesh->RepairDuplicateVertexes(1e-9);                              // if meshes are not watertight

        double mmass;
        ChVector<> mcog;
        ChMatrix33<> minertia;
        mmesh->ComputeMassProperties(true, mmass, mcog, minertia);
        mmesh->Transform(body_pos, ChMatrix33<>(body_rot));  // rotate the mesh based on the orientation of body

        if (save_obj) {  // save to obj file
            filename = rover_dir + "/upperRod_" + std::to_string(i + 1) + "_" + std::to_string(frame_number) + ".obj";
            std::vector<geometry::ChTriangleMeshConnected> meshes = {*mmesh};
            geometry::ChTriangleMeshConnected::WriteWavefront(filename, meshes);
        } else {  // save to vtk file
            filename = rover_dir + "/upperRod_" + std::to_string(i + 1) + "_" + std::to_string(frame_number) + ".vtk";
            std::ofstream file;
            file.open(filename);
            file << "# vtk DataFile Version 2.0" << std::endl;
            file << "VTK from simulation" << std::endl;
            file << "ASCII" << std::endl;
            file << "DATASET UNSTRUCTURED_GRID" << std::endl;
            auto nv = mmesh->getCoordsVertices().size();
            file << "POINTS " << nv << " float" << std::endl;
            for (auto& v : mmesh->getCoordsVertices())
                file << v.x() << " " << v.y() << " " << v.z() << std::endl;
            auto nf = mmesh->getIndicesVertexes().size();
            file << "CELLS " << nf << " " << 4 * nf << std::endl;
            for (auto& f : mmesh->getIndicesVertexes())
                file << "3 " << f.x() << " " << f.y() << " " << f.z() << std::endl;
            file << "CELL_TYPES " << nf << std::endl;
            for (size_t ii = 0; ii < nf; ii++)
                file << "5 " << std::endl;
            file.close();
        }
    }

    // save box obstacle to vtk files
    double lx = 0.1;
    double ly = 0.25;
    double lz = 0.05;
    ChVector<double> Node1 = ChVector<double>(-lx, -ly, -lz);
    ChVector<double> Node2 = ChVector<double>(lx, -ly, -lz);
    ChVector<double> Node3 = ChVector<double>(lx, -ly, lz);
    ChVector<double> Node4 = ChVector<double>(-lx, -ly, lz);
    ChVector<double> Node5 = ChVector<double>(-lx, ly, -lz);
    ChVector<double> Node6 = ChVector<double>(lx, ly, -lz);
    ChVector<double> Node7 = ChVector<double>(lx, ly, lz);
    ChVector<double> Node8 = ChVector<double>(-lx, ly, lz);

    for (int i = 0; i < 2; i++) {
        filename = rover_dir + "/obstacle_" + std::to_string(i + 1) + "_" + std::to_string(frame_number) + ".vtk";
        std::ofstream file;
        file.open(filename);
        file << "# vtk DataFile Version 2.0" << std::endl;
        file << "VTK from simulation" << std::endl;
        file << "ASCII" << std::endl;
        file << "DATASET POLYDATA" << std::endl;

        file << "POINTS " << 8 << " "
             << "float" << std::endl;
        auto Body = sysMBS.Get_bodylist()[i + 2 + 16];
        ChVector<> center = Body->GetPos();
        ChMatrix33<> Rotation = Body->GetRot();
        ChVector<double> vertex1 = Rotation * Node1 + center;
        ChVector<double> vertex2 = Rotation * Node2 + center;
        ChVector<double> vertex3 = Rotation * Node3 + center;
        ChVector<double> vertex4 = Rotation * Node4 + center;
        ChVector<double> vertex5 = Rotation * Node5 + center;
        ChVector<double> vertex6 = Rotation * Node6 + center;
        ChVector<double> vertex7 = Rotation * Node7 + center;
        ChVector<double> vertex8 = Rotation * Node8 + center;
        file << vertex1.x() << " " << vertex1.y() << " " << vertex1.z() << "\n";
        file << vertex2.x() << " " << vertex2.y() << " " << vertex2.z() << "\n";
        file << vertex3.x() << " " << vertex3.y() << " " << vertex3.z() << "\n";
        file << vertex4.x() << " " << vertex4.y() << " " << vertex4.z() << "\n";
        file << vertex5.x() << " " << vertex5.y() << " " << vertex5.z() << "\n";
        file << vertex6.x() << " " << vertex6.y() << " " << vertex6.z() << "\n";
        file << vertex7.x() << " " << vertex7.y() << " " << vertex7.z() << "\n";
        file << vertex8.x() << " " << vertex8.y() << " " << vertex8.z() << "\n";

        file << "POLYGONS " << 6 << " " << 30 << std::endl;
        file << "4 " << 0 << " " << 1 << " " << 2 << " " << 3 << "\n";
        file << "4 " << 0 << " " << 1 << " " << 5 << " " << 4 << "\n";
        file << "4 " << 0 << " " << 4 << " " << 7 << " " << 3 << "\n";
        file << "4 " << 4 << " " << 5 << " " << 6 << " " << 7 << "\n";
        file << "4 " << 1 << " " << 5 << " " << 6 << " " << 2 << "\n";
        file << "4 " << 3 << " " << 2 << " " << 6 << " " << 7 << "\n";
    }

    // save rigid body position and rotation
    for (int i = 1; i < sysMBS.Get_bodylist().size(); i++) {
        auto body = sysMBS.Get_bodylist()[i];
        ChFrame<> ref_frame = body->GetFrame_REF_to_abs();
        ChVector<> pos = ref_frame.GetPos();
        ChQuaternion<> rot = ref_frame.GetRot();
        ChVector<> vel = body->GetPos_dt();

        std::string delim = ",";
        filename = rover_dir + "/body_pos_rot_vel" + std::to_string(i) + ".csv";
        std::ofstream file;
        if (sysMBS.GetChTime() > 0)
            file.open(filename, std::fstream::app);
        else {
            file.open(filename);
            file << "Time" << delim << "x" << delim << "y" << delim << "z" << delim << "q0" << delim << "q1" << delim
                 << "q2" << delim << "q3" << delim << "Vx" << delim << "Vy" << delim << "Vz" << std::endl;
        }

        file << sysMBS.GetChTime() << delim << pos.x() << delim << pos.y() << delim << pos.z() << delim << rot.e0()
             << delim << rot.e1() << delim << rot.e2() << delim << rot.e3() << delim << vel.x() << delim << vel.y()
             << delim << vel.z() << std::endl;

        file.close();
    }

    std::cout << "-------------------------------------" << std::endl;
    std::cout << " Output frame:  " << frame_number << std::endl;
    std::cout << " Time:          " << mTime << std::endl;
    std::cout << "-------------------------------------" << std::endl;
}
