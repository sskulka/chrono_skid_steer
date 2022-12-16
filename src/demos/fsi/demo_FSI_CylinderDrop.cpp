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
// Author: Milad Rakhsha, Wei Hu
// =============================================================================

#include <cassert>
#include <cstdlib>
#include <ctime>

#include "chrono/physics/ChSystemSMC.h"
#include "chrono/utils/ChUtilsCreators.h"
#include "chrono/utils/ChUtilsGenerators.h"
#include "chrono/utils/ChUtilsGeometry.h"
#include "chrono/assets/ChBoxShape.h"
#include "chrono/core/ChTransform.h"

#include "chrono_fsi/ChSystemFsi.h"
#include "chrono_fsi/ChVisualizationFsi.h"

#include "chrono_thirdparty/filesystem/path.h"

using namespace chrono;
using namespace chrono::collision;
using namespace chrono::fsi;

// -----------------------------------------------------------------

// Output directories and settings
const std::string out_dir = GetChronoOutputPath() + "FSI_Cylinder_Drop/";

// Output frequency
bool output = true;
double out_fps = 20;

// Dimension of the space domain
double bxDim = 1.0;
double byDim = 0.6;
double bzDim = 1.2;

// Size of the cylinder
double cyl_length = 0.2001;
double cyl_radius = 0.12;

// Final simulation time
double t_end = 2.0;

// Enable/disable run-time visualization (if Chrono::OpenGL is available)
bool render = true;
float render_fps = 1000;

//------------------------------------------------------------------
// Function to add walls into Chrono system
//------------------------------------------------------------------
void AddWall(std::shared_ptr<ChBody> body,
             const ChVector<>& dim,
             std::shared_ptr<ChMaterialSurface> mat,
             const ChVector<>& loc) {
    body->GetCollisionModel()->AddBox(mat, dim.x(), dim.y(), dim.z(), loc);
    auto box = chrono_types::make_shared<ChBoxShape>();
    box->GetBoxGeometry().Size = dim;
}

//------------------------------------------------------------------
// Function to save cylinder to Paraview VTK files
//------------------------------------------------------------------
void WriteCylinderVTK(const std::string& filename,
                      double radius,
                      double length,
                      const ChFrame<>& frame,
                      unsigned int res) {
    std::ofstream outf;
    outf.open(filename, std::ios::app);
    outf << "# vtk DataFile Version 1.0\nUnstructured Grid Example\nASCII\n\n" << std::endl;
    outf << "DATASET UNSTRUCTURED_GRID\nPOINTS " << 2 * res << " float\n";

    for (int i = 0; i < res; i++) {
        auto w = frame.TransformPointLocalToParent(
            ChVector<>(radius * cos(2 * i * 3.1415 / res), -1 * length / 2, radius * sin(2 * i * 3.1415 / res)));
        outf << w.x() << " " << w.y() << " " << w.z() << "\n";
    }

    for (int i = 0; i < res; i++) {
        auto w = frame.TransformPointLocalToParent(
            ChVector<>(radius * cos(2 * i * 3.1415 / res), +1 * length / 2, radius * sin(2 * i * 3.1415 / res)));
        outf << w.x() << " " << w.y() << " " << w.z() << "\n";
    }

    outf << "\n\nCELLS " << res + res << "\t" << 5 * (res + res) << "\n";

    for (int i = 0; i < res - 1; i++) {
        outf << "4 " << i << " " << i + 1 << " " << i + res + 1 << " " << i + res << "\n";
    }
    outf << "4 " << res - 1 << " " << 0 << " " << res << " " << 2 * res - 1 << "\n";

    for (int i = 0; i < res / 4; i++) {
        outf << "4 " << i << " " << i + 1 << " " << +res / 2 - i - 1 << " " << +res / 2 - i << "\n";
    }

    for (int i = 0; i < res / 4; i++) {
        outf << "4 " << i + res << " " << i + 1 + res << " " << +res / 2 - i - 1 + res << " " << +res / 2 - i + res
             << "\n";
    }

    outf << "4 " << +res / 2 << " " << 1 + res / 2 << " " << +res - 1 << " " << 0 << "\n";

    for (int i = 1; i < res / 4; i++) {
        outf << "4 " << i + res / 2 << " " << i + 1 + res / 2 << " " << +res / 2 - i - 1 + res / 2 << " "
             << +res / 2 - i + res / 2 << "\n";
    }

    outf << "4 " << 3 * res / 2 << " " << 1 + 3 * res / 2 << " " << +2 * res - 1 << " " << +res << "\n";

    for (int i = 1; i < res / 4; i++) {
        outf << "4 " << i + 3 * res / 2 << " " << i + 1 + 3 * res / 2 << " " << +2 * res - i - 1 << " " << +2 * res - i
             << "\n";
    }

    outf << "\nCELL_TYPES " << res + res << "\n";

    for (int iele = 0; iele < (res + res); iele++) {
        outf << "9\n";
    }
}

//------------------------------------------------------------------
// Create the objects of the MBD system. Rigid bodies, and if FSI,
// their BCE representation are created and added to the systems
//------------------------------------------------------------------
void CreateSolidPhase(ChSystemSMC& sysMBS, ChSystemFsi& sysFSI) {
    // Set gravity to the rigid body system in chrono
    sysMBS.Set_G_acc(sysFSI.Get_G_acc());

    // Set common material Properties
    auto mysurfmaterial = chrono_types::make_shared<ChMaterialSurfaceSMC>();
    mysurfmaterial->SetYoungModulus(1e8);
    mysurfmaterial->SetFriction(0.2f);
    mysurfmaterial->SetRestitution(0.05f);
    mysurfmaterial->SetAdhesion(0);

    // Get particle spacing in the simulation
    auto initSpace0 = sysFSI.GetInitialSpacing();

    // Bottom and top wall - size and position
    ChVector<> size_XY(bxDim / 2 + 3 * initSpace0, byDim / 2 + 3 * initSpace0, 2 * initSpace0);
    ChVector<> pos_zp(0, 0, 2 * bzDim + 1 * initSpace0);
    ChVector<> pos_zn(0, 0, -3 * initSpace0);

    // Left and right wall - size and position
    ChVector<> size_YZ(2 * initSpace0, byDim / 2 + 3 * initSpace0, bzDim);
    ChVector<> pos_xp(bxDim / 2 + initSpace0, 0.0, bzDim + 0 * initSpace0);
    ChVector<> pos_xn(-bxDim / 2 - 3 * initSpace0, 0.0, bzDim + 0 * initSpace0);

    // Front and back wall - size and position
    ChVector<> size_XZ(bxDim / 2, 2 * initSpace0, bzDim);
    ChVector<> pos_yp(0, byDim / 2 + initSpace0, bzDim + 0 * initSpace0);
    ChVector<> pos_yn(0, -byDim / 2 - 3 * initSpace0, bzDim + 0 * initSpace0);

    // Create a container
    auto box = chrono_types::make_shared<ChBody>();
    box->SetPos(ChVector<>(0.0, 0.0, 0.0));
    box->SetRot(ChQuaternion<>(1, 0, 0, 0));
    box->SetIdentifier(-1);
    box->SetBodyFixed(true);
    box->GetCollisionModel()->ClearModel();
    box->GetCollisionModel()->SetSafeMargin(initSpace0 / 2);

    // Add the walls into chrono system
    AddWall(box, size_XY, mysurfmaterial, pos_zp);
    AddWall(box, size_XY, mysurfmaterial, pos_zn);
    AddWall(box, size_YZ, mysurfmaterial, pos_xp);
    AddWall(box, size_YZ, mysurfmaterial, pos_xn);
    AddWall(box, size_XZ, mysurfmaterial, pos_yp);
    AddWall(box, size_XZ, mysurfmaterial, pos_yn);
    box->GetCollisionModel()->BuildModel();
    box->SetCollide(true);
    sysMBS.AddBody(box);

    // Add BCE particles attached on the walls into FSI system
    sysFSI.AddBoxBCE(box, pos_zp, QUNIT, size_XY, 12);
    sysFSI.AddBoxBCE(box, pos_zn, QUNIT, size_XY, 12);
    sysFSI.AddBoxBCE(box, pos_xp, QUNIT, size_YZ, 23);
    sysFSI.AddBoxBCE(box, pos_xn, QUNIT, size_YZ, 23);
    sysFSI.AddBoxBCE(box, pos_yp, QUNIT, size_XZ, 13);
    sysFSI.AddBoxBCE(box, pos_yn, QUNIT, size_XZ, 13);

    // Create a falling cylinder
    auto cylinder = chrono_types::make_shared<ChBody>();

    // Set the general properties of the cylinder
    double volume = chrono::utils::CalcCylinderVolume(cyl_radius, cyl_length / 2);
    double density = sysFSI.GetDensity() * 2.0;
    double mass = density * volume;
    ChVector<> cyl_pos = ChVector<>(0, 0, bzDim + cyl_radius + 2 * initSpace0);
    ChVector<> cyl_vel = ChVector<>(0.0, 0.0, 0.0);
    ChQuaternion<> cyl_rot = QUNIT;
    ChVector<> gyration = chrono::utils::CalcCylinderGyration(cyl_radius, cyl_length / 2).diagonal();
    cylinder->SetPos(cyl_pos);
    cylinder->SetPos_dt(cyl_vel);
    cylinder->SetMass(mass);
    cylinder->SetInertiaXX(mass * gyration);

    // Set the collision type of the cylinder
    cylinder->SetCollide(true);
    cylinder->SetBodyFixed(false);
    cylinder->GetCollisionModel()->ClearModel();
    cylinder->GetCollisionModel()->SetSafeMargin(initSpace0);
    chrono::utils::AddCylinderGeometry(cylinder.get(), mysurfmaterial, cyl_radius, cyl_length,
                                       ChVector<>(0.0, 0.0, 0.0), cyl_rot);
    cylinder->GetCollisionModel()->BuildModel();

    // Add this body to chrono system
    sysMBS.AddBody(cylinder);

    // Add this body to the FSI system (only those have inetraction with fluid)
    sysFSI.AddFsiBody(cylinder);

    // Add BCE particles attached on the cylinder into FSI system
    sysFSI.AddCylinderBCE(cylinder, ChVector<>(0), ChQuaternion<>(1, 0, 0, 0), cyl_radius, cyl_length + initSpace0,
                          sysFSI.GetKernelLength(), false);
}

// =============================================================================
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

    std::string inputJson = GetChronoDataFile("fsi/input_json/demo_FSI_CylinderDrop_Explicit.json");
    if (argc == 1) {
        std::cout << "Use the default JSON file" << std::endl;
    } else if (argc == 2) {
        std::cout << "Use the specified JSON file" << std::endl;
        std::string my_inputJson = std::string(argv[1]);
        inputJson = my_inputJson;
    } else {
        std::cout << "usage: ./demo_FSI_CylinderDrop <json_file>" << std::endl;
        return 1;
    }
    sysFSI.ReadParametersFromFile(inputJson);

    // Set the periodic boundary condition (if not, set relative larger values)
    auto initSpace0 = sysFSI.GetInitialSpacing();
    ChVector<> cMin(-bxDim / 2 * 10, -byDim / 2 * 10, -bzDim * 10);
    ChVector<> cMax(bxDim / 2 * 10, byDim / 2 * 10, bzDim * 10);
    sysFSI.SetBoundaries(cMin, cMax);

    // Setup the output directory for FSI data
    sysFSI.SetOutputDirectory(out_dir);

    // Create an initial box for the terrain patch
    chrono::utils::GridSampler<> sampler(initSpace0);

    // Use a chrono sampler to create a bucket of granular material
    ChVector<> boxCenter(0, 0, bzDim / 2);
    ChVector<> boxHalfDim(bxDim / 2, byDim / 2, bzDim / 2);
    std::vector<ChVector<>> points = sampler.SampleBox(boxCenter, boxHalfDim);

    // Add SPH particles from the sampler points to the FSI system
    size_t numPart = (int)points.size();
    double gz = std::abs(sysFSI.Get_G_acc().z());
    for (int i = 0; i < numPart; i++) {
        double pre_ini = sysFSI.GetDensity() * gz * (-points[i].z() + bzDim);
        double rho_ini = sysFSI.GetDensity() + pre_ini / (sysFSI.GetSoundSpeed() * sysFSI.GetSoundSpeed());
        sysFSI.AddSPHParticle(points[i], rho_ini, pre_ini, sysFSI.GetViscosity(), sysFSI.GetKernelLength(),
                              ChVector<>(0));
    }

    // Create MBD and BCE particles for the solid domain
    CreateSolidPhase(sysMBS, sysFSI);

    // Complete construction of the FSI system
    sysFSI.Initialize();

    // Set up integrator for the multi-body dynamics system
    sysMBS.SetTimestepperType(ChTimestepper::Type::HHT);
    auto mystepper = std::static_pointer_cast<ChTimestepperHHT>(sysMBS.GetTimestepper());
    mystepper->SetAlpha(-0.2);
    mystepper->SetMaxiters(1000);
    mystepper->SetAbsTolerances(1e-6);
    mystepper->SetMode(ChTimestepperHHT::ACCELERATION);
    mystepper->SetScaling(true);

    // Create a run-tme visualizer
    ChVisualizationFsi fsi_vis(&sysFSI);
    if (render) {
        fsi_vis.SetTitle("Chrono::FSI cylinder drop");
        auto origin = sysMBS.Get_bodylist()[1]->GetPos();
        fsi_vis.SetCameraPosition(origin - ChVector<>(0, 3 * byDim, 0), origin);
        fsi_vis.SetCameraMoveScale(1.0f);
        fsi_vis.EnableBoundaryMarkers(false);
        fsi_vis.Initialize();
    }

    // Start the simulation
    double dT = sysFSI.GetStepSize();
    unsigned int output_steps = (unsigned int)(1 / (out_fps * dT));
    unsigned int render_steps = (unsigned int)(1 / (render_fps * dT));

    double time = 0.0;
    int current_step = 0;

    ChTimer<> timer;
    timer.start();
    while (time < t_end) {
        std::cout << "step: " << current_step << "  time: " << time << std::endl;
        std::cout << "   box: " << sysMBS.Get_bodylist()[0]->GetPos() << std::endl;
        std::cout << "   cyl: " << sysMBS.Get_bodylist()[1]->GetPos() << std::endl;

        if (output && current_step % output_steps == 0) {
            std::cout << "-------- Output" << std::endl;
            sysFSI.PrintParticleToFile(out_dir + "/particles");
            static int counter = 0;
            std::string filename = out_dir + "/vtk/cylinder." + std::to_string(counter++) + ".vtk";
            WriteCylinderVTK(filename, cyl_radius, cyl_length, sysFSI.GetFsiBodies()[0]->GetFrame_REF_to_abs(), 100);
        }

        // Render SPH particles
        if (render && current_step % render_steps == 0) {
            if (!fsi_vis.Render())
                break;
        }

        // Call the FSI solver
        sysFSI.DoStepDynamics_FSI();
        time += dT;
        current_step++;
    }
    timer.stop();
    std::cout << "\nSimulation time: " << timer() << " seconds\n" << std::endl;

    return 0;
}
