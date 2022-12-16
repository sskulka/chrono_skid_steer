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
// Definition of the rigid TERRAIN NODE (using Chrono::Multicore).
//
// The global reference frame has Z up, X towards the front of the vehicle, and
// Y pointing to the left.
//
// =============================================================================

#ifndef CH_VEHCOSIM_TERRAIN_NODE_RIGID_H
#define CH_VEHCOSIM_TERRAIN_NODE_RIGID_H

#include "chrono/ChConfig.h"
#include "chrono_multicore/physics/ChSystemMulticore.h"

#include "chrono_vehicle/cosim/terrain/ChVehicleCosimTerrainNodeChrono.h"

#ifdef CHRONO_OPENGL
    #include "chrono_opengl/ChVisualSystemOpenGL.h"
#endif

#include "chrono_thirdparty/rapidjson/document.h"

namespace chrono {
namespace vehicle {

/// @addtogroup vehicle_cosim_chrono
/// @{

/// Definition of the rigid terrain node (using Chrono::Multicore).
class CH_VEHICLE_API ChVehicleCosimTerrainNodeRigid : public ChVehicleCosimTerrainNodeChrono {
  public:
    /// Create a rigid terrain node using the specified contact method (SMC or NSC).
    ChVehicleCosimTerrainNodeRigid(double length, double width, ChContactMethod method);

    /// Create a rigid terrain node using the specified contact method (SMC or NSC) and set parameters from the provided
    /// JSON specfile.
    ChVehicleCosimTerrainNodeRigid(ChContactMethod method, const std::string& specfile);

    ~ChVehicleCosimTerrainNodeRigid();

    virtual ChSystem* GetSystem() override { return m_system; }

    /// Set full terrain specification from JSON specfile.
    void SetFromSpecfile(const std::string& specfile);

    /// Set the material properties for terrain.
    /// The type of material must be consistent with the contact method (SMC or NSC)
    /// specified at construction. These parameters characterize the material for the container and
    /// (if applicable) the granular material.  Tire contact material is received from the rig node.
    void SetMaterialSurface(const std::shared_ptr<ChMaterialSurface>& mat);

    /// Specify whether contact coefficients are based on material properties (default: true).
    /// Note that this setting is only relevant when using the SMC method.
    void UseMaterialProperties(bool flag);

    /// Set the normal contact force model (default: Hertz)
    /// Note that this setting is only relevant when using the SMC method.
    void SetContactForceModel(ChSystemSMC::ContactForceModel model);

    /// Set proxy contact radius (default: 0.01).
    /// When using a rigid tire mesh, this is a "thickness" for the collision mesh (a non-zero value can improve
    /// robustness of the collision detection algorithm).  When using a flexible tire, this is the radius of the proxy
    /// spheres attached to each FEA mesh node.
    void SetProxyContactRadius(double radius) { m_radius_p = radius; }

    /// Output post-processing visualization data.
    virtual void OutputVisualizationData(int frame) override final;

  private:
    ChSystemMulticore* m_system;  ///< containing system
    double m_radius_p;            ///< radius for a proxy body

#ifdef CHRONO_OPENGL
    opengl::ChVisualSystemOpenGL* m_vsys;  ///< OpenGL visualization system
#endif

    virtual bool SupportsMeshInterface() const override { return true; }

    virtual void Construct() override;

    /// Return current total number of contacts.
    virtual int GetNumContacts() const override { return m_system->GetNcontacts(); }

    virtual void CreateMeshProxies(unsigned int i) override;
    virtual void UpdateMeshProxies(unsigned int i, MeshState& mesh_state) override;
    virtual void GetForcesMeshProxies(unsigned int i, MeshContact& mesh_contact) override;
    void PrintMeshProxiesUpdateData(unsigned int i, const MeshState& mesh_state);

    virtual void CreateWheelProxy(unsigned int i) override;
    virtual void UpdateWheelProxy(unsigned int i, BodyState& spindle_state) override;
    virtual void GetForceWheelProxy(unsigned int i, TerrainForce& wheel_contact) override;

    virtual void OnAdvance(double step_size) override;
    virtual void Render(double time) override;
};

/// @} vehicle_cosim_chrono

}  // end namespace vehicle
}  // end namespace chrono

#endif
