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
// Base class for a TERRAIN NODE using a Chrono deformable soil formulation.
//
// The global reference frame has Z up, X towards the front of the vehicle, and
// Y pointing to the left.
//
// =============================================================================

#ifndef CH_VEHCOSIM_TERRAIN_NODE_CHRONO_H
#define CH_VEHCOSIM_TERRAIN_NODE_CHRONO_H

#include "chrono/ChConfig.h"

#include "chrono/physics/ChSystem.h"
#include "chrono/physics/ChSystemSMC.h"

#include "chrono_vehicle/cosim/ChVehicleCosimTerrainNode.h"

namespace chrono {
namespace vehicle {

/** @addtogroup vehicle_cosim_chrono
 *
 * This module defines concrete terrain nodes using Chrono physics:
 * - ChVehicleCosimTerrainNodeChrono is a base class (itself derived from ChVehicleCosimTerrainNode).
 * - ChVehicleCosimTerrainNodeRigid wraps a rigid terrain rectangular patch which interacts with tires through friction
 * and contact.
 * - ChVehicleCosimTerrainNodeSCM wraps an SCM deformable terrain rectangular patch.
 * - ChVehicleCosimTerrainNodeGranularOMP wraps a deformable terrain rectangular patch modeled with granular material
 * (using the Chrono::Multicore module).
 * - ChVehicleCosimTerrainNodeGranularGPU wraps a deformable terrain rectangular patch modeled with granular material
 * (using the Chrono::GPU module).
 * - ChVehicleCosimTerrainNodeGranularSPH wraps a deformable terrain rectangular patch modeled with granular material
 * (using the Chrono::FSI module).
 * - ChVehicleCosimTerrainNodeGranularMPI wraps a deformable terrain rectangular patch modeled with granular material
 * (using the Chrono::Distributed module).
 */

/// @addtogroup vehicle_cosim_chrono
/// @{

/// Base class for terrain nodes that use one of the Chrono terrain formulations.
class CH_VEHICLE_API ChVehicleCosimTerrainNodeChrono : public ChVehicleCosimTerrainNode {
  public:
    /// Type of Chrono terrain
    enum class Type {
        RIGID,         ///< rigid terrain
        SCM,           ///< Soil Contact Model
        GRANULAR_OMP,  ///< granular terrain (Chrono::Multicore)
        GRANULAR_GPU,  ///< granular terrain (Chrono::Gpu)
        GRANULAR_MPI,  ///< granular terrain (Chrono::Distributed)
        GRANULAR_SPH,  ///< continuous representation of granular terrain (Chrono::FSI)
        UNKNOWN        ///< unknown terrain type
    };

    /// Specification of a rigid obstacle.
    struct RigidObstacle {
        std::string m_mesh_filename;  ///< OBJ file with mesh specification
        double m_density;             ///< material density
        ChVector<> m_init_pos;        ///< initial position of obstacle
        ChQuaternion<> m_init_rot;    ///< initial orientation of obstacle
        ChVector<> m_oobb_center;     ///< center of bounding box
        ChVector<> m_oobb_dims;       ///< dimensions of bounding box
        MaterialInfo m_contact_mat;   ///< contact material parameters
    };

    virtual ~ChVehicleCosimTerrainNodeChrono() {}

    /// Return the type of this terrain node.
    Type GetType() const { return m_type; }

    /// Return a string describing the type of this terrain node.
    static std::string GetTypeAsString(Type type);

    /// Infer the terrain node type from the given string.
    static Type GetTypeFromString(const std::string& type);

    /// Read a JSON specification file for a Chrono terrain node.
    static bool ReadSpecfile(const std::string& specfile, rapidjson::Document& d);

    /// Get the terrain type from the given JSON specification file.
    static Type GetTypeFromSpecfile(const std::string& specfile);

    /// Get the terrain dimensions (length and width) from the given JSON specification file.
    static ChVector2<> GetSizeFromSpecfile(const std::string& specfile);

    /// Set the proxy bodies as fixed to ground.
    void SetProxyFixed(bool fixed) { m_fixed_proxies = fixed; }

    /// Return the terrain initial height.
    virtual double GetInitHeight() const override final { return m_init_height; }

    /// Add a rigid obstacle.
    void AddRigidObstacle(const RigidObstacle& obstacle);

  protected:
    /// Construct a base class terrain node.
    ChVehicleCosimTerrainNodeChrono(Type type,              ///< terrain type
                                    double length,          ///< terrain patch length
                                    double width,           ///< terain patch width
                                    ChContactMethod method  ///< contact method (SMC or NSC)
    );

    /// Initialize this Chrono terrain node.
    /// Construct the terrain system and the proxy bodies.
    virtual void OnInitialize(unsigned int num_tires) override;

    /// Advance simulation.
    /// This function is called after a synchronization to allow the node to advance
    /// its state by the specified time step.  A node is allowed to take as many internal
    /// integration steps as required, but no inter-node communication should occur.
    virtual void OnAdvance(double step_size) override;

    /// Return a pointer to the underlying Chrono system.
    virtual ChSystem* GetSystem() = 0;

    /// Construct the terrain (independent of the vehicle system).
    virtual void Construct() = 0;

    /// Create proxy body for the i-th tire.
    /// Use information in the m_mesh_data struct (vertex positions expressed in local frame).
    virtual void CreateWheelProxy(unsigned int i) = 0;

    /// Create proxy bodies for the i-th tire mesh.
    /// Use information in the m_mesh_data struct (vertex positions expressed in local frame).
    virtual void CreateMeshProxies(unsigned int i) {
        if (SupportsMeshInterface()) {
            throw ChException("Current terrain type does not support the MESH communication interface!");
        }
    }

  protected:
    /// Association between a proxy body and a mesh index.
    /// The body can be associated with either a mesh vertex or a mesh triangle.
    struct ProxyBody {
        ProxyBody(std::shared_ptr<ChBody> body, int index) : m_body(body), m_index(index) {}
        std::shared_ptr<ChBody> m_body;
        int m_index;
    };

    typedef std::vector<ProxyBody> Proxies;

    Type m_type;  ///< terrain type

    ChContactMethod m_method;                               ///< contact method (SMC or NSC)
    std::shared_ptr<ChMaterialSurface> m_material_terrain;  ///< material properties for terrain bodies

    double m_init_height;  ///< terrain initial height

    std::vector<Proxies> m_proxies;  ///< proxy bodies for each tire
    bool m_fixed_proxies;            ///< are proxy bodies fixed to ground?

    std::vector<RigidObstacle> m_obstacles;  ///< list of rigid obstacles
};

/// @} vehicle_cosim_chrono

}  // end namespace vehicle
}  // end namespace chrono

#endif
