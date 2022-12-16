// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2019 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Asher Elmquist
// =============================================================================
//
// Container class for a camera sensor. This specifies a default ray tracing
// for cameras.
//
// =============================================================================

#ifndef CHCAMERASENSOR_H
#define CHCAMERASENSOR_H

#include "chrono_sensor/sensors/ChOptixSensor.h"

namespace chrono {
namespace sensor {

/// @addtogroup sensor_sensors
/// @{

/// The type of lens model that camera can use for rendering
enum class CameraLensModelType {
    PINHOLE,  ///< traditional computer graphics ideal camera model.
    FOV_LENS  ///< Wide angle lens model based on single spherical lens.
};

/// Camera class
class CH_SENSOR_API ChCameraSensor : public ChOptixSensor {
  public:
    /// @brief Constructor for the base camera class that defaults to a pinhole lens model
    /// @param parent A shared pointer to a body on which the sensor should be attached.
    /// @param updateRate The desired update rate of the sensor in Hz.
    /// @param offsetPose The desired relative position and orientation of the sensor on the body.
    /// @param w The width of the image the camera should generate.
    /// @param h The height of the image the camera should generate.
    /// @param hFOV The horizontal field of view of the camera lens.
    /// @param supersample_factor The number of rays that should be sampled per pixel for antialiasing.
    /// @param lens_model A enum specifying the desired lens model.
    /// @param use_gi Enable the global illumination, with significant decrease in performace
    /// @param gamma correction of the image, 1 for linear color space, 2.2 for sRGB
    ChCameraSensor(std::shared_ptr<chrono::ChBody> parent,  // object to which the sensor is attached
                   float updateRate,                        // rate at which the sensor updates
                   chrono::ChFrame<double> offsetPose,      // position of sensor relative to parent object
                   unsigned int w,                          // image width
                   unsigned int h,                          // image height
                   float hFOV,                              // horizontal field of view
                   unsigned int supersample_factor = 1,     // number of samples per pixel for antialiasing
                   CameraLensModelType lens_model = CameraLensModelType::PINHOLE,
                   bool use_gi = false,  // camera model to use for rendering
                   float gamma = 2.2);   // gamma correction value

    /// camera class destructor
    ~ChCameraSensor();

    /// returns the camera's horizontal field of view. Vertical field of view is determined by the image aspect
    /// ratio and the lens model
    /// @return The horizontal field of view of the camera lens
    float GetHFOV() const { return m_hFOV; }

    /// returns the lens model type used for rendering
    /// @return An enum specifying which lens model is being used. (0: PINHOLE, 1: SPHERICAL)
    CameraLensModelType GetLensModelType() const { return m_lens_model_type; }

    /// returns if the cemera requesting global illumination
    /// @return True if it does request
    bool GetUseGI() { return m_use_gi; }

    /// returns the gamma correction value of this camera.
    /// 1 means no correction and the image is in linear color space. Useful for other ML applications
    /// 2.2 means the image is in sRGB color space. Useful for display
    /// @return Gamma value of the image
    float GetGamma() { return m_gamma; }

  private:
    float m_hFOV;                           ///< the horizontal field of view of the sensor
    unsigned int m_supersample_factor;      ///< super sampling factor for antialiasing
    CameraLensModelType m_lens_model_type;  ///< lens model used by the camera
    bool m_use_gi;                          ///< to hold reference to whether user what to use GI or not
    float m_gamma;
};

/// @} sensor_sensors

}  // namespace sensor
}  // namespace chrono

#endif
