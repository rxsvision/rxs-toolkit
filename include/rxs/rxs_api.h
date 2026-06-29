#pragma once

/// @file rxs_api.h
/// @brief RXS Toolkit Modern C++ API (namespace rxs)
/// @author Suzhou Ruixin Vision Technology Co., Ltd.
/// @date 2026-06-29
///
/// This header provides a clean, namespace-scoped API layer over the legacy
/// czxToolkit DLL exports. It enables type-safe usage without GetProcAddress
/// boilerplate and serves as the migration path toward a header-only library.

#include <vector>
#include <string>
#include <memory>
#include <cstdint>
#include <Eigen/Dense>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

/// @brief RXS Vision namespace
namespace rxs {

// ---------------------------------------------------------------------------
// Type aliases
// ---------------------------------------------------------------------------
using PointT = pcl::PointXYZ;
using CloudT = pcl::PointCloud<PointT>;
using CloudPtr = CloudT::Ptr;

/// @brief Result report row (replaces rxsResultReport)
struct ResultRow {
    int id = 0;
    std::string name;
    std::map<std::string, std::string> values;
    bool pass = true;
};

/// @brief Base face descriptor (replaces BaseFace)
struct BaseFace {
    double min_x = 0;
    double max_x = 0;
    double min_y = 0;
    double max_y = 0;
};

/// @brief Deformation direction classification
enum class DeformationType : int {
    Up    = 0,
    Down  = 1,
    Other = 2,
    Empty = 3,
};

/// @brief Laptop type enumeration
enum class LaptopType : int {
    BlueLaptopWithHoles   = 0,
    BlueLaptop            = 1,
    Keyboard              = 2,
    AssembledKeyboard     = 3,
    AssembledBackplate    = 4,
    AssembledBackplatePre = 5,
    AssembledKeyboardPre  = 6,
};

// ---------------------------------------------------------------------------
// Algorithm categories
// ---------------------------------------------------------------------------

/// @defgroup curve Curve Fitting
/// @{

/// @brief B-spline curve fitting
/// @param cloud Input point cloud
/// @param debug Enable debug visualization
/// @return [arc_length, projection_length, profile_deviation]
std::vector<double> fitBSpline(CloudPtr cloud, bool debug = false);

/// @}

/// @defgroup feature Feature Measurement
/// @{

/// @brief Semiconductor feature measurement
/// @param cloud Input point cloud
/// @param roi Region of interest bounding box
/// @return Detected feature points
std::vector<PointT> semiconductor(CloudPtr cloud, const BaseFace& roi);

/// @brief PIN pin measurement
/// @param cloud Input point cloud
/// @param roi Region of interest bounding box
/// @return Detected PIN positions
std::vector<PointT> measurePIN(CloudPtr cloud, const BaseFace& roi);

/// @brief Keyboard feature extraction
/// @param cloud Input point cloud
/// @param initial_plane_pos Initial plane position file
/// @param verify_paths Verification plane files
/// @return [planes, flatness_values, offset_matrix]
struct KeyboardResult {
    std::vector<CloudPtr> planes;
    std::vector<std::vector<float>> flatness;
    Eigen::MatrixXf offset;
};
KeyboardResult keyboard(CloudPtr cloud,
                        const std::string& initial_plane_pos,
                        const std::vector<std::string>& verify_paths);

/// @brief Surface profile measurement
/// @param cloud Input point cloud
/// @param resolution Grid resolution (default: 12)
/// @return Surface profile deviation (mm)
double surfaceProfile(CloudPtr cloud, int resolution = 12);

/// @brief Volume calculation
/// @param cloud Input point cloud
/// @param xyInterval Grid interval (default: 12)
/// @return Calculated volume
double volume(CloudPtr cloud, double xyInterval = 12.0);

/// @brief Flatness calculation
/// @param cloud Input point cloud
/// @return Flatness value (mm)
double flatness(CloudPtr cloud);

/// @}

/// @defgroup registration Registration
/// @{

/// @brief Point cloud registration
/// @param source Source point cloud
/// @param target Target point cloud
/// @param hole_pos_x X-axis hole position file
/// @param hole_pos_y Y-axis hole position file
/// @return 4x4 transformation matrix
Eigen::Matrix4f registration(CloudPtr source, CloudPtr target,
                             const std::string& hole_pos_x,
                             const std::string& hole_pos_y);

/// @brief Apply RT transformation
/// @param cloud Point cloud to transform
/// @param transform 4x4 transformation matrix
/// @return Transformed point cloud
CloudPtr rtConvert(CloudPtr cloud, const Eigen::Matrix4f& transform);

/// @}

/// @defgroup hull Convex Hull
/// @{

/// @brief Compute convex hull (PCL wrapper)
/// @param cloud Input point cloud
/// @return Convex hull point cloud
/// @note This is a thin wrapper around PCL's convex hull implementation.
CloudPtr computeConvexHull(CloudPtr cloud);

/// @}

/// @defgroup deformation Deformation
/// @{

/// @brief Compute deformation (naked)
/// @param model Reference model point cloud
/// @param cloud Measured point cloud
/// @return [x_offset, y_offset, z_offset, deformation_v1, deformation_v2,
///          measured_length, projection_length, deformation_type]
std::vector<float> computeDeformation(CloudPtr model, CloudPtr cloud);

/// @brief Laptop calculation
/// @param model Reference model
/// @param cloud Measured cloud
/// @param type Laptop type
/// @param check_flag Auxiliary check flag
/// @param aux_x Auxiliary X positions
/// @param aux_z Auxiliary Z values
/// @param aux_ret Auxiliary check results (output)
/// @return [deformation, sharpness, length, gap1, gap2, ...]
std::vector<float> laptopCalculation(
    CloudPtr model, CloudPtr cloud,
    const std::string& type, int check_flag,
    const std::vector<float>& aux_x,
    const std::vector<float>& aux_z,
    std::vector<bool>& aux_ret);

/// @}

/// @defgroup photometric Photometric Stereo
/// @{

/// @brief Photometric stereo reconstruction
/// @param images Input images
/// @param light_dirs Light directions
/// @return Normal map and height map
struct PhotometricStereoResult {
    std::vector<float> normals;
    std::vector<float> heights;
};

PhotometricStereoResult photometricStero(
    const std::vector<std::string>& image_paths,
    const std::string& config_path);

/// @}

/// @defgroup distance Distance Calculation
/// @{

/// @brief Dis2Dis distance calculation
/// @param model Reference model
/// @param source Source point cloud
/// @param interval_x X-axis sampling interval
/// @param interval_y Y-axis sampling interval
/// @return Distance vector
std::vector<float> dis2dis(CloudPtr model, CloudPtr source,
                           float interval_x = 20.0f,
                           float interval_y = 20.0f);

/// @}

// ---------------------------------------------------------------------------
// Factory & lifecycle
// ---------------------------------------------------------------------------

/// @brief Toolkit handle for DLL-based operation
/// @deprecated Direct API calls are preferred. This exists for backward
///             compatibility with LoadLibrary-based consumers.
class ToolkitHandle {
public:
    ToolkitHandle();
    ~ToolkitHandle();

    /// @brief Load the czxToolkit DLL
    /// @param dll_path Path to czxToolkit.dll
    /// @return true if loaded successfully
    bool load(const std::string& dll_path = "czxToolkit.dll");

    /// @brief Check if a specific function is available
    /// @param func_name Function name to check
    /// @return true if the function exists in the DLL
    bool hasFunction(const std::string& func_name) const;

    /// @brief Get list of all exported functions
    /// @return Vector of function names
    std::vector<std::string> getExportedFunctions() const;

private:
    struct Impl;
    std::unique_ptr<Impl> pimpl;
};

} // namespace rxs
