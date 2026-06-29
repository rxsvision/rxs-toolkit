// rxs_api.cpp — Implementation of namespace rxs API layer
// Bridges the modern C++ API to the legacy algorithm code in czxToolkit.
//
// Copyright (c) 2026 Suzhou Ruixin Vision Technology Co., Ltd.
// Licensed under BSL 1.1 (Change Date: 2030-01-01)

// ---------------------------------------------------------------------------
// PCH is enabled for this target. We include it first for maximum benefit.
// If PCH is disabled, the individual includes below cover everything.
// ---------------------------------------------------------------------------
#include "pch.h"

#include <rxs/rxs_api.h>

#include <opencv2/opencv.hpp>
#include <pcl/common/transforms.h>
#include <pcl/registration/transformation_estimation_svd.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/search/kdtree.h>
#include <pcl/surface/convex_hull.h>

#ifdef _WIN32
#include <windows.h>
#endif

#include <algorithm>

// ===========================================================================
//  Forward declarations of legacy dllexported functions (defined in dllmain.cpp)
//  We call them directly since we're in the same DLL.
// ===========================================================================

// dllmain.cpp exports — we can call these directly as intra-DLL calls
extern std::vector<double> fitBSpline_(CP cloud, bool visual);
extern std::vector<pcl::PointXYZ> semiconductor(CP cloud, double min_x, double max_x, double min_y, double max_y);
extern std::vector<pcl::PointXYZ> PIN(CP cloud, double min_x, double max_x, double min_y, double max_y);
extern Eigen::Matrix4f registration(CP c1, CP c2, std::string hole_pos_path_x, std::string hole_pos_path_y);
extern void keyboard(CP cloud, std::string initial_plane_pos,
                     std::vector<std::string> verify_path,
                     std::vector<CP>& planes,
                     std::vector<std::vector<float>>& flatness,
                     Eigen::MatrixXf& offset);
extern pcl::PointCloud<pcl::PointXYZ>::Ptr computeConvexHull(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
extern double surfaceProfile(CP cloud, int zone_num);
extern double volume(CP cloud, double xyInterval);
extern void initModelNake(CP model);
extern std::vector<float> computeDeformationNake(CP model, CP cloud);
extern std::vector<float> LaptopCalculation(CP model, CP cloud, int type);
extern std::vector<float> dis2dis(CP model, CP source, float interval_x, float interval_y);
extern std::string colorReg(std::string model_root, std::string object_directory);
extern float Flatness(CP cloud, pcl::PointXYZ& min_point, pcl::PointXYZ& max_point);

// Photometric stereo exports (4 variants)
extern cv::Mat photometricStero(const std::vector<cv::Mat>& imgs,
                                const std::vector<float>& slants,
                                const std::vector<float>& tilts);
extern cv::Mat photometricSteroNormal(const std::vector<cv::Mat>& imgs,
                                      const std::vector<float>& slants,
                                      const std::vector<float>& tilts);
extern cv::Mat photometricSteroGray(const std::vector<cv::Mat>& imgs,
                                    const std::vector<float>& slants,
                                    const std::vector<float>& tilts);
extern cv::Mat photometricSteroHeights(const std::vector<cv::Mat>& imgs,
                                       const std::vector<float>& slants,
                                       const std::vector<float>& tilts);

// ===========================================================================
//  rxs:: namespace — Direct API implementations
//  These call the legacy functions directly (same DLL, zero indirection).
// ===========================================================================

namespace rxs {

// ---------------------------------------------------------------------------
//  Curve Fitting
// ---------------------------------------------------------------------------

std::vector<double> fitBSpline(CloudPtr cloud, bool visual) {
    return ::fitBSpline_(cloud, visual);
}

// ---------------------------------------------------------------------------
//  Feature Measurement
// ---------------------------------------------------------------------------

std::vector<PointT> semiconductor(CloudPtr cloud, const BaseFace& roi) {
    return ::semiconductor(cloud, roi.min_x, roi.max_x, roi.min_y, roi.max_y);
}

std::vector<PointT> measurePIN(CloudPtr cloud, const BaseFace& roi) {
    return ::PIN(cloud, roi.min_x, roi.max_x, roi.min_y, roi.max_y);
}

KeyboardResult keyboard(CloudPtr cloud,
                        const std::string& initial_plane_pos,
                        const std::vector<std::string>& verify_paths) {
    KeyboardResult result;
    ::keyboard(cloud, initial_plane_pos, verify_paths,
               result.planes, result.flatness, result.offset);
    return result;
}

double surfaceProfile(CloudPtr cloud, int zone_num) {
    return ::surfaceProfile(cloud, zone_num);
}

double volume(CloudPtr cloud, double xyInterval) {
    return ::volume(cloud, xyInterval);
}

FlatnessResult flatness(CloudPtr cloud) {
    FlatnessResult result;
    result.value = ::Flatness(cloud, result.min_point, result.max_point);
    return result;
}

// ---------------------------------------------------------------------------
//  Registration
// ---------------------------------------------------------------------------

Eigen::Matrix4f registration(CloudPtr source, CloudPtr target,
                             const std::string& hole_pos_x,
                             const std::string& hole_pos_y) {
    return ::registration(source, target, hole_pos_x, hole_pos_y);
}

CloudPtr rtConvert(CloudPtr cloud, const Eigen::Matrix4f& transform) {
    CloudPtr transformed(new CloudT);
    pcl::transformPointCloud(*cloud, *transformed, transform);
    return transformed;
}

// ---------------------------------------------------------------------------
//  Convex Hull
// ---------------------------------------------------------------------------

CloudPtr computeConvexHull(CloudPtr cloud) {
    return ::computeConvexHull(cloud);
}

// ---------------------------------------------------------------------------
//  Deformation & Laptop
// ---------------------------------------------------------------------------

void initModelNake(CloudPtr model) {
    ::initModelNake(model);
}

std::vector<float> computeDeformationNake(CloudPtr model, CloudPtr cloud) {
    return ::computeDeformationNake(model, cloud);
}

std::vector<float> laptopCalculation(CloudPtr model, CloudPtr cloud,
                                     LaptopType type) {
    return ::LaptopCalculation(model, cloud, static_cast<int>(type));
}

// ---------------------------------------------------------------------------
//  Distance Calculation
// ---------------------------------------------------------------------------

std::vector<float> dis2dis(CloudPtr model, CloudPtr source,
                           float interval_x, float interval_y) {
    return ::dis2dis(model, source, interval_x, interval_y);
}

// ---------------------------------------------------------------------------
//  Color Recognition
// ---------------------------------------------------------------------------

std::string colorReg(const std::string& model_root,
                     const std::string& object_directory) {
    return ::colorReg(model_root, object_directory);
}

// ---------------------------------------------------------------------------
//  Photometric Stereo
// ---------------------------------------------------------------------------

cv::Mat photometricStero(const std::vector<cv::Mat>& imgs,
                         const std::vector<float>& slants,
                         const std::vector<float>& tilts,
                         PhotometricMode mode) {
    switch (mode) {
    case PhotometricMode::Albedo:
        return ::photometricStero(imgs, slants, tilts);
    case PhotometricMode::Normal:
        return ::photometricSteroNormal(imgs, slants, tilts);
    case PhotometricMode::Gray:
        return ::photometricSteroGray(imgs, slants, tilts);
    case PhotometricMode::Heights:
        return ::photometricSteroHeights(imgs, slants, tilts);
    default:
        return ::photometricStero(imgs, slants, tilts);
    }
}

// ===========================================================================
//  ToolkitHandle — Dynamic loading implementation
// ===========================================================================

struct ToolkitHandle::Impl {
#ifdef _WIN32
    HMODULE hModule = nullptr;
#endif

    // Cached function pointers (resolved lazily on first call)
    // Using void* for storage; cast at call site
    void* fn_fitBSpline_           = nullptr;
    void* fn_semiconductor         = nullptr;
    void* fn_PIN                   = nullptr;
    void* fn_registration          = nullptr;
    void* fn_keyboard              = nullptr;
    void* fn_computeConvexHull     = nullptr;
    void* fn_surfaceProfile        = nullptr;
    void* fn_volume                = nullptr;
    void* fn_initModelNake         = nullptr;
    void* fn_computeDeformationNake = nullptr;
    void* fn_LaptopCalculation     = nullptr;
    void* fn_dis2dis               = nullptr;
    void* fn_colorReg              = nullptr;
    void* fn_Flatness              = nullptr;
    void* fn_photometricStero           = nullptr;
    void* fn_photometricSteroNormal     = nullptr;
    void* fn_photometricSteroGray       = nullptr;
    void* fn_photometricSteroHeights    = nullptr;

    bool loaded = false;

    void resolveAll() {
#ifdef _WIN32
        if (!hModule) return;
        fn_fitBSpline_            = GetProcAddress(hModule, "fitBSpline_");
        fn_semiconductor          = GetProcAddress(hModule, "semiconductor");
        fn_PIN                    = GetProcAddress(hModule, "PIN");
        fn_registration           = GetProcAddress(hModule, "registration");
        fn_keyboard               = GetProcAddress(hModule, "keyboard");
        fn_computeConvexHull      = GetProcAddress(hModule, "computeConvexHull");
        fn_surfaceProfile         = GetProcAddress(hModule, "surfaceProfile");
        fn_volume                 = GetProcAddress(hModule, "volume");
        fn_initModelNake          = GetProcAddress(hModule, "initModelNake");
        fn_computeDeformationNake = GetProcAddress(hModule, "computeDeformationNake");
        fn_LaptopCalculation      = GetProcAddress(hModule, "LaptopCalculation");
        fn_dis2dis                = GetProcAddress(hModule, "dis2dis");
        fn_colorReg               = GetProcAddress(hModule, "colorReg");
        fn_Flatness               = GetProcAddress(hModule, "Flatness");
        fn_photometricStero        = GetProcAddress(hModule, "photometricStero");
        fn_photometricSteroNormal  = GetProcAddress(hModule, "photometricSteroNormal");
        fn_photometricSteroGray    = GetProcAddress(hModule, "photometricSteroGray");
        fn_photometricSteroHeights = GetProcAddress(hModule, "photometricSteroHeights");
#endif
    }

    void clear() {
        fn_fitBSpline_ = fn_semiconductor = fn_PIN = fn_registration = nullptr;
        fn_keyboard = fn_computeConvexHull = fn_surfaceProfile = fn_volume = nullptr;
        fn_initModelNake = fn_computeDeformationNake = fn_LaptopCalculation = nullptr;
        fn_dis2dis = fn_colorReg = fn_Flatness = nullptr;
        fn_photometricStero = fn_photometricSteroNormal = nullptr;
        fn_photometricSteroGray = fn_photometricSteroHeights = nullptr;
        loaded = false;
    }
};

// --- Lifecycle ---

ToolkitHandle::ToolkitHandle()
    : pimpl(std::make_unique<Impl>()) {}

ToolkitHandle::~ToolkitHandle() {
    unload();
}

ToolkitHandle::ToolkitHandle(ToolkitHandle&&) noexcept = default;
ToolkitHandle& ToolkitHandle::operator=(ToolkitHandle&&) noexcept = default;

bool ToolkitHandle::load(const std::string& dll_path) {
#ifdef _WIN32
    unload();
    pimpl->hModule = LoadLibraryA(dll_path.c_str());
    if (pimpl->hModule) {
        pimpl->loaded = true;
        pimpl->resolveAll();
        return true;
    }
    return false;
#else
    (void)dll_path;
    return false;
#endif
}

void ToolkitHandle::unload() {
#ifdef _WIN32
    if (pimpl->hModule) {
        FreeLibrary(pimpl->hModule);
        pimpl->hModule = nullptr;
    }
#endif
    pimpl->clear();
}

bool ToolkitHandle::isLoaded() const {
    return pimpl->loaded;
}

bool ToolkitHandle::hasFunction(const std::string& func_name) const {
#ifdef _WIN32
    if (!pimpl->hModule) return false;
    return GetProcAddress(pimpl->hModule, func_name.c_str()) != nullptr;
#else
    (void)func_name;
    return false;
#endif
}

void* ToolkitHandle::getFunction(const std::string& func_name) const {
#ifdef _WIN32
    if (!pimpl->hModule) return nullptr;
    return reinterpret_cast<void*>(GetProcAddress(pimpl->hModule, func_name.c_str()));
#else
    (void)func_name;
    return nullptr;
#endif
}

std::vector<std::string> ToolkitHandle::getExportedFunctionNames() {
    return {
        "fitBSpline_", "semiconductor", "PIN", "registration",
        "keyboard", "computeConvexHull", "surfaceProfile", "volume",
        "initModelNake", "computeDeformationNake", "dis2dis", "colorReg",
        "Flatness", "LaptopCalculation", "photometricStero",
        "photometricSteroNormal", "photometricSteroGray", "photometricSteroHeights"
    };
}

// --- Convenience methods (call through cached function pointers) ---

std::vector<double> ToolkitHandle::fitBSpline(CloudPtr cloud, bool visual) {
    if (!pimpl->fn_fitBSpline_) return {};
    using FnT = std::vector<double>(*)(CP, bool);
    return reinterpret_cast<FnT>(pimpl->fn_fitBSpline_)(cloud, visual);
}

std::vector<PointT> ToolkitHandle::semiconductor(CloudPtr cloud, const BaseFace& roi) {
    if (!pimpl->fn_semiconductor) return {};
    using FnT = std::vector<PointT>(*)(CP, double, double, double, double);
    return reinterpret_cast<FnT>(pimpl->fn_semiconductor)(cloud, roi.min_x, roi.max_x, roi.min_y, roi.max_y);
}

std::vector<PointT> ToolkitHandle::measurePIN(CloudPtr cloud, const BaseFace& roi) {
    if (!pimpl->fn_PIN) return {};
    using FnT = std::vector<PointT>(*)(CP, double, double, double, double);
    return reinterpret_cast<FnT>(pimpl->fn_PIN)(cloud, roi.min_x, roi.max_x, roi.min_y, roi.max_y);
}

Eigen::Matrix4f ToolkitHandle::registration(CloudPtr source, CloudPtr target,
                                             const std::string& hole_pos_x,
                                             const std::string& hole_pos_y) {
    if (!pimpl->fn_registration) return Eigen::Matrix4f::Identity();
    using FnT = Eigen::Matrix4f(*)(CP, CP, std::string, std::string);
    return reinterpret_cast<FnT>(pimpl->fn_registration)(source, target, hole_pos_x, hole_pos_y);
}

KeyboardResult ToolkitHandle::keyboard(CloudPtr cloud,
                                       const std::string& initial_plane_pos,
                                       const std::vector<std::string>& verify_paths) {
    KeyboardResult result;
    if (!pimpl->fn_keyboard) return result;
    using FnT = void(*)(CP, std::string, std::vector<std::string>,
                        std::vector<CP>&, std::vector<std::vector<float>>&, Eigen::MatrixXf&);
    reinterpret_cast<FnT>(pimpl->fn_keyboard)(cloud, initial_plane_pos, verify_paths,
                                               result.planes, result.flatness, result.offset);
    return result;
}

double ToolkitHandle::surfaceProfile(CloudPtr cloud, int zone_num) {
    if (!pimpl->fn_surfaceProfile) return -1.0;
    using FnT = double(*)(CP, int);
    return reinterpret_cast<FnT>(pimpl->fn_surfaceProfile)(cloud, zone_num);
}

CloudPtr ToolkitHandle::computeConvexHull(CloudPtr cloud) {
    if (!pimpl->fn_computeConvexHull) return nullptr;
    using FnT = CloudPtr(*)(CloudPtr);
    return reinterpret_cast<FnT>(pimpl->fn_computeConvexHull)(cloud);
}

double ToolkitHandle::volume(CloudPtr cloud, double xyInterval) {
    if (!pimpl->fn_volume) return 0.0;
    using FnT = double(*)(CP, double);
    return reinterpret_cast<FnT>(pimpl->fn_volume)(cloud, xyInterval);
}

void ToolkitHandle::initModelNake(CloudPtr model) {
    if (!pimpl->fn_initModelNake) return;
    using FnT = void(*)(CP);
    reinterpret_cast<FnT>(pimpl->fn_initModelNake)(model);
}

std::vector<float> ToolkitHandle::computeDeformationNake(CloudPtr model, CloudPtr cloud) {
    if (!pimpl->fn_computeDeformationNake) return {};
    using FnT = std::vector<float>(*)(CP, CP);
    return reinterpret_cast<FnT>(pimpl->fn_computeDeformationNake)(model, cloud);
}

std::vector<float> ToolkitHandle::laptopCalculation(CloudPtr model, CloudPtr cloud,
                                                     LaptopType type) {
    if (!pimpl->fn_LaptopCalculation) return {};
    using FnT = std::vector<float>(*)(CP, CP, int);
    return reinterpret_cast<FnT>(pimpl->fn_LaptopCalculation)(model, cloud, static_cast<int>(type));
}

std::vector<float> ToolkitHandle::dis2dis(CloudPtr model, CloudPtr source,
                                           float interval_x, float interval_y) {
    if (!pimpl->fn_dis2dis) return {};
    using FnT = std::vector<float>(*)(CP, CP, float, float);
    return reinterpret_cast<FnT>(pimpl->fn_dis2dis)(model, source, interval_x, interval_y);
}

std::string ToolkitHandle::colorReg(const std::string& model_root,
                                    const std::string& object_directory) {
    if (!pimpl->fn_colorReg) return "";
    using FnT = std::string(*)(std::string, std::string);
    return reinterpret_cast<FnT>(pimpl->fn_colorReg)(model_root, object_directory);
}

FlatnessResult ToolkitHandle::flatness(CloudPtr cloud) {
    FlatnessResult result;
    if (!pimpl->fn_Flatness) return result;
    using FnT = float(*)(CP, PointT&, PointT&);
    result.value = reinterpret_cast<FnT>(pimpl->fn_Flatness)(cloud, result.min_point, result.max_point);
    return result;
}

CloudPtr ToolkitHandle::rtConvert(CloudPtr cloud, const Eigen::Matrix4f& transform) {
    // rtConvert is not a legacy export — use PCL directly
    // (This method is provided for API completeness via ToolkitHandle)
    CloudPtr transformed(new CloudT);
    pcl::transformPointCloud(*cloud, *transformed, transform);
    return transformed;
}

} // namespace rxs
