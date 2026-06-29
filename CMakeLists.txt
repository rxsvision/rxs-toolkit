cmake_minimum_required(VERSION 3.20)

project(rxs-toolkit VERSION 1.0.0 LANGUAGES CXX)

# ---------------------------------------------------------------------------
# Options
# ---------------------------------------------------------------------------
option(RXS_BUILD_CUDA "Enable CUDA support for photometricStero" OFF)
option(RXS_BUILD_TESTS "Build test executables" OFF)
option(RXS_USE_PCH "Use precompiled headers" ON)

# ---------------------------------------------------------------------------
# C++ Standard
# ---------------------------------------------------------------------------
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# ---------------------------------------------------------------------------
# Find dependencies
# ---------------------------------------------------------------------------
find_package(PCL 1.10 REQUIRED COMPONENTS
    common features filters io io_ply kdtree sample_consensus search
    segmentation surface registration visualization
)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(FLANN REQUIRED)
find_package(Boost REQUIRED COMPONENTS system filesystem)

# OpenMP
find_package(OpenMP REQUIRED)

# CUDA (optional)
if(RXS_BUILD_CUDA)
    enable_language(CUDA)
    find_package(CUDAToolkit REQUIRED)
endif()

# ---------------------------------------------------------------------------
# Source files
# ---------------------------------------------------------------------------
set(RXS_TOOLKIT_SOURCES
    dllmain.cpp
    BSplineLength.cpp
    ColorModel.cpp
    CzxRansac.cpp
    Hole.cpp
    LaptopLine.cpp
    NurbsFit.cpp
    NurbsSolve.cpp
    NurbsSurface.cpp
    OMPSurface.cpp
    photometricStero.cpp
    PointGrid.cpp
    utils.cpp
    czxDependence/czxTool.cpp
)

set(RXS_TOOLKIT_HEADERS
    framework.h
    pch.h
    types_.h
    utlis.h
    BSplineLength.h
    ColorModel.h
    CzxRansac.h
    Hole.h
    Hole.hpp
    LaptopLine.h
    NurbsFit.h
    NurbsSolve.h
    NurbsSurface.h
    OMPSurface.h
    photometricStero.h
    PointGrid.h
    nanoflann.hpp
    czxDependence/czxTool.h
)

# CUDA source (conditional)
if(RXS_BUILD_CUDA)
    list(APPEND RXS_TOOLKIT_SOURCES photometricStero.cu)
    list(APPEND RXS_TOOLKIT_HEADERS photometricStero_cu.h)
endif()

# ---------------------------------------------------------------------------
# Create shared library
# ---------------------------------------------------------------------------
add_library(rxs-toolkit SHARED ${RXS_TOOLKIT_SOURCES} ${RXS_TOOLKIT_HEADERS})

set_target_properties(rxs-toolkit PROPERTIES
    OUTPUT_NAME "czxToolkit"
    WINDOWS_EXPORT_ALL_SYMBOLS OFF
    CXX_VISIBILITY_PRESET hidden
    VISIBILITY_INLINES_HIDDEN ON
)

# Module definition file
set_target_properties(rxs-toolkit PROPERTIES
    LINK_FLAGS "/DEF:Source.def"
)

# ---------------------------------------------------------------------------
# Precompiled header
# ---------------------------------------------------------------------------
if(RXS_USE_PCH)
    target_precompile_headers(rxs-toolkit PRIVATE pch.h)
    # Files that should NOT use PCH
    set_source_files_properties(
        BSplineLength.cpp
        ColorModel.cpp
        CzxRansac.cpp
        Hole.cpp
        LaptopLine.cpp
        NurbsFit.cpp
        NurbsSolve.cpp
        NurbsSurface.cpp
        OMPSurface.cpp
        photometricStero.cpp
        PointGrid.cpp
        utils.cpp
        czxDependence/czxTool.cpp
        PROPERTIES SKIP_PRECOMPILE_HEADERS ON
    )
endif()

# ---------------------------------------------------------------------------
# Include directories
# ---------------------------------------------------------------------------
target_include_directories(rxs-toolkit PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/czxDependence
    ${PCL_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
    ${FLANN_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
)

# ---------------------------------------------------------------------------
# Compile definitions
# ---------------------------------------------------------------------------
target_compile_definitions(rxs-toolkit PRIVATE
    CZXTOOLKIT_EXPORTS
    _WINDOWS
    _USRDLL
    NOMINMAX
    BOOST_USE_WINDOWS_H
    PCL_NO_PRECOMPILE
    _CRT_SECURE_NO_WARNINGS
    _SCL_SECURE_NO_WARNINGS
)

if(RXS_BUILD_CUDA)
    target_compile_definitions(rxs-toolkit PRIVATE RXS_ENABLE_CUDA)
endif()

# ---------------------------------------------------------------------------
# Link libraries
# ---------------------------------------------------------------------------
target_link_libraries(rxs-toolkit PRIVATE
    ${PCL_LIBRARIES}
    ${OpenCV_LIBS}
    ${FLANN_LIBRARIES}
    ${Boost_LIBRARIES}
    OpenMP::OpenMP_CXX
)

if(RXS_BUILD_CUDA)
    target_link_libraries(rxs-toolkit PRIVATE CUDA::cudart)
    set_target_properties(rxs-toolkit PROPERTIES CUDA_SEPARABLE_COMPILATION ON)
endif()

# ---------------------------------------------------------------------------
# Install
# ---------------------------------------------------------------------------
include(GNUInstallDirs)

install(TARGETS rxs-toolkit
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
)

install(FILES ${RXS_TOOLKIT_HEADERS}
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/rxs-toolkit
)

# ---------------------------------------------------------------------------
# Summary
# ---------------------------------------------------------------------------
message(STATUS "=== rxs-toolkit Configuration ===")
message(STATUS "  C++ Standard:  ${CMAKE_CXX_STANDARD}")
message(STATUS "  CUDA:          ${RXS_BUILD_CUDA}")
message(STATUS "  PCH:           ${RXS_USE_PCH}")
message(STATUS "  PCL:           ${PCL_VERSION}")
message(STATUS "  OpenCV:        ${OpenCV_VERSION}")
message(STATUS "  Source files:  ${RXS_TOOLKIT_SOURCES}")
message(STATUS "=================================")
