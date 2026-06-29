#pragma once

/// @file rxs_export.h
/// @brief DLL export/import macros for RXS Toolkit
/// @author Suzhou Ruixin Vision Technology Co., Ltd.
/// @date 2026-06-29
///
/// This header defines the RXS_API macro used to mark functions and classes
/// for DLL export (when building the library) or import (when consuming).
/// It also defines RXS_LOCAL for symbols that should remain internal.

#ifdef RXS_TOOLKIT_STATIC
    // Static linkage — no DLL annotations
    #define RXS_API
    #define RXS_LOCAL
#elif defined(RXS_TOOLKIT_EXPORTS)
    // Building the DLL — export symbols
    #define RXS_API __declspec(dllexport)
    #define RXS_LOCAL
#else
    // Consuming the DLL — import symbols
    #define RXS_API __declspec(dllimport)
    #define RXS_LOCAL
#endif
