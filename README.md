# RXS Toolkit

> 核心公共库 + DLL 导出层 — 苏州锐新视科技有限公司 (RXS Vision)

## 概述

RXS Toolkit 是锐新视 3D 测量算法库的核心公共组件，提供 18 个 DLL 导出函数，涵盖 NURBS 曲面拟合、点云配准、几何特征计算、光度立体重建等工业 AI 视觉算法。

Phase 5 新增 **namespace rxs 现代 C++ API 层**，在保持 18 个 C 风格导出函数完全向后兼容的同时，提供类型安全、命名空间隔离的 C++ 接口。

## 两种调用方式

### 方式一：静态链接（推荐新代码使用）

链接 `czxToolkit.lib` 导入库，直接调用 `rxs::` 命名空间函数：

```cpp
#include <rxs/rxs_api.h>

// B样条曲线拟合
auto result = rxs::fitBSpline(cloud, false);
// result = [arc_length, projection_length, profile_deviation]

// 平面度计算
auto flat = rxs::flatness(plane_cloud);
// flat.value = 平面度值 (mm)
// flat.min_point / flat.max_point = 极值点

// 体积计算
double vol = rxs::volume(cloud, 12.0);

// 半导体特征测量
rxs::BaseFace roi{0.0, 100.0, 0.0, 100.0};
auto features = rxs::semiconductor(cloud, roi);

// 笔记本计算
auto deform = rxs::laptopCalculation(model, cloud, rxs::LaptopType::BlueLaptopWithHoles);
// deform = [deformation, sharpness, 2D_length, left_gap, right_gap, ...]
```

### 方式二：动态加载（向后兼容）

使用 `ToolkitHandle` 通过 `LoadLibrary`/`GetProcAddress` 动态加载：

```cpp
#include <rxs/rxs_api.h>

rxs::ToolkitHandle toolkit;
if (!toolkit.load("czxToolkit.dll")) {
    std::cerr << "DLL加载失败\n";
    return;
}

// 检查函数是否可用
if (toolkit.hasFunction("fitBSpline_")) {
    auto result = toolkit.fitBSpline(cloud, false);
}

// 检查所有18个导出函数
for (const auto& name : rxs::ToolkitHandle::getExportedFunctionNames()) {
    std::cout << name << ": " << toolkit.hasFunction(name) << "\n";
}
```

## DLL 导出函数清单 (18个)

| # | 导出名 | rxs:: API 等价 | 功能 | 源文件 |
|---|--------|---------------|------|--------|
| 1 | `fitBSpline_` | `rxs::fitBSpline()` | B样条曲线拟合 | dllmain.cpp |
| 2 | `semiconductor` | `rxs::semiconductor()` | 半导体特征测量 | dllmain.cpp |
| 3 | `PIN` | `rxs::measurePIN()` | PIN引脚测量 | dllmain.cpp |
| 4 | `registration` | `rxs::registration()` | 点云配准(3孔SVD) | dllmain.cpp |
| 5 | `keyboard` | `rxs::keyboard()` | 键盘/按键特征提取 | dllmain.cpp |
| 6 | `computeConvexHull` | `rxs::computeConvexHull()` | 凸包计算(PCL) | dllmain.cpp |
| 7 | `surfaceProfile` | `rxs::surfaceProfile()` | 曲面轮廓提取 | dllmain.cpp |
| 8 | `volume` | `rxs::volume()` | 体积计算 | dllmain.cpp |
| 9 | `initModelNake` | `rxs::initModelNake()` | 形变模型初始化 | dllmain.cpp |
| 10 | `computeDeformationNake` | `rxs::computeDeformationNake()` | 形变计算(裸模式) | dllmain.cpp |
| 11 | `dis2dis` | `rxs::dis2dis()` | 距离矩阵计算 | dllmain.cpp |
| 12 | `colorReg` | `rxs::colorReg()` | 颜色配准识别 | dllmain.cpp |
| 13 | `Flatness` | `rxs::flatness()` | 平面度计算 | dllmain.cpp |
| 14 | `LaptopCalculation` | `rxs::laptopCalculation()` | 笔记本计算(完整) | dllmain.cpp |
| 15 | `photometricStero` | `rxs::photometricStero(Albedo)` | 光度立体(反照率) | dllmain.cpp |
| 16 | `photometricSteroNormal` | `rxs::photometricStero(Normal)` | 光度立体(法线) | dllmain.cpp |
| 17 | `photometricSteroGray` | `rxs::photometricStero(Gray)` | 光度立体(灰度) | dllmain.cpp |
| 18 | `photometricSteroHeights` | `rxs::photometricStero(Heights)` | 光度立体(高度) | dllmain.cpp |

> **注**: `rxs::rtConvert()` 是新增的便捷函数，基于 PCL `transformPointCloud` 实现，不是遗留导出。通过 `ToolkitHandle` 调用时直接使用 PCL 实现。

> **holeDeepth 重建**: `holeDeepth` 原为幽灵引用 — `GetProcAddress(dllHander, "holeDeepth")` 返回 NULL，DLL 从未导出此函数。Phase 1 已清除消费者侧 24 处无效引用，同时在 `rxs-algorithms/modules/hole_depth/` 重建为独立算法模块。详见 [rxs-algorithms/modules/hole_depth/README.md](https://github.com/rxsvision/rxs-algorithms/tree/main/modules/hole_depth)。

## namespace rxs API 层架构

```
┌──────────────────────────────────────────────────┐
│                消费者代码                          │
├────────────────────┬─────────────────────────────┤
│  静态链接 (#include) │  动态加载 (ToolkitHandle)     │
│  rxs::fitBSpline()  │  toolkit.fitBSpline()       │
├────────────────────┼─────────────────────────────┤
│  rxs_api.h (声明)   │  ToolkitHandle::Impl         │
│  rxs_export.h (宏)  │  LoadLibrary / GetProcAddress│
├────────────────────┴─────────────────────────────┤
│           rxs_api.cpp (实现)                       │
│  - 直接调用遗留函数 (::fitBSpline_ 等)              │
│  - PCL直接调用 (rtConvert)                         │
├──────────────────────────────────────────────────┤
│           dllmain.cpp (遗留导出层)                  │
│  __declspec(dllexport) × 18 函数                   │
│  Source.def 导出定义                               │
├──────────────────────────────────────────────────┤
│     算法类 (BSplineLength, Hole, LaptopLine,      │
│      OMPSurface, PointGrid, ColorModel, etc.)     │
└──────────────────────────────────────────────────┘
```

### 设计原则

1. **零破坏性**：18个 C 风格导出函数完全不变，现有消费者代码无需修改
2. **零间接**：`rxs::` 函数与遗留函数在同一 DLL 内，直接调用无额外开销
3. **类型安全**：`BaseFace` 替代4个裸 double 参数，`FlatnessResult` 替代输出引用参数
4. **枚举安全**：`LaptopType` / `PhotometricMode` 强类型枚举替代裸 int
5. **RAII**：`ToolkitHandle` 支持移动语义，析构自动 FreeLibrary

## 目录结构

```
rxs-toolkit/
├── .github/workflows/
│   └── encoding-check.yml    # CI: 检测 GBK 编码回归
├── include/rxs/               # Phase 5: 公共 API 头文件
│   ├── rxs_export.h          #   DLL 导出/导入宏
│   └── rxs_api.h             #   namespace rxs API 声明
├── src/                       # Phase 5: API 实现层
│   └── rxs_api.cpp           #   rxs:: 函数 + ToolkitHandle 实现
├── czxDependence/
│   ├── czxTool.h             # 公共工具库 (Timer/Log/Ransac/readProfile)
│   ├── czxTool.cpp           # v8 最新版 (13,885 bytes)
│   ├── cuda.props            # CUDA 构建属性
│   └── pcl_release.props     # PCL Release 构建属性
├── dllmain.cpp               # 生产 DLL 导出 (18 函数)
├── dllmain-d.cpp             # 调试 DLL 导出 (10 函数, #define CZX_DEBUG)
├── Source.def                # DLL 导出定义文件 (18 EXPORTS)
├── BSplineLength.cpp/h       # B样条曲线长度计算
├── ColorModel.cpp/h          # 颜色模型
├── CzxRansac.cpp/h           # RANSAC 随机采样一致性
├── Hole.cpp/h/hpp            # 孔特征检测
├── LaptopLine.cpp/h          # 笔记本线条计算
├── NurbsFit.cpp/h            # NURBS 拟合
├── NurbsSolve.cpp/h          # NURBS 求解器
├── NurbsSurface.cpp/h        # NURBS 曲面
├── OMPSurface.cpp/h          # OMP 并行曲面
├── PointGrid.cpp/h           # 点云网格化
├── photometricStero.cpp/cu/h # 光度立体 (CUDA)
├── nanoflann.hpp             # 头文件 KD-Tree 库
├── utils.cpp / utlis.h       # 工具函数
├── types_.h                  # 类型定义
├── framework.h / pch.h       # 预编译头
├── conf_surface.czx          # 曲面配置文件
├── vcpkg.json                # 依赖管理 (PCL/OpenCV/Eigen/FLANN/Boost)
├── LICENSE                   # BSL 1.1 (Change Date: 2030-01-01)
└── README.md
```

## 构建

### 依赖

- Visual Studio 2022
- PCL >= 1.14.0 (via vcpkg)
- OpenCV >= 4.8.0 (via vcpkg)
- Eigen3 >= 3.4.0
- CUDA Toolkit (可选, 用于 photometricStero)
- Qt (用于工具类, 不强制)

### vcpkg 安装

```bash
vcpkg install pcl opencv4 eigen3 flann boost-system boost-filesystem
```

### CMake 构建（推荐）

```bash
mkdir build && cd build
cmake .. -DCMAKE_TOOLCHAIN_FILE=[vcpkg]/scripts/buildsystems/vcpkg.cmake
cmake --build . --config Release
```

### VS2022 构建（传统）

1. 打开 `czxToolkit.sln`
2. 配置 PCL/OpenCV 路径 (参考 `pcl_release.props`)
3. 选择 Release x64 配置
4. 生成解决方案

### 消费者 CMake 集成

```cmake
# 在消费者项目中
find_package(rxs-toolkit REQUIRED)
target_link_libraries(my-app PRIVATE rxs-toolkit)
# 消费者可以直接 #include <rxs/rxs_api.h>
```

## 许可证

Business Source License 1.1 (BSL 1.1)

- **Change Date**: 2030-01-01
- **Change License**: GPLv2
- **非生产使用**: 免费 (开发/测试/学术研究/内部评估)
- **生产使用**: 需购买商业许可 — rain3dmetrology@gmail.com

详见 [LICENSE](LICENSE)。

## 公司

苏州锐新视科技有限公司 (Suzhou Ruixin Vision Technology Co., Ltd.)

工业 AI 视觉 + 3D AOI 检测
