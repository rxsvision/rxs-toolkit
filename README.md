# RXS Toolkit

> 核心公共库 + DLL 导出层 — 苏州锐新视科技有限公司 (RXS Vision)

## 概述

RXS Toolkit 是锐新视 3D 测量算法库的核心公共组件，提供 18 个 DLL 导出函数，涵盖 NURBS 曲面拟合、点云配准、几何特征计算、光度立体重建等工业 AI 视觉算法。

## DLL 导出函数清单 (18个)

| # | 导出名 | 功能 | 源文件 |
|---|--------|------|--------|
| 1 | `fitBSpline_` | B 样条曲线拟合 | dllmain.cpp |
| 2 | `semiconductor` | 半导体特征测量 | dllmain.cpp |
| 3 | `PIN` | PIN 引脚测量 | dllmain.cpp |
| 4 | `registration` | 点云配准 | dllmain.cpp |
| 5 | `rtConvert` | RT 坐标转换 | dllmain.cpp |
| 6 | `keyboard` | 键盘/按键特征提取 | dllmain.cpp |
| 7 | `surfaceProfile` | 曲面轮廓提取 | dllmain.cpp |
| 8 | `computeConvexHull` | 凸包计算 (PCL 薄包装) | dllmain.cpp |
| 9 | `volume` | 体积计算 | dllmain.cpp |
| 10 | `dis2dis` | 距离矩阵计算 | dllmain.cpp |
| 11 | `computeDeformationNake` | 形变计算 | dllmain.cpp |
| 12 | `colorReg` | 颜色配准 | dllmain.cpp |
| 13 | `Flatness` | 平面度计算 | dllmain.cpp |
| 14 | `LaptopCalculation` | 笔记本计算 | dllmain.cpp |
| 15 | `photometricStero` | 光度立体 (CUDA) | dllmain.cpp |
| 16 | `photometricStero_iv` | 光度立体 (多视角) | dllmain.cpp |
| 17 | `photometricStero_ivs` | 光度立体 (多视角+阴影) | dllmain.cpp |
| 18 | `photometricStero_ivss` | 光度立体 (多视角+双阴影) | dllmain.cpp |

> **holeDeepth 重建**: `holeDeepth` 原为幽灵引用 — `GetProcAddress(dllHander, "holeDeepth")` 返回 NULL，DLL 从未导出此函数。Phase 1 已清除消费者侧 24 处无效引用，同时在 `rxs-algorithms/modules/hole_depth/` 重建为独立算法模块，基于 `Hole.cpp/h/hpp` 已有算法（KNN 边界提取、半径离群点去除、RANSAC 椭圆拟合）。支持三种使用方式：独立编译、集成到 DLL、直接链接。详见 [rxs-algorithms/modules/hole_depth/README.md](https://github.com/rxsvision/rxs-algorithms/tree/main/modules/hole_depth)。

## 目录结构

```
rxs-toolkit/
├── .github/workflows/
│   └── encoding-check.yml    # CI: 检测 GBK 编码回归
├── czxDependence/
│   ├── czxTool.h             # 公共工具库 (Timer/Log/Ransac/readProfile)
│   ├── czxTool.cpp           # v8 最新版 (13,885 bytes)
│   ├── cuda.props            # CUDA 构建属性
│   └── pcl_release.props     # PCL Release 构建属性
├── dllmain.cpp               # 生产 DLL 导出 (18 函数)
├── dllmain-d.cpp             # 调试 DLL 导出 (10 函数, #define CZX_DEBUG)
├── Source.def                # DLL 导出定义文件 (18 EXPORTS)
├── BSplineLength.cpp/h       # B 样条曲线长度计算
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

### VS2022 构建

1. 打开 `czxToolkit.sln`
2. 配置 PCL/OpenCV 路径 (参考 `pcl_release.props`)
3. 选择 Release x64 配置
4. 生成解决方案

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
