# CameraSkinPhysics

A Direct3D 12 research project comparing GPU soft-body collision pipelines:

1. **Tetrahedral XPBD** — GPU compute passes integrate particles and solve tetrahedral volume constraints.
2. **Camera-space collision** — capture the opposing body's depth and world-space normal, then project and correct particles against that surface.

The target render mesh contains exactly 100,000 vertices per sphere. Render vertices and simulation particles are separate: the volumetric solver uses an interior lattice and a surface-to-simulation mapping.

## Current implementation status

- Win32 + D3D12 device, swap chain, depth buffer, graphics pipeline, and two-body sphere rendering.
- Deterministic 100,000-vertex sphere surface generator.
- CPU tetrahedral lattice generation, rest-volume construction, particle-to-tetrahedron incidence CSR, and surface mapping.
- Initial HLSL kernels for particle integration, tetrahedral volume constraints, gather-based correction, and camera-space collision.
- Portable CTest validation for sphere topology, normals, tetrahedron indices/volumes, CSR incidence, and surface mapping.

**The D3D12 executable currently renders two static spheres. GPU compute dispatch, deformed-surface rendering, camera-depth capture, and GPU timestamp queries are still under development.** Do not interpret the current app as a completed soft-body demo.

## Requirements

Portable geometry tests:
- CMake 3.24+
- C++20 compiler

D3D12 demo:
- Windows 10/11
- Visual Studio 2022 with C++20 support
- Windows SDK with Direct3D 12
- DirectX Shader Compiler (dxc) on PATH

## Build and test (all platforms)

~~~sh
cmake -S . -B build -DCSP_BUILD_D3D12_APP=OFF
cmake --build build --config Release
ctest --test-dir build --output-on-failure
~~~

## Build the Windows demo

~~~powershell
cmake -S . -B build -G "Visual Studio 17 2022" -A x64
cmake --build build --config Release
~~~

Run build/Release/CameraSkinPhysics.exe.

## Solver design

The current tetrahedral lattice is a regular grid clipped to the sphere, split into six tetrahedra per complete grid cell. This is a prototype volumetric discretization; boundary quality and rest-shape constraints need refinement before performance comparisons are meaningful. The current XPBD kernel solves volume preservation only. Edge/shape constraints are needed to preserve shear and local shape.

A camera collision pass captures depth plus world-space normals. A single depth target stores only the nearest layer; layered depth slicing/peeling will be optional, with separate per-slice capture resources.
