# CameraSkinPhysics

A Direct3D 12 research project comparing GPU soft-body collision pipelines:

1. **Tetrahedral XPBD** — GPU compute passes integrate particles and solve tetrahedral volume constraints.
2. **Camera-space collision** — capture the opposing body's depth and world-space normal, then project and correct particles against that surface.

The target render mesh contains exactly 100,000 vertices per sphere. Render vertices and simulation particles are separate: the volumetric solver uses a lower-resolution interior lattice with a surface-to-simulation mapping.

## Current implementation status

- Win32 + D3D12 device, swap chain, depth buffer, graphics pipeline, and two-body sphere rendering.
- Deterministic 100,000-vertex sphere surface generator.
- CPU tetrahedral lattice generation, rest-volume construction, particle-to-tetrahedron incidence CSR, and surface-to-particle mapping.
- Initial HLSL kernels for integration, tetrahedral volume constraints, gather-based correction, and camera-space collision.
- CMake shader compilation through DXC.
- CSV benchmark writer interface.

**The current executable renders two static, slightly overlapping 100k-vertex spheres. The compute kernels are not yet dispatched, so deformation and GPU timing are not implemented yet.** The tetrahedral lattice is generated and its counts are shown in the window title.

## Requirements

- Windows 10/11
- Visual Studio 2022 with C++20 support
- Windows SDK with Direct3D 12
- DirectX Shader Compiler (dxc) on PATH
- CMake 3.24+

## Build

~~~powershell
cmake -S . -B build -G "Visual Studio 17 2022" -A x64
cmake --build build --config Release
~~~

Run build/Release/CameraSkinPhysics.exe.

## Solver design

Both modes will use the same two sphere transforms, initial velocities, timestep, and 100k-vertex render surface.

The tetrahedral lattice currently uses a regular grid clipped to the sphere and six tetrahedra per complete grid cell. This is a prototype volumetric discretization; boundary quality and rest-shape constraints need refinement before performance comparisons are meaningful. The current XPBD kernel solves volume preservation only. Edge/shape constraints are planned.

Camera collision captures depth plus world-space normals. A single depth target stores only the nearest layer; layered depth slicing/peeling will be an optional mode with separate per-slice capture resources.
