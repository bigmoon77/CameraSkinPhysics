# CameraSkinPhysics

A Direct3D 12 research project for comparing two GPU soft-body collision pipelines:

1. **Tetrahedral XPBD** — GPU compute passes integrate particles and solve tetrahedral volume constraints.
2. **Camera-space collision** — capture the opposing body's depth and world-space normal, then project and correct particles against that surface.

The target render mesh contains exactly 100,000 vertices per sphere. Render vertices and simulation particles are separate: the volumetric tetrahedral solver will use a lower-resolution interior lattice with a surface-to-simulation mapping.

## Current implementation status

The repository currently contains:
- A Win32 + D3D12 device/swap-chain bootstrap.
- A deterministic 100,000-vertex sphere surface generator.
- Initial HLSL kernels for particle integration, tetrahedral volume constraints, gather-based correction, and camera-space collision.
- CMake shader compilation through DXC.
- A CSV benchmark writer interface.

**The current executable is a D3D12 initialization/clear-screen milestone. The simulation kernels are not yet wired into command lists, and the sphere mesh is generated for validation but is not yet rendered.** GPU timings will be added only after the corresponding passes are dispatched and timestamp queries are read back.

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

## Planned comparison contract

Both modes will use the same two sphere transforms, initial velocities, timestep, and 100k-vertex render surface. The benchmark will report measured CPU frame time and GPU pass durations separately.

Camera collision captures depth plus world-space normals. A single depth target stores only the nearest layer; layered depth slicing/peeling will be an optional mode with separate per-slice capture resources.

The tetrahedral mode will use a volumetric tetrahedralization, per-particle inverse masses, surface mapping, and Jacobi-style gather to avoid floating-point atomics during constraint correction.
