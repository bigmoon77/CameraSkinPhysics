# CameraSkinPhysics

A Direct3D 12 research demo comparing two GPU soft-body collision pipelines:

1. **Tetrahedral XPBD** — GPU compute passes integrate particles, solve tetrahedral volume/shape constraints, and resolve body contacts.
2. **Camera-space collision** — render the opposing body into depth/normal buffers, then project and correct soft-body particles against the captured surface. Optional camera-depth slicing is designed for layered contacts.

Both modes use the same generated sphere surface, initial transforms, timestep, and collision scenario. The default surface target is 100,000 vertices per body.

## Status

The repository is being built from an empty initial state. The first milestone is a compilable D3D12 application foundation with deterministic sphere generation, compute-based simulation kernels, selectable solver modes, and GPU timestamp logging. Numerical validation and performance tuning follow once the full pipeline is integrated.

## Requirements

- Windows 10/11
- Visual Studio 2022 (C++20)
- Windows SDK with Direct3D 12 headers
- DirectX Shader Compiler (DXC) available on PATH

## Build

Open `CameraSkinPhysics.sln` in Visual Studio, select x64, and build. The project uses the Windows SDK D3D12 runtime and invokes DXC at runtime to compile HLSL.

## Controls

- `1`: tetrahedral solver
- `2`: camera-space solver
- `Space`: pause/resume
- `R`: reset simulation
- `WASD` / mouse: camera
- `Esc`: quit

## Benchmark output

The application writes CSV rows to `logs/benchmark.csv`, including frame time and GPU timestamp durations for simulation, camera depth capture, collision correction, and rendering. GPU timings are only meaningful after the relevant passes have been integrated and enabled on the target GPU.

## Design notes

- The surface mesh and simulation particles are separate data sets. A 100k-vertex render surface does not imply 100k tetrahedral elements.
- The tetrahedral mode creates a volumetric tetrahedral lattice inside the sphere and maps surface vertices to simulation nodes.
- Camera-space collision uses a depth/normal capture of the other body. A conventional depth buffer stores only the nearest visible layer; depth slicing/peeling is required for multiple layers.
- The benchmark reports measured values; it does not fabricate GPU memory or pass timings.
