# CameraSkinPhysics

A Direct3D 12 soft-body prototype for comparing two GPU deformation models under the same sphere-to-sphere camera-space contact setup.

The executable starts with two 100,000-vertex spheres. Press **F1** or **F2** to select which independent simulation is displayed.

- **F1 — Tetrahedral XPBD:** an interior particle lattice is solved with one tetrahedron volume constraint and six local edge-length constraints per tetrahedron. A CSR gather pass accumulates constraint corrections without floating-point atomics. A barycentric mapping skins the render surface.
- **F2 — Camera surface:** the 100,000 render vertices are the simulation particles. Unique triangle edges form a surface spring network, solved with XPBD edge-length constraints and a vertex-to-edge CSR gather. This mode has no tetrahedral interior mesh.
- **Shared contact:** both modes render the opposing sphere into a depth/normal target and project their particles into that target for contact correction. Both use the same camera, initial sphere centers, initial velocities, capture resolution and iteration count.

Each mode owns independent particle state. Switching to a mode for the first time starts it from the same initial condition; switching back resumes that mode's previous state. Run one mode at a time for a clean benchmark.

## Controls

| Key | Action |
|---|---|
| F1 | Tetrahedral XPBD mode |
| F2 | Camera-surface mode |

## Current implementation

- Win32 + D3D12 device, swap chain, graphics/compute root signatures, PSOs, GPU buffers and explicit resource-state transitions.
- Two 100,000-vertex sphere surfaces with 599,988 indices and closed upper/lower caps.
- Surface topology: 299,994 unique edges and 599,988 vertex-edge CSR incidence entries per sphere at the default resolution.
- Tetrahedral topology: 10,144 interior particles, 51,246 tetrahedra and 204,984 particle-element CSR incidence entries per sphere at the default resolution.
- Tetrahedral XPBD: particle integration, volume and edge constraints, ping-pong multipliers, CSR correction gather and barycentric GPU skinning.
- Camera-surface XPBD: 100,000 surface particles per sphere, edge-length constraints, ping-pong multipliers, CSR correction gather and direct GPU skinning.
- Two 1024 × 1024 opposing-body normal/depth capture targets and the shared camera-space particle contact pass.
- GPU timestamp queries. CSV files are written separately:
  - `output/benchmark_xpbd.csv`
  - `output/benchmark_camera_surface.csv`

Each CSV contains per-frame CPU frame time, GPU simulation, capture, collision/skin, render intervals, particle count and tetrahedron count. The camera-surface run records zero tetrahedra because it has no tetrahedral volume mesh. The first timestamp interval includes solver passes and the initial skinning dispatch; the collision interval includes contact correction and the final skinning dispatch.

## Known limitations

- The camera method is currently a surface-edge XPBD model paired with depth/normal contact. It does not have a volume or thickness constraint. Surface triangles can fold or invert under severe deformation; bending/dihedral and area constraints are possible next steps.
- The camera contact uses one frontmost depth layer from a single view. It cannot classify deep interior overlap or the far side of the opposing body. The shader limits correction to a narrow contact band to avoid projecting deep particles onto the visible shell. Multi-view capture, depth peeling or thickness reconstruction is required for robust deep overlap.
- The tetrahedral domain is a regular lattice clipped to the sphere. Its boundary is coarse and a small number of render samples use a nearest-element fallback. Rest offsets preserve the initial rest pose.
- Render normals are currently radial approximations. Recompute from deformed triangle adjacency for improved lighting.
- The renderer waits for the GPU once per frame to keep resource and query reuse simple. This is a correctness-oriented baseline, not a throughput-optimized renderer.
- DXC compilation and D3D12 runtime validation are required before trusting GPU results. The development environment used to author this branch did not provide a Windows D3D12 runtime or DXC, so no claim of successful hardware execution is made.

## Requirements

Portable geometry tests:
- CMake 3.24+
- C++20 compiler

D3D12 comparison app:
- Windows 10/11
- Visual Studio 2022 with C++20 support
- Windows SDK with Direct3D 12
- DirectX Shader Compiler (`dxc`) on PATH
- D3D12-capable GPU

## Build and test the portable geometry code

Run from the repository root:

```sh
cmake -S . -B build -DCSP_BUILD_D3D12_APP=OFF
cmake --build build --config Release
ctest --test-dir build --output-on-failure
```

The validation checks sphere topology, outward triangle winding, unique surface edges, edge CSR, tetrahedron indices/volumes, tetrahedral CSR and rest-pose surface mappings.

## Build the Windows comparison app

Run from a Visual Studio 2022 Developer PowerShell:

```powershell
cmake -S . -B build -G "Visual Studio 17 2022" -A x64
cmake --build build --config Release
.\build\Release\CameraSkinPhysics.exe
```

The application begins in F1 mode. Press F2 to switch to the camera-surface simulation. CSV output is written relative to the current working directory.
