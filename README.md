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
| F3 | Show/hide the live parameter panel |

## Live parameter controls

The application opens a separate **Live Parameters** window. Use **F3** to show or hide it. Trackbars apply changes immediately to subsequent GPU dispatches. The panel also provides **Pause simulation / Resume simulation** and **Reset parameters** controls. While paused, the last deformed pose remains visible and the GPU simulation, contact and skinning dispatches are skipped.

- Solver iterations: 1–4. The upper bound currently matches the timestamp-query layout.
- Edge relaxation: 0.0–1.5.
- Gather max correction: 0.0001–0.05.
- Camera-surface particle radius: 0.001–0.05.
- Camera-surface pushout: 0.0–0.03.
- Camera-surface velocity friction and restitution: 0.0–1.0.
- Camera-surface contact band: 0.005–0.5.

The collision controls currently affect Camera Surface mode. XPBD retains its existing collision constants. Use **Reset parameters** to restore defaults. The panel displays the latest CPU frame time, total GPU time, simulation, collision, render, edge-solve and gather timings. Every benchmark CSV row stores the parameter values active for that frame, allowing performance changes to be correlated with settings.

## Current implementation

- Win32 + D3D12 device, swap chain, graphics/compute root signatures, PSOs, GPU buffers and explicit resource-state transitions.
- Two 100,000-vertex sphere surfaces with 599,988 indices and closed upper/lower caps.
- Surface topology: 299,994 unique edges and 599,988 vertex-edge CSR incidence entries per sphere at the default resolution.
- Tetrahedral topology: 10,144 interior particles, 51,246 tetrahedra and 204,984 particle-element CSR incidence entries per sphere at the default resolution.
- Tetrahedral XPBD: particle integration, volume and edge constraints, ping-pong multipliers, CSR correction gather and barycentric GPU skinning.
- Camera-surface XPBD: 100,000 surface particles per sphere, edge-length constraints, ping-pong multipliers, CSR correction gather and direct GPU skinning.
- Generic surface topology preprocessing: triangle rest-area constraints, shared-edge dihedral rest-angle constraints, boundary/non-manifold/orientation diagnostics and signed rest volume are generated from arbitrary indexed triangle meshes. Edge records are sorted contiguously to avoid per-edge heap allocations at high mesh counts. Triangle-area constraints now also have a vertex-to-triangle CSR adjacency (`triangle_incidence_ranges` / `incident_triangle_corners`) so a GPU correction gather can be implemented without floating-point atomics. Area/bending records are still preprocessing data; their GPU XPBD solve passes are the next implementation stage.
- Two 1024 × 1024 opposing-body normal/depth capture targets and the shared camera-space particle contact pass.
- GPU timestamp queries. CSV files are written separately:
  - `output/benchmark_xpbd.csv`
  - `output/benchmark_camera_surface.csv`

Each CSV contains per-frame CPU frame time/FPS, GPU simulation, capture, collision/skin, render intervals, total GPU time/FPS, particle count and tetrahedron count. It also records the active runtime settings on every row: solver iterations, edge relaxation, gather max correction, particle radius, pushout, velocity friction, restitution and contact band. The camera-surface run records zero tetrahedra because it has no tetrahedral volume mesh. The first timestamp interval includes solver passes and the initial skinning dispatch; the collision interval includes contact correction and the final skinning dispatch.

The memory columns report the allocated simulation-buffer sizes for the selected solver and for both solvers resident in the process. These are sums of the D3D12 buffer resource widths created for the simulation. Read-only topology buffers use upload heaps, so this is a resource-size accounting metric, not a measurement of dedicated VRAM residency. It excludes shared graphics resources such as swap-chain buffers, depth/normal capture textures, the render index buffer, descriptor heaps and driver/runtime allocations.

## Known limitations

- The camera method currently solves surface-edge XPBD only. Area and dihedral rest constraints are generated and validated on the CPU, but their GPU solve passes are not integrated yet; signed rest volume is currently topology metadata only. Until those passes are added, surface triangles can fold or invert under severe deformation. Closed-volume solving must be gated by the generated manifold/orientation validation; open meshes remain supported for edge/area/bending constraints.
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

The CTest suite includes two portable tests:
- `CameraSkinPhysics.MeshValidation`: sphere topology, outward triangle winding, unique surface edges, edge CSR, tetrahedron indices/volumes, tetrahedral CSR and rest-pose surface mappings.
- `CameraSkinPhysics.BenchmarkLogValidation`: CSV column count, CPU/GPU FPS calculations, fresh-file truncation and simulation-buffer byte fields.

## Build the Windows comparison app

Run from a Visual Studio 2022 Developer PowerShell:

```powershell
cmake -S . -B build -G "Visual Studio 17 2022" -A x64
cmake --build build --config Release
.\build\Release\CameraSkinPhysics.exe
```

The application begins in F1 mode. Press F2 to switch to the camera-surface simulation. Press Esc to close it.\n\nFor a repeatable comparison, run `CameraSkinPhysics.exe --benchmark=300`. This executes 300 fixed-60-Hz frames in F1 mode followed by 300 frames in F2 mode, then exits. The default is 300 frames per mode; values from 1 to 36,000 are accepted. Each launch creates fresh CSV files under `output/` relative to the current working directory.
