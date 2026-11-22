# CameraSkinPhysics

A Direct3D 12 soft-body prototype for comparing two GPU deformation models under the same sphere-to-sphere camera-space contact setup.

The executable starts with two 100,000-vertex spheres. Press **F1** or **F2** to select which independent simulation is displayed.

- **F1 — Tetrahedral XPBD:** an interior particle lattice is solved with one tetrahedron volume constraint and six local edge-length constraints per tetrahedron. A CSR gather pass accumulates constraint corrections without floating-point atomics. A barycentric mapping skins the render surface.
- **F2 — Camera surface:** the 100,000 render vertices are the simulation particles. Unique triangle edges provide XPBD edge-length constraints, each triangle adds a rest-area constraint, and each shared edge adds a hinge-distance bending constraint between its opposite vertices. A combined vertex CSR gather accumulates all corrections without floating-point atomics. This mode has no tetrahedral interior mesh.
- **Shared contact:** both modes render the opposing sphere into a depth/normal target and project their particles into that target for contact correction. Both use the same camera, initial sphere centers, initial velocities, capture resolution and iteration count.

Each mode owns independent particle state. Switching to a mode for the first time starts it from the same initial condition; switching back resumes that mode's previous state. Run one mode at a time for a clean benchmark.

## Controls

| Key | Action |
|---|---|
| F1 | Tetrahedral XPBD mode |
| F2 | Camera-surface mode |
| F3 | Show/hide the live parameter panel |

## Live parameter controls

アプリケーション起動時に「パラメータ」ウィンドウが開きます。**F3** で表示・非表示を切り替えられます。スライダーの変更は次のGPU処理から即時反映されます。「シミュレーションを一時停止／再開」と「パラメータを初期化」も利用できます。 While paused, the last deformed pose remains visible and the GPU simulation, contact and skinning dispatches are skipped.

- 反復回数: 1–16。GPUタイムスタンプ領域も最大回数に合わせて確保します。
- エッジ制約の緩和率: 0.0–1.5。
- 補正量の上限: 0.0001–0.05。
- 粒子の半径: 0.001–0.05。
- 接触時の押し出し量: 0.0–0.03。
- 接触時の摩擦と反発係数: 0.0–1.0。
- 接触判定の幅: 0.005–0.5。
- エッジ長・面積・曲げのコンプライアンス: それぞれ独立して調整可能。スライダーは対数目盛で、値が小さいほど硬く、大きいほど柔らかくなります。初期値はエッジ長 2e-6、面積 1e-7、曲げ 2e-4 です。
- クラスタ形状の復元強度: 0.0–1.0。重なりを持つ局所クラスタごとに最適回転・平行移動をGPUで推定し、元の局所形状へ近づけます。初期値は 0.25 です。
- ボーン追従の強さ: 0.0–1.0。アニメーションから計算した各頂点の絶対目標位置へ引き寄せます。初期値は 0.35 です。ボーン付きモデルでのみ有効です。

The collision controls currently affect Camera Surface mode. XPBD retains its existing collision constants. Use **Reset parameters** to restore defaults. The panel displays the latest CPU frame time, total GPU time, simulation, collision, render, combined surface-constraint-solve and gather timings. Every benchmark CSV row stores the parameter values active for that frame, allowing performance changes to be correlated with settings.

## Current implementation

- Win32 + D3D12 device, swap chain, graphics/compute root signatures, PSOs, GPU buffers and explicit resource-state transitions.
- Two 100,000-vertex sphere surfaces with 599,988 indices and closed upper/lower caps.
- Surface topology: 299,994 unique edges and 599,988 vertex-edge CSR incidence entries per sphere at the default resolution.
- Tetrahedral topology: 10,144 interior particles, 51,246 tetrahedra and 204,984 particle-element CSR incidence entries per sphere at the default resolution.
- Tetrahedral XPBD: particle integration, volume and edge constraints, ping-pong multipliers, CSR correction gather and barycentric GPU skinning.
- Camera-surface XPBD: 100,000 surface particles per sphere, edge-length, triangle-area and shared-edge hinge-distance bending constraints, ping-pong multipliers for all three constraint families, combined edge/area/bending correction storage, vertex-edge/triangle/hinge CSR gather, and direct GPU skinning.
- Cluster shape matching: connected geodesic core patches expanded by one adjacency ring, bidirectional cluster/vertex CSR membership, per-cluster GPU covariance fitting with iterative polar rotation extraction, and weighted overlapping target blending. The runtime uses 2048-vertex geodesic core patches expanded by two adjacency rings to preserve larger-scale curvature; the per-iteration correction strength defaults to 0.25 and is adjustable from the GUI. For imported rigged models, sampled bone poses also drive the cluster rest shape and the edge, area and hinge rest metrics each frame.
- Generic surface topology preprocessing: triangle rest-area constraints, shared-edge hinge rest-distance constraints, boundary/non-manifold/orientation diagnostics and signed rest volume are generated from arbitrary indexed triangle meshes. Edge records are sorted contiguously to avoid per-edge heap allocations at high mesh counts. Triangle-area constraints have vertex-to-triangle CSR adjacency (`triangle_incidence_ranges` / `incident_triangle_corners`). Bending hinges use the distance between the two opposite vertices as a stable rest metric and have vertex-to-hinge-corner CSR adjacency (`bending_incidence_ranges` / `incident_bending_corners`). The area XPBD pass computes one correction per triangle corner. The bending XPBD pass constrains the distance between the two opposite vertices of each shared-edge hinge and writes four corner slots (the shared-edge endpoint slots are zero). The combined vertex gather accumulates edge, area and bending corrections without floating-point atomics.
- Two 1024 × 1024 opposing-body normal/depth capture targets and the shared camera-space particle contact pass.
- GPU timestamp queries. CSV files are written separately:
  - `output/benchmark_xpbd.csv`
  - `output/benchmark_camera_surface.csv`

Each CSV contains per-frame CPU frame time/FPS, GPU simulation, capture, collision/skin, render intervals, total GPU time/FPS, particle count and tetrahedron count. It also records the active runtime settings on every row: solver iterations, edge relaxation, gather max correction, particle radius, pushout, velocity friction, restitution, contact band and cluster shape matching strength. The camera-surface run records zero tetrahedra because it has no tetrahedral volume mesh. The first timestamp interval includes solver passes and the initial skinning dispatch; the collision interval includes contact correction and the final skinning dispatch.

The memory columns report the allocated simulation-buffer sizes for the selected solver and for both solvers resident in the process. These are sums of the D3D12 buffer resource widths created for the simulation. Read-only topology buffers use upload heaps, so this is a resource-size accounting metric, not a measurement of dedicated VRAM residency. It excludes shared graphics resources such as swap-chain buffers, depth/normal capture textures, the render index buffer, descriptor heaps and driver/runtime allocations.

## Known limitations

- The camera method currently solves surface-edge, unsigned triangle-area and hinge-distance bending XPBD constraints plus overlapping cluster shape matching. The bending term resists local folding by maintaining the distance between opposite vertices across each shared edge; it is a lightweight hinge approximation, not a full dihedral-angle constraint. Area constraints resist local triangle collapse, but do not preserve triangle orientation by themselves. Cluster shape matching is an elastic local-rest-shape term, not a replacement for volume or self-collision constraints. Closed-volume GPU solving remains pending; signed rest volume is currently topology metadata only. Closed-volume solving must be gated by the generated manifold/orientation validation; open meshes remain supported for edge/area/bending constraints.
- The camera contact uses one frontmost depth layer from a single view. It cannot classify deep interior overlap or the far side of the opposing body. The shader limits correction to a narrow contact band to avoid projecting deep particles onto the visible shell. Multi-view capture, depth peeling or thickness reconstruction is required for robust deep overlap.
- The tetrahedral domain is a regular lattice clipped to the sphere. Its boundary is coarse and a small number of render samples use a nearest-element fallback. Rest offsets preserve the initial rest pose.
- Camera-surface render normals are recomputed from the deformed triangle adjacency on the GPU. Very low-quality or inconsistently wound input topology can still produce unstable normals.
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


## Blender model import

The parameter window provides **Blender モデルを読み込む** and **シミュレーションをやり直す** controls.
The importer runs `blender.exe` in background mode. It exports the mesh as OBJ and writes a companion
`.cspanim` file containing the Armature hierarchy, up to four normalized bone influences per vertex,
and sampled skinning matrices for every frame in the Blender scene range.

A single shared Armature is supported per imported model. The exporter temporarily disables Armature modifiers while evaluating the dependency graph, so
non-armature modifiers are included and armature deformation is applied exactly once at runtime. Vertices without valid bone groups remain rigid. The first four influences with the
largest weights are retained and normalized. Animation is sampled at the scene frame rate and loops
continuously at runtime.

The renderer updates the surface rest positions and edge-length, triangle-area and hinge-distance rest
metrics from the current bone pose before each F2 simulation step. The per-vertex bone-follow term drives particles toward the animated pose while cluster shape matching,
XPBD constraints and camera-space contact provide local shape preservation and secondary motion.
The pose sampling and rest-metric refresh currently run on the CPU; the XPBD solver remains on the GPU.

Imported models run with the camera-surface XPBD solver (F2); the tetrahedral solver (F1) remains
available for the built-in sphere only. The restart control relaunches the application with the currently
imported mesh and animation, restoring the initial simulation state and animation clock. It also restores
simulation parameters to their defaults.
