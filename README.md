# CameraSkinPhysics

A Direct3D 12 research prototype for real-time soft-body interaction between two 100,000-vertex spheres.

The current executable combines two techniques in one scene:

- **GPU tetrahedral XPBD** — particles are integrated on the GPU; each tetrahedron solves one volume and six edge-length constraints, and a CSR gather pass accumulates the per-tetrahedron corrections without floating-point atomics.
- **Camera-space contact correction** — each body is rendered into an opposing-body depth/normal capture. Particle positions are projected into that capture and corrected against a bounded contact band.

The current build is a hybrid prototype. The camera pass supplements the tetrahedral solver; it is not yet an independent camera-only A/B demo. The project still needs that separate comparison path before performance claims can be made between the two methods.

## Current implementation

- Win32 + D3D12 device, swap chain, graphics pipeline, compute root signature, PSOs, descriptor heap and GPU resource transitions.
- Two sphere surface meshes, each with exactly 100,000 vertices and 597,600 indices.
- CPU tetrahedral lattice generation: 10,144 simulation particles, 51,246 tetrahedra and 204,984 particle-element incidence entries per body at the default resolution.
- Per-body ping-pong particle buffers, integration, four XPBD iterations, volume constraints and six local edge constraints per tetrahedron.
- CSR gather pass for deterministic accumulation of tetrahedral corrections.
- GPU barycentric skinning of 100,000 render vertices per body.
- Two 1024 × 1024 opposing-body normal/depth capture targets and a particle camera-contact correction pass.
- GPU timestamp queries for simulation, capture, camera collision/skin update and rendering.
- CSV output at `output/benchmark.csv`. Times are GPU timestamp intervals; CPU frame time is also recorded.

## Known limitations

- This is currently a hybrid tetrahedral + camera-contact solver. A separate camera-only soft-body implementation and a proper side-by-side A/B scene remain outstanding.
- The camera pass uses one frontmost depth layer per body. It cannot infer the full inside/outside state or distinguish the far side of a captured surface. The collision shader therefore limits correction to a thin contact band; multi-view capture, depth peeling or thickness reconstruction is needed for robust deep overlap.
- The tetrahedral domain is a regular lattice clipped to the sphere. Its boundary is coarse and some render samples use a nearest-element fallback. The surface rest offsets preserve the original rest pose.
- Skin normals are currently radial approximations, not normals recomputed from deformed triangle adjacency.
- The renderer currently waits for the GPU each frame to keep descriptor and query reuse simple. This is a correctness-oriented baseline and reduces throughput; frame-latency overlap should be added before final performance comparison.
- GPU timestamp values and physical deformation have not been validated on a Windows D3D12 runtime in this development environment. Treat the new GPU path as unverified until it is built and exercised on a Windows machine with DXC and a D3D12-capable GPU.

## Requirements

Portable geometry tests:
- CMake 3.24+
- C++20 compiler

D3D12 demo:
- Windows 10/11
- Visual Studio 2022 with C++20 support
- Windows SDK with Direct3D 12
- DirectX Shader Compiler (`dxc`) on PATH

## Build and test (all platforms)

```sh
cmake -S . -B build -DCSP_BUILD_D3D12_APP=OFF
cmake --build build --config Release
ctest --test-dir build --output-on-failure
```

## Build the Windows demo

```powershell
cmake -S . -B build -G "Visual Studio 17 2022" -A x64
cmake --build build --config Release
```

Run `build/Release/CameraSkinPhysics.exe`. The benchmark CSV is written relative to the working directory.

## Solver data layout

The surface mesh and volumetric simulation mesh are separate. Every render vertex has a four-particle barycentric mapping and a rest offset. The volume solver uses a regular 3D lattice clipped to the sphere and splits complete lattice cells into six tetrahedra. Each tetrahedron computes a volume constraint plus six edge-distance constraints. The gather pass follows per-particle CSR incidence lists and sums contributions from adjacent tetrahedra.

The camera contact pass captures the opposing body into a normal target and a depth target. For each particle it reconstructs the captured surface position using the inverse view-projection matrix, evaluates the signed distance along the captured normal and applies a bounded positional/velocity correction.
