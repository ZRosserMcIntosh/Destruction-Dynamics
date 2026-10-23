# Research

## 1. Survey of the existing repositories (2026-10-23)

These are the repositories this session could reach, inspected before any
design decision:

| Repository | Contents | Relevance |
|---|---|---|
| `ZRosserMcIntosh/Aviation-Dynamics` | C++20 flight-simulation core with a WASM build and a C ABI. 240 test cases. Its world uses 500 m tiles centred on multiples of 500 m in an ENU frame (`avd::world::tileIndex`, `world_ground`). It drives zrossermcintosh.com/sao-paulo/flying. | The only substantial codebase. It sets the conventions this repository follows: ENU world frame, 500 m tiles, ADRs in `docs/decisions`, honest validation tables. Its WASM loader is a reference for a future Rust/WASM crate. |
| `ZRosserMcIntosh/liquid` | Only a `LICENSE` file. | **No fluid engine exists yet.** `FluidAdapter` is designed so that a future `liquid` engine can plug in (see INTEGRATION.md). |
| `ZRosserMcIntosh/Glass` | Initial commit, README only. | Nothing to integrate. |
| `ZRosserMcIntosh/ballistic-dynamics` | Initial commit, README only. | Nothing to integrate. |
| `ZRosserMcIntosh/Destruction-Dynamics` | Initial commit (MIT license and README). | This repository. |

The Brazil open-world game itself (renderer, asset streaming, world
streaming) is not in any accessible repository. Nothing here assumes its
internals. The integration points are interfaces (`TileActivationHost`,
`WorldStateStore`, `FluidAdapter`, `PhysicsAdapter`, `AudioEngine`) plus the
documented ENU and tile conventions taken from Aviation Dynamics.

## 2. Techniques considered

### Destruction

| Approach | Verdict |
|---|---|
| Fully simulated rigid-body collapse of a whole tower (thousands of pieces, unconstrained) | Rejected for landmarks. It is unpredictable, expensive, hard to art-direct and hard to synchronise. |
| Pre-baked vertex or alembic animation | Rejected as the only method. It is not interactive, cannot be scrubbed with physics, and is a large download. |
| **Hierarchical authored motion with bounded runtime physics** (the approach of large set-piece collapses in AAA games, from public talks) | **Chosen.** Macro motion is a function of time. Meso pieces are budgeted rigid bodies. Micro detail is particles. |
| Voronoi pre-fracture at build time | Planned for meso pieces (walls, slabs) through `FractureAssetRef`. The first cycle uses facade panels and box chunks. |
| Voxel destruction in the style of Teardown | Out of scope for a browser open world at city scale. It is noted as a possible future option for small interiors. |

The tip motion uses an inverted-pendulum profile (a uniform rod pivoting at
its base) so that the acceleration feels physical. It is then
time-normalised to the authored stage duration. That gives credible motion
and exact timing together. It is not a structural analysis.

### Water

| Approach | Verdict |
|---|---|
| FFT ocean (Tessendorf 2001) | Planned for the far field on WebGPU, where compute makes it cheap. Gerstner waves stand in for now and use the same shader interface (ADR 0007). |
| Gerstner waves (GPU Gems 1, ch. 1) | **Chosen for the far field now.** They are cheap, analytic and identical on CPU and GPU. |
| Height-field wave equation | Considered. It has no velocity field to drive foam advection or drag on floating objects. |
| **Linearised shallow water on a C-grid** | **Chosen for the near field.** It provides velocities, depth-dependent speed (waves slow and steepen toward the beach) and land walls, at O(N²) cost. |
| SPH or FLIP particles | Rejected at city scale for real-time use. A possible Cinematic-preset option for local splashes, behind `FluidAdapter`. |
| Authored surge packets | **Chosen as an art-direction layer** on top of the solver for landmark impacts, so the money shot is guaranteed. |

### Physics engines

| Engine | Notes |
|---|---|
| **Rapier (rapier3d-compat)** | Rust compiled to WASM. Maintained, with deterministic builds available, CCD, sleeping and collision groups. The compat build embeds its WASM, so it loads anywhere. **Chosen** (ADR 0003). |
| Jolt (JoltPhysics.js) | Strong candidate. It is used in AAA games and its WASM is available. Worth benchmarking against Rapier when debris counts grow. The adapter keeps the switch cheap. |
| Ammo.js (Bullet) | Ageing and heavy. Rejected. |
| cannon-es | Pure JS and slower at high body counts. Rejected for debris. |

### Rendering

Three.js WebGLRenderer is the default today. Three's WebGPURenderer
requires node materials (TSL), and the hand-written GLSL here would need
porting. The plan is in ADR 0002.

## 3. References

- J. Tessendorf, *Simulating Ocean Water*, SIGGRAPH course notes, 2001.
- M. Finch, *Effective Water Simulation from Physical Models*, GPU Gems, ch. 1, 2004.
- R. Bridson, *Fluid Simulation for Computer Graphics*, 2nd ed., 2015, on staggered grids and shallow water.
- A. Arakawa & V. Lamb, *Computational design of the basic dynamical processes of the UCLA general circulation model*, 1977, on the C-grid.
- Rapier documentation, https://rapier.rs/docs/.
- Three.js documentation, https://threejs.org/docs/.
- Web Audio API, W3C Recommendation (PannerNode, HRTF).
