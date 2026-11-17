# Pole Dynamics

Concrete utility poles and overhead wires for the São Paulo driving game:
a car hits a pole, the pole cracks, leans or snaps, the wires react, lines
come down into the street, live conductors arc, and the lights go out.

A native C++20 module with the same shape as OpenRoad Dynamics (`ord_*`)
and Ballistic Dynamics (`bd_*`): a static core, a shared library with a C ABI
(`pd_*`), a thin Unity wrapper over P/Invoke, and a WebAssembly build for the
browser lab.

**This is a game simulation.** The numbers are chosen to look and behave
plausibly. They are not engineering values and must not be used to design,
assess or work on real poles or power lines.

## Layout

| Path | What |
|---|---|
| `include/pd/params.hpp` | Every tunable, with the real-world value it is based on |
| `include/pd/world.hpp`, `src/world.cpp` | Tiles, spans, power sections, lanes, queries, persistence, `step()` |
| `src/bodies.cpp` | Impact pulse, soil, cracking/snapping, rigid pieces, transformer |
| `src/wires.cpp` | XPBD wires: waking, solving, clamps, snapping, sleep, hazards |
| `src/poles.cpp` | The world builder's seed formula, pole attributes, mass properties |
| `include/pd/c_api.h`, `src/c_api.cpp` | The C ABI |
| `include/pd/unity_frame.hpp` | Sim ↔ Unity frame conversion |
| `unity/PoleDynamics.cs` | Unity wrapper (P/Invoke, frame conversion, events) |
| `unity/PoleDynamicsBridge.cs` | Example MonoBehaviour: stepping, instanced poles, wire switching, events |
| `tools/scenarios.*`, `tools/pdtrace.cpp` | Shared test scenarios; command-line trace tool |
| `tools/plot_traces.py` | Trace JSON → SVG (standard library only) |
| `tests/` | Headless tests |
| `web/` | WebAssembly build script and JS loader |
| `docs/traces/` | Trace CSV/JSON and SVG plots from the test run |

## Building

```sh
cmake -S . -B build            # Release by default
cmake --build build -j
./build/tests/pd_tests         # all tests; pass a substring to filter: pd_tests street:
./build/pdtrace 80 2 8         # km/h, pole index, seconds [wetness transformer-index out-prefix yaw]
python3 tools/plot_traces.py build/traces/*.json -o docs/traces
web/build-wasm.sh              # WebAssembly (wasi-sdk), for the browser lab
```

As in the other native modules, the build uses `-ffp-contract=off` and no
fast-math. The maths that has to be bit-identical across platforms
(`sin`, `cos`, `atan2`, `sqrt` …) goes through `pd::dm`, vendored from
Ballistic Dynamics.

## Conventions

* **Units:** SI throughout (m, kg, s, N, rad). Degrees only at the API, in
  parameters ending in `_deg`.
* **Frame:** ENU, right-handed, Z up, metres from MASP (the world data's
  origin). Unity is Y up and left-handed. The wrapper swaps Y and Z for
  points and vectors, and maps a rotation `(w,x,y,z)` to `(w,−x,−z,−y)`
  (`pd/unity_frame.hpp`, tested).
* **Pole body frame:** x along the line (the pole's heading, which is also
  the double-T flange and strong axis), y to the left, z up. The street is
  on the right (−y).
* **Time:** the game calls `pd_step(1/60)`. The world runs 4 internal steps
  (240 Hz). An accumulator handles other `dt`s.
* **Determinism:** no wall-clock time, no unordered iteration in the
  simulation, fixed iteration counts, `dm` maths, and a seeded LCG for arcs.
  `pd_state_hash` is an FNV-style hash over the bits of every pole, body and
  node. Two runs give the same hash (tested).
* **Memory:** pools are sized when tiles load: rope slots (96 by default),
  bodies (24), node arrays, events (2,048-entry ring) and scratch for power
  and lane checks. `step()`, `reportImpact()` and `pollEvents()` never
  allocate (tested by counting `operator new`).

## World data

The `poles_<tile>.json` format is unchanged:
`{tile, v, poles:[[x,y,hdg_deg,cls]], spans:[[ax,ay,bx,by]]}`.

The seed formula is reproduced exactly as the world builder computes it,
including Python's round-half-to-even and integer `//`:

```
s = ((int(round(x*10)) * 73856093) ^ (int(round(y*10)) * 19349663)) & 0x7fffffff
transformer    = s % 100 < 14
light          = (s // 100) % 100 < {55, 70, 85, 95}[cls]
telecom_cables = 4 + cls*2 + (s // 10000) % 5
service_drops  = (s // 100000) % 4
```

The pole ID is the seed. Spans link poles by position, with a 0.1 m cell
search. A span that crosses a tile border links when the second tile loads,
and is not duplicated. Every check in this section is tested against
reference values from CPython.

Lanes come from the site's lanes tiles (`T_<i>_<j>.json`, 0.5 m pixels).
`LANE_BLOCKED` carries the `(tile, edge, lane)` triple packed into 64 bits:
16 bits tile x, 16 bits tile y, 24 bits edge, 8 bits lane. `pd_unpack_lane`
unpacks it.

## The model

### Pole and impact

* **Pole:** 11 m above ground plus 1.5 m buried. Double-T section,
  0.26 × 0.30 m at the ground line, tapering to 45% at the top, 78% solid,
  2,500 kg/m³. Mass properties come from 48 slices plus the hardware
  (crossarm 45 kg, LV rack 25 kg, LED luminaire 18 kg, transformer 350 kg
  at 8.9 m).
* **Impact:** the vehicle module reports a contact point, a push direction,
  the relative velocity and the car's mass. The pulse is integrated at
  50 µs: the car's mass on a crush spring (400 kN/m, the narrow-object
  stiffness of a modern car front), unloading at 16× the stiffness so most
  of the crush is permanent, against the pole's effective mass at bumper
  height. The output is the peak force, impulse, the car's Δv and the pulse
  duration, which goes back to the vehicle module.
* **Section checks:**
  * Peak shear at bumper height against `impactCrack` and
    `impactUltimate` (per class).
  * Ground-line moment against the cracking and ultimate moments. Those
    moments are the NBR 8451 nominal load × the lever arm, × 2 for
    ultimate, × 0.5 on the weak axis, with an elliptical interaction
    between the two axes.
  * Each pole's strength varies by ±12%, drawn from its seed.
* **States:**

  | State | When |
  |---|---|
  | `INTACT` | No damage |
  | `CRACKED` | The section cracked. Leaves a 0.6° permanent tilt plus any small soil set |
  | `LEANING` | Soil plastic tilt above 5° |
  | `SNAPPED` | Sheared at bumper height. The top becomes a free rigid body, kicked at its base. A bending break at the ground line instead keeps a rebar hinge (10% residual moment, tears at 60 kN) |
  | `FALLEN` | The piece lies on the ground above 60° and is asleep |

* **Soil:** an elastic–plastic rotational spring and damper at the pivot,
  1.0 m below ground:
  * Stiffness 30 MN·m/rad.
  * Yield moment 50 kN·m dry, ×0.55 when saturated; stiffness ×0.5 when
    saturated.
  * Hardening 0.1%.
  * The foundation lets go at 20°, leaving 15% of the yield moment.

  `pd_set_weather(wetness 0…1)` blends these.
* **Rigid bodies:**
  * Pieces use semi-implicit integration with sequential-impulse contacts:
    8 iterations, against the ground plane and the host's capsules (cars).
  * When a falling piece hits a car, `POLE_HIT_BODY` reports the car's ID
    and the impulse. This includes the car that hit the pole.
  * Damping: 0.4/s in the air, 3/s on the ground.
  * Sleep is judged over 1 s windows (less than 2 cm and 0.3° of net
    movement), so a piece jittering in its wires still sleeps.
* **Transformer:** detaches when the tank's acceleration during the pulse
  exceeds 40 m/s² (about 4 g, about 14 kN on its brackets) or when the
  pole tilts past 35°. It then falls as its own body.
  `TRANSFORMER_LANDED` reports the landing speed, with hook bits for an oil
  leak (above 3 m/s) and fire (above 8 m/s).

### Wires

Each span between two poles carries:

* 3 MV conductors at 10.96 m, offset −1.65/−0.55/+0.55 m on the crossarm;
* 4 LV conductors at 7.40–8.06 m;
* the telecom bundle as one rope at 5.92 m, its mass growing with the
  cable count;
* service drops (from the seed) to houses 12 m away.

* **Asleep:** the span is an analytic parabolic catenary,
  sag = wL²/8H, with the rest tension below. It costs nothing, and Unity
  draws it with the existing catenary renderer.
* **Awake:**
  * XPBD rope with one node per 3.5 m, between 10 and 24 nodes.
  * Each wire is solved with a direct tridiagonal (Thomas) solve per
    substep, which converges where Gauss–Seidel would need many
    iterations (Deul et al. 2018), with compliance from EA.
  * Rest lengths are pre-stretched to the analytic tension, so waking a
    span does not jolt it.
  * Velocity damping 0.6/s, ground friction 0.6, capsule push-out against
    cars.
* **Waking:** an impact wakes spans up to 2 poles away. After that, a pole
  wakes only when its wire load *changes* from the load it settled under by
  more than 60% of its cracking capacity. A pole that permanently carries a
  hanging neighbour stays asleep. Spans go back to sleep after 1 s of
  stillness. They return to the analytic catenary if nothing changed (no
  broken wire, no displaced pole, tension within 20% of rest), and
  otherwise freeze in place.
* **Clamps:**
  * Each fitting holds a load relative to its rest load (`clampBreak` per
    kind), varied ±45% by the pole's seed.
  * Ties are directional: a conductor pulled off the top of its insulator
    holds 30% of the strength.
  * A pole going over carries its conductors with it (×3).
  * Dead ends use strain clamps at 90% of the conductor's strength.
  * Guys are tension-only, capped at twice the rest load.
* **Snapping:** a segment above the conductor's rated tensile strength
  breaks, and 25% of its stored elastic energy comes back as whip-back
  speed.
* **Hazards:**
  * A live MV conductor on the ground raises `WIRE_ON_GROUND_LIVE` (a 4 m
    hazard radius), then arcs at random every 0.6 s on average (`ARC`).
  * `sim.mv_trip_time` > 0 lets protection de-energise it after that many
    seconds. By default it stays live.
  * LV conductors and drops are live only while their pole is powered.
* **Lanes:** each host step, anything below 2.2 m over a lane (a wire node
  or a fallen piece) raises `LANE_BLOCKED` once for that lane. A repair
  raises `LANE_CLEARED`.

### Power

Each pole belongs to the section of its nearest transformer, found by a
breadth-first search along the spans. A pole is powered when its section's
transformer is attached with its MV intact and an unbroken LV path reaches
the pole within the section. Changes raise `POWER_OUT(section)`,
`LIGHT_CHANGED` per streetlight and `POWER_RESTORED`.

### Persistence

* `pd_save_tile_state` writes the diff of a tile ("PD01"): damaged poles
  (state, tilts, pieces, broken fittings, transformer) and displaced spans.
  Spans are keyed by their poles' IDs, so a diff survives a different tile
  load order.
* `pd_unload_tile` keeps the diff for the session, and loading the tile
  again restores it.
* `pd_repair_pole` is the crew hook: it puts the pole back up, re-strings
  its spans, and raises `POLE_REPAIRED`, `LANE_CLEARED` and power events.

## Results

From `pd_tests`. The street has six poles 35 m apart, residential class,
with the lanes tile's kerb 0.5 m from the pole line. A 1,300 kg car hits
pole 2 at 0.5 m. Plots and traces are in `docs/traces/`.

| Test | Outcome |
|---|---|
| 30 km/h | `CRACKED`, 3.1°; the other poles intact. Peak 185 kN |
| 50 km/h | `LEANING`, 7.0° in the soil; neighbours 0.1° |
| 80 km/h | `SNAPPED` (sheared at the bumper). The top hangs in its wires at about 25°; neighbours pulled 0.1–0.2° |
| 80 km/h, saturated ground | `SNAPPED`; neighbours pulled, none fall |
| 100 km/h | The top comes down (63–89°). 12–14 wires snap. Neighbours left 4° out of plumb (one leans 15° in some seeds). Live MV on the ground arcs, lanes blocked, power out |
| 60 km/h, transformer pole | `TRANSFORMER_DETACHED`, lands. `POWER_OUT` for its section only; the section of the transformer two poles away stays lit |
| 30 km/h, transformer pole | Transformer stays; no power loss |
| Determinism | Identical 64-bit state hash over two runs. A 1 km/h change gives a different hash |
| Idle | A loaded, undisturbed world runs zero internal steps |
| Settling | After an 80 km/h snap, everything is asleep again within 20 s |
| Allocation | 0 heap allocations in 10 s of a double hit (step, impact, events) |
| Persistence | Save → fresh world → load gives the same state and tilt. Unload/reload keeps the damage. Repair restores the pole |

### Domino: what happens and what does not

The brief asked for "snap + domino" at 80 km/h. The model gives a snap, and
the line drags the neighbours, but it does **not** give a chain of falling
poles.

* At 80 km/h the snapped top hangs in its conductors at about 25°, which is
  what real snapped concrete poles usually do.
* At 90 km/h and above it falls, snaps 12–14 wires, and pulls the next poles
  1–4° out of plumb, sometimes 15° (`LEANING`).
* The neighbours do not go over because their conductors fail first. Ties
  and clamps let go, or the wire snaps, before the pull reaches a
  neighbour's soil or section capacity. That is how distribution lines are
  meant to fail.

To get dramatic dominoes for gameplay, the lab and `pd_set_param` expose
the levers: lower `soil.yield` and `strength.nominal.*`, raise
`wire.*.clamp` so the conductors hold and transmit the pull, and raise the
wetness. Treat that as an art-direction choice, not a fix.

### Performance

Measured on the container's CPU: an Intel Xeon @ 2.10 GHz, a vCPU,
Release build, one thread. The scenario is two parallel 12-pole streets
with three poles hit at 80 km/h.

| Window | Mean | p95 | Max |
|---|---|---|---|
| ≥ 3 bodies moving, ≤ 45 woken spans (the brief's case) | 0.18–0.24 ms | 0.31–0.45 ms | 0.33–0.52 ms |
| Whole 6 s run, including the line unzipping to ~80 woken spans | 0.32–0.46 ms | 0.41–0.59 ms | up to 2.5 ms (one step) |

* The mean meets the 0.3 ms budget. The 95th percentile does not.
* The worst single step (up to 2.5 ms) is the first step after three
  simultaneous impacts, when about 60 spans wake at once.
* An idle world costs zero internal steps.
* The test asserts only a loose 1.5 ms mean ceiling, so it does not flake
  on CI machines.

## Tunables and where the numbers come from

All tunables are in `include/pd/params.hpp`. The common ones can be set by
name through `pd_set_param` (the table in `src/c_api.cpp`) and from the
lab.

| Tunable | Value | Basis |
|---|---|---|
| Nominal load, residential/tertiary/secondary/primary | 600 / 600 / 1,000 / 1,500 daN | ABNT NBR 8451 (concrete poles for distribution networks). An 11 m double-T pole is rated by its nominal load 0.10 m below the top on the strong face. Brazilian utility standards specify 11 m/600 daN for urban streets with a 13.8 kV crossarm, and 1,000/1,500 daN for heavier structures |
| Ultimate / nominal | 2.0 | NBR 8451: breaking load ≥ 2 × nominal |
| Weak face / strong face | 0.5 | NBR 8451 rates the weak face (A) at half the strong face (B) for double-T poles |
| Impact crack / ultimate shear (residential) | 150 / 420 kN | Game-tuned. Sits between the peak forces of 30 and 80 km/h pulses so the brief's outcomes land |
| Impact moment factor | 5 | Game-tuned. Concrete and steel are stronger under fast loading: DIF about 1.2–1.5 (*fib* Model Code 2010, §5.1.11). Most of the factor is the pole's own inertia: the top does not follow the base within a 10–20 ms pulse |
| Rebar hinge | 10% residual, tears at 60 kN | Game-tuned. A snapped prestressed or reinforced pole hangs on its steel |
| Soil stiffness / yield | 30 MN·m/rad, 50 kN·m | Game-tuned. The order of magnitude follows Broms-type lateral capacity for a 1.5 m embedment in compacted fill. Yield sits below the pole's weak-axis strength, so poles lean before they break |
| Wet soil | ×0.5 stiffness, ×0.55 yield | Saturation roughly halves the effective stress near the surface. Game-tuned |
| MV conductor | 0.216 kg/m, 19.5 kN RTS, Ø 10.1 mm | 1/0 AWG ACSR "Raven" (6/1), as in manufacturer datasheets (for example Southwire's ACSR catalogue) |
| LV conductor | 0.147 kg/m, 8.9 kN RTS, Ø 9.35 mm | 1/0 AWG AAC "Poppy" |
| Rest tensions | MV 2.4 kN, LV 1.0 kN, telecom 1.2 kN, drop 0.3 kN | MV at about 12% RTS (everyday stress, a typical 10–20%). This gives about 0.3 m of sag on 35 m, which matches the game's renderer |
| Telecom bundle | 0.40 kg/m + 0.15 kg/m per cable | A lashed bundle on a 6.3 mm messenger. Game-tuned |
| Clamp/tie holding | MV 8 kN, LV 4 kN, telecom 5 kN, drop 0.8 kN, ±45% | Game-tuned. Distribution ties are designed to slip or let go before the conductor or the pole fails |
| Transformer | 350 kg at 8.9 m | Pole-top 45–75 kVA tanks |
| Detach acceleration | 40 m/s² | Game-tuned (about 4 g on the brackets) |
| Car crush stiffness | 400 kN/m | Narrow-object (pole) frontal stiffness. Car frontal stiffness against rigid barriers is typically 0.5–1.5 MN/m, and lower against a narrow pole. Game-tuned |
| XPBD | compliance 1/EA per metre, direct solve | Macklin, Müller, Chentanez, "XPBD: Position-Based Simulation of Compliant Constrained Dynamics", MIG 2016. Deul, Kugelstadt, Weiler, Bender, "Direct Position-Based Solver for Stiff Rods", CGF 2018 |
| Lane width / clearance | 3.2 m / 2.2 m | Typical urban lane width. Clearance is game-tuned: what a car cannot drive under |

## Simplifications

* **Flat ground at z = 0.** Heights are above the local ground. Pole bases
  sit at the tile's terrain height in Unity; the wrapper does not add it.
* **Parabolic catenary for sleeping spans.** The error against the true
  catenary is under 1 mm at these sags.
* **Pole as a rigid body.** Bending is not resolved: the impact checks use
  a peak-force and dynamic-moment model instead. A crack leaves a fixed
  permanent tilt rather than modelling degraded stiffness.
* **One shear break per impact, at bumper height.** Bending breaks higher
  up are not modelled.
* **Soil is one rotational spring.** There is no sliding, uplift or
  per-direction stiffness. Wetness is one global value.
* **Wires are point masses on distance constraints.** Wires have no bending
  stiffness and no torsion. Insulators are point attachments, and wires do
  not collide with each other.
* **Telecom is one rope for the whole bundle**, and service drops end at a
  fixed point 12 m from the pole.
* **The domino is weak** (see above). Neighbours lean but do not fall with
  the default parameters.
* **Power:** sections by nearest transformer; no feeder topology. An MV
  conductor on the ground stays live unless `mv_trip_time` is set. There is
  no recloser logic.
* **Lanes:** the near test is against each edge's polyline and lane count.
  Lanes are assumed to be equal width, from the left edge of the road.
* **Bodies:** cars are capsules supplied by the host each step. The pole
  sends no impulses back into the cars; the host applies `POLE_HIT_BODY`.
* **Performance:** the budget holds on the mean. The worst steps (mass
  wake-up, 80+ awake spans) exceed it.
