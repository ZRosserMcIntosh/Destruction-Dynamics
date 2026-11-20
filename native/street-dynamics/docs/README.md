# Street Dynamics

Destructible street furniture for the game: sign posts, street-name plates, bollards and bike
racks that a car can bend, fold, snap, uproot or shear off, with panels that tear free and fly.
Phase 1 of the destructible-world plan (signs and posts). Later phases add lighting columns,
signals, wood and concrete poles (Pole Dynamics), trees, clutter, heavy fixtures and road safety
hardware.

A native C++20 module with a C ABI (`include/sd/c_api.h`), built the same way as the other
Destruction Dynamics modules: deterministic (strict IEEE, no FMA contraction, our own `dmath`),
allocation-free in `step()`, and asleep (zero cost) until something touches it.

## What is modelled

Every object is **data** (`data/street_library.json`). An entry gives the post's section and
material, its height, how it meets the ground and what hangs on it. The section properties and
the hinge law are derived (`src/library.cpp`), so a new kind of object is a new JSON entry.

| Type | Post | Base | Panels | Source |
|---|---|---|---|---|
| `stop_sign_r1` | 2½″ × 2.65 mm galvanised tube, 2.75 m | embedded in a 0.3 × 0.3 × 0.6 m footing | octagon 0.604 m across flats, bottom ≥ 2.10 m | CONTRAN MBST Vol. I/III, CET-SP, NBR 11904 |
| `regulatory_r19` | as R-1 | as R-1 | circle 0.50 m | MBST Vol. I (≥ 0.40 m urban) |
| `warning_diamond` | as R-1 | as R-1 | diamond 0.50 m side | estimated |
| `street_name_double` | DN65 76.1 × 3.35 mm | embedded | two plates 80 × 40 cm, crossed | CET-SP; NBR 5580 class L |
| `sign_slip_base` | 76.1 × 3.35 mm | breakaway slip base (12 kN shear) | octagon | comparison case; MASH stub limit |
| `bollard_cast_iron` | Ø 90 mm grey cast iron, 0.69 m | embedded | — | Brazilian "frade" catalogue |
| `bollard_steel` | 114.3 × 6 mm, concrete filled, 0.8 m | footing | — | typical sidewalk bollard (not ASTM F2656 rated) |
| `bollard_concrete` | Ø 300 mm precast, 0.6 m | footing | — | SINAPI 103309 |
| `bike_rack_u` | inverted U, 50 × 2 mm tube, 0.8 m | two embedded feet | — | CET-SP paraciclo |

Each entry carries its own `source` line in the JSON, saying what is measured and what is estimated.

### The mechanics

* **Rigid bodies and XPBD** (Müller et al. 2020, *Detailed Rigid Body Simulation with Extended
  Position Based Dynamics*). Each post is a chain of capsule segments. Joints between them hold
  position rigidly and bending with a compliance of ℓ / EI, which is a beam between segment
  centres. The joint multipliers are vectors, because a joint is solved twice per substep and its
  error can change direction between passes. 16 substeps per 60 Hz step.
* **Section mechanics.** Round tube I = π/64 (D⁴ − d⁴) and Z = (D³ − d³)/6, plus square,
  rectangular and solid sections.
  * EN 1993-1-1 class limits 50/70/90 ε² set each hinge's moment-rotation law: rise to a peak
    (hardening for stocky walls, below M_p for slender ones), local buckling, softening to a
    residual, and tearing at the ductility limit.
  * Brittle materials (cast iron, concrete) use the modulus of rupture. They snap and keep a small
    residual.
  * Concrete-filled tubes get a factor.
* **Plastic hinges.** When the moment a joint carries (λ / h²) exceeds its capacity, its rest
  rotation moves toward the current one by the excess. That move is the permanent bend; past the
  ductility limit the hinge tears.
* **Bases.**
  * *Embedded*: the post hinges at the ground line.
  * *Slip base*: lets go at a shear.
  * *Socket*: tips out past an angle.
  * *Bolted*: fails at a bolt moment.
  * *Footing*: the soil rotates in series with the post, yields, and lets go past its angle.
    Saturated ground (`wetness`) weakens it by up to 45 %.
  * Each object varies its strengths by ±10 %, seeded from its position, so a row of identical
    signs doesn't fail identically.
* **Panels** are clamped plates that let go at a force or a twist. Once loose they fly as flat
  plates: C_N = 1.2 sin α, with the centre of pressure moving with incidence.
* **Contacts.**
  * Ground with friction. A penetration that already existed (a base end released below grade)
    is eased out at 1 m/s, not converted into a launch velocity.
  * Host boxes: kinematic oriented boxes for hosts without a crash model, with approach-aware
    normals, so a point inside a moving box leaves through the face it entered by.
* **Sleep.** An object sleeps when its parts have drifted less than 1.5 cm in 0.5 s. A bent post
  lying on the pavement trembles but goes nowhere.
* **Closed frames.** A rack with both feet in the ground is a loop that one sequential pass per
  substep never quite closes. These get 4 finer substeps of their own (they are light).
* **Joint damping** at 30 /s on relative spin stands in for the steel's own damping.

### Coupling to the car (Vehicle-Dynamics PR #30)

The vehicle is the authority. The recipe is in `tests/vehicle_adapter.cpp`:

1. The car's `ObstacleQuery` asks Street Dynamics for point and edge contacts
   (`obstaclePoint` / `obstacleEdge`, or `sd_obstacle_point` / `sd_obstacle_edge`). The query
   returns depth, normal and a **yield force**: the most the object can push back this step. That
   is max(1.2 × its last contact force, its static resistance (capacity / lever arm) + effective
   mass × closing speed / step).
2. The car solves its crash structure against those contacts. Its `ObstacleContact::yieldForce`
   caps the shared multiplier, so a post that gives way doesn't stop the car like a wall.
3. The car reports `ObstacleImpact`s (impulse, point, normal). The host passes each to
   `applyImpact` / `sd_apply_impact`, so momentum balances exactly between the two engines.
4. Loose debris that hits the car comes back through `bodyImpulse` / `sd_body_impulse`.

The normal is approach-aware: a point already past a post's axis along the relative motion is
still pushed back the way it came, so a fast car cannot tunnel through a thin post between steps.

## Results (`ctest`, 26 test cases)

### Car crashes

The crash tests run the real vehicle model (OpenRoad Dynamics hatch, scaled to the test mass)
into the objects. Δv is measured against a coasting twin car, because the test car decelerates
about 0.25 g on its own.

| Hit | Exit speed | Δv (km/h) | ASI | THIV (km/h) | OIV (m/s) | Object |
|---|---|---|---|---|---|---|
| Stop sign, 900 kg, 100 km/h | 95.5 km/h | 4.5 | 0.24 | 7.2 | 2.0 | folds flat, panel off |
| Stop sign, 900 kg, 35 km/h | 32.1 km/h | 2.9 | 0.13 | 5.5 | 1.5 | torn |
| Stop sign, 1100 kg, 50 km/h | 46.8 km/h | 3.2 | 0.20 | 5.6 | 1.5 | down |
| The same post, rigid | 11.2 km/h | 38.8 | 1.11 | — | — | peak 16.9 g, crush 0.84 m |
| Slip base, 1100 kg, 100 km/h, ¼-point | 98.4 km/h | 1.6 | 0.13 | 5.9 | 1.5 | released, stub 0.05 m |
| Cast-iron frade, 1100 kg, 50 km/h | 48.5 km/h | 1.5 | 0.11 | 4.3 | 1.2 | snaps |
| Steel bollard, 1100 kg, 20 km/h | 6.7 km/h | 13.3 | 0.47 | 16.3 | 4.5 | holds, bent 9°, crush 0.18 m |
| Bike rack, 1250 kg, 30 km/h | 28.1 km/h | 1.9 | 0.09 | 4.2 | 1.2 | torn |
| Street-name post, 1250 kg, 60 km/h | 53.8 km/h | 6.2 | 0.39 | 7.4 | 2.1 | down, both plates fly |

* EN 12767 category for the stop sign at 100 km/h: **NE** (non-energy-absorbing). That is what a
  light sign post should be: ASI ≤ 1.0, THIV ≤ 27 km/h, exit speed well above the HE/LE bands.
* The slip base meets MASH: OIV ≤ 5 m/s (and the preferred 3 m/s), ridedown ≤ 20.49 g, stub
  ≤ 102 mm.
* Calibration checks:
  * A quasi-static push peaks between M_el / arm and M_peak / arm.
  * At 50 km/h the post folds.
  * Cast iron snaps where steel holds.
  * The same post held rigid stops the car at 16.9 g.

### World tests

* **Allocation.** No heap allocation in `step()` across 10 s of a car ploughing through a street
  (counted with a replaced `operator new`).
* **Cost.** A street of 160 objects with a car driving through it: 0.68 ms mean and 1.9 ms worst
  per 60 Hz step, with at most 10 awake. 10 s after the car has gone, nothing is awake.
* **Persistence.** Damage survives a save to a fresh world, a tile unload/reload and a repair.
  The format is "SD01", bit-exact tilt.
* **Wet ground.** A stout bollard on a shallow footing, given the same 300 N s blow, leans 4.2°
  in dry soil and 6.9° saturated.
* **Fuzz.** 60 random hits (speed, angle, height, offset, every type) never fault, never leave a
  part below the ground, never produce a non-finite number.
* **Determinism.** The same inputs give the same state hash.
* **C API.** Library, objects, impacts, events, parts, persistence and repair through the ABI.

## Tunables

| Name | Default | Meaning |
|---|---|---|
| `substepsPerStep` | 16 | XPBD substeps per host step |
| `hostFriction` | 0.45 | parts against host boxes (paint on steel) |
| `groundFriction` | 0.55 | parts on asphalt or paving |
| `airDensity` | 1.225 | loose panels' aerodynamics |
| `wetness` | 0 | 0 dry … 1 saturated; footings up to 45 % weaker |
| `jointDamping` | 30 /s | relative spin across a joint decays at this rate |
| `sleepSpeed`, `sleepTime` | 0.03 m/s, 0.5 s | sleep when drift < speed × time over the window |

## Simplifications and gaps

* Posts are chains of rigid segments with lumped hinges: 4–8 per post. That resolves where a
  post folds and how far, not the shape of the crease.
* Parts of different objects don't collide with each other. Debris hits the ground and the cars.
* Panels are rigid: they tear off but don't crumple.
* Footing stiffness and yield are **estimates**, marked as such in the JSON. No pull-over test
  data was found for São Paulo sign footings.
* Brazilian primary sources were partly unreachable: gov.br blocks automated fetches. CONTRAN /
  MBST and CET-SP values come from secondary sources quoting them, and each entry says so.
* Steel yield for galvanised conduit (NBR 5580) is unspecified; 250 MPa is assumed.

## Build

```sh
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release \
      -DSD_VEHICLE_DYNAMICS=/path/to/Vehicle-Dynamics   # optional: the crash tests
cmake --build build -j
./build/tests/sd_tests
```

`SD_WARNINGS_AS_ERRORS=ON` turns warnings into errors. The crash tests need Vehicle-Dynamics
with yielding obstacles (PR #30).
