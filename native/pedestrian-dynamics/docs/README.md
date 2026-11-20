# Pedestrian Dynamics

Physically simulated people for the game. Each person is an active ragdoll with these properties:

* **Pose:** stands and holds a pose with muscles.
* **Blows:** loses strength to blows.
* **Vehicles:** wraps onto a bonnet, is projected ahead of an SUV, or is thrown over a wing, the
  way the accident literature describes.
* **Momentum:** gives every impulse back to the car that struck them, so momentum balances.
* **Injuries:** reports what a crash-test lab would measure: HIC, tibia acceleration, knee
  bending, ligament tears, throw distance and the kinematics class.

It's a native C++20 module with a C ABI (`include/ped/c_api.h`), built like the other Destruction
Dynamics modules:

* deterministic (strict IEEE, no FMA contraction, our own `dmath`);
* allocation-free in `step()`;
* asleep, costing nothing, until something touches it.

It's the first part of the active-ragdoll plan from the repository review:

* **Jolt Physics** (MIT): the joint-limit and pose-motor ideas.
* **ProtoMotions / MuJoCo** (Apache-2.0): the body layout and gains, cross-checked.
* **de Leva's anthropometry** (CC-BY): the masses.

No code or data with a non-commercial licence is used.

## The body

`data/pedestrian_body.json` holds 16 capsule segments: pelvis, abdomen, thorax, head, and for each
side thigh, shank, foot, upper arm, forearm and hand. They are defined as fractions of stature and
built for any height and mass.

| | Source |
|---|---|
| Masses | de Leva (1996), male: head 6.94 %, trunk 43.46 %, thigh 14.16 %, shank 4.33 %, foot 1.37 %, upper arm 2.71 %, forearm 1.62 %, hand 0.61 % |
| Lengths | Drillis & Contini (1966) as tabulated by Winter: shoulder 0.818 H, hip 0.530 H, knee 0.285 H, ankle 0.039 H, and so on |
| Joint ranges | AAOS / Greene & Heckman typical active ranges. Examples: hip −20…120° flexion, −25…45° abduction; knee −3…140°; ankle −50…20°; shoulder −50…170°; neck −55…45° |
| Gains | Estimated, the same order as the DeepMimic / AMP humanoid gains and Coros et al. (2010). They scale with mass × stature |

The test reproduces the standing centre of mass at 54.4 % of stature (≈ 55 % expected).

### Joints

* **Ball joints** are rigid in position.
* **Range model.** Each range is four quarter-ellipses of swing about the neutral pose plus a
  twist range, so every direction has its own limit: hip flexion 120° and extension 20° are both
  exact. A pose is clamped to the closest point on the ellipse, as Jolt does. Pulling it straight
  back toward neutral instead would drag a narrow joint, like the elbow (145° × ±3°), halfway back
  from a little sideways error.
* **End of range** is stiff but compliant, standing in for ligaments.
* **Ligament tears.** Past the tear torque the ligaments tear: the range opens by 20° and the
  joint loosens. The `LigamentTear` event names the joint.
* **Muscles** are XPBD angular drives toward a target pose.
  * Compliance is 1/(kp × strength), and torque is capped at the joint's maximum × strength.
  * Damping is critical.
  * Targets are clamped to the range (muscles don't tear their own ligaments).
* **Strength.** It runs from 1 (holding the pose) to 0 (limp, with a little muscle tone that
  stays).
  * A vehicle striking faster than 1.5 m/s takes it away.
  * A blow takes up to half of it (a 9 mm round, about 3 N s, takes 7 %).
* **Balance assist** (0…1): a capped torque holding the pelvis and chest upright, for standing in
  place until struck. A stepping balance controller comes next (see below).

### Contacts

* **Ground** is solved last in each substep, so friction undoes the whole substep's slide and the
  feet stay put.
  * Friction is μ 0.66, the value Searle & Searle (1983) give for a body sliding and tumbling on
    asphalt.
* **Every contact is the body's own tissue in series with what it hits.**
  * Scalp 1 MN/m, limbs and pelvis 300 kN/m, torso 150 kN/m (estimates).
  * Damping is Hunt–Crossley style (Hunt & Crossley 1975): it grows with penetration, so a contact
    starts at zero force. A plain damper jolts a head with 400 g in the first millisecond of
    touching the asphalt.
* **Self-contact** is tissue on tissue, filtered like Jolt's ragdolls: parent–child pairs are
  skipped, and so are pairs that already overlap when standing.

## Vehicles

`data/vehicle_profiles.json` describes each vehicle exterior as stiffness-tagged regions: bumper,
grille (bonnet leading edge), bonnet, cowl, windscreen, A-pillars, roof header, roof and body.

| Profile | Bumper | Bonnet leading edge | Windscreen rake | Mass |
|---|---|---|---|---|
| sedan | 0.20–0.55 m | 0.73 m | 27° | 1300 kg |
| hatch | 0.18–0.52 m | 0.70 m | 26° | 1100 kg |
| suv | 0.35–0.75 m | 1.02 m | 34° | 1900 kg |
| van (forward control) | 0.30–0.60 m, flat front to 1.05 m | — | 76° | 2600 kg |

Shapes follow the usual grouping of front shapes in pedestrian-protection work (EEVC WG17, Euro
NCAP). Region stiffness and damping are estimates.

### Coupling recipe for hosts

1. `ped_set_vehicles` with each car's pose and velocity at the start of the step.
2. `ped_step`.
3. `ped_vehicle_impulse` per car: add the linear impulse and its moment (about the car's origin)
   to the car's rigid body.

The car is moved by its own model. A person is light next to a car, so the one-step lag is small,
and momentum balances exactly: the test shows an imbalance of 1e-10 N s.

Box contacts use approach-aware normals: a body part inside a moving box leaves through the face
it entered by. They only trust a face crossed in the last 50 ms. Otherwise a shin riding along with
the bumper could be shoved out of the bumper's far side; that bug was found here and also fixed in
Street Dynamics.

## Validation (`ped_tests`, 18 cases)

Throws come from the simple test car (mass, yaw inertia, 0.7 g braking from impact). Each impulse
the person puts on it is handed back.

| Impact | Throw (Searle band¹) | Kinematics | Hardest head strike | HIC15, car phase | Upper tibia² | Car Δv |
|---|---|---|---|---|---|---|
| sedan 20 km/h | 3.6 m (2.4–3.4) | wrap | bonnet 4.2 m/s at 251 ms | 148 | 80 g | 0.9 km/h |
| sedan 30 km/h | 7.8 m (5.4–7.7) | wrap | bonnet 6.4 m/s at 176 ms | 330 | 138 g | 1.3 km/h |
| sedan 40 km/h | 11.5 m (9.5–13.7) | wrap | bonnet 9.3 m/s at 127 ms | 765 | 185 g | 1.7 km/h |
| sedan 50 km/h | 17.2 m (14.9–21.4) | wrap | bonnet 11.5 m/s at 106 ms | 1283 | 239 g | 2.2 km/h |
| sedan 60 km/h | 23.6 m (21.5–30.8) | wrap | bonnet 14.6 m/s at 88 ms | 2679 | 384 g | 2.7 km/h |
| SUV 40 km/h | 13.2 m (9.5–13.7) | forward projection | bonnet 6.4 m/s at 104 ms | 306 | 228 g | 1.5 km/h |
| van 40 km/h | 17.3 m (9.5–13.7) | forward projection | windscreen 8.7 m/s at 31 ms | 726 | 217 g | 1.5 km/h |
| hatch 40 km/h | 10.9 m (9.5–13.7) | wrap | windscreen, glancing | 63 | 178 g | 2.0 km/h |

¹ Searle & Searle (1983): s = v²/(2μg) for a horizontal launch to v²(1+μ²)/(2μg) for the best
angle, at launch speed = car speed and μ = 0.66. Real data scatter about ±30–40 % around it.
² First 80 ms, as a legform test measures.

* **Head strike at 40 km/h.** A sedan's strike lands ~125 ms after the bumper touches at roughly
  the car's speed. Published PMHS and THUMS work gives 111–150 ms and 10.7–15.3 m/s for adults.
* **Kinematics follow Ravani et al. (1981).**
  * An adult wraps on a sedan, whose bonnet edge is below the adult's centre of mass.
  * An SUV or a forward-control van projects them forward.
  * A 1.2 m child in front of a sedan is projected forward.
  * A strike at the car's corner gives a fender vault.
* **Car Δv** is close to m/(M+m) of its speed: the person leaves at about the car's speed.
* **Blows.** A light blow (15 N s) staggers. A hard one (120 N s) takes the legs away.
* **Head on the ground.** HIC there varies a lot with how the head lands; it is reported separately
  (`hic_ground`). An unprotected head reaching asphalt at 5–8 m/s gives 300–700 g pulses of
  2–4 ms, as helmet drop tests show. That is real, not noise.
* **Determinism.** The same crash twice gives the same bits.
* **Allocation.** None in `step()` with a car ploughing through six people.
* **Cost.** One struck person costs ~0.35 ms mean (under 0.5 ms worst) per 60 Hz step, 16 substeps;
  40 people standing asleep cost nothing.
* **Fuzz.** 24 random cars, speeds (5–90 km/h), angles, statures and strengths never fault and
  never leave a part below the ground.

## Gaps and next steps

* **Knee bending reads high.** It's 40–55° in the bumper phase; EEVC legform tests of real cars
  typically see 10–25°, against a 15° limit. Once the ligaments tear the knee's sideways range
  opens. The tear torques and end-of-range stiffness need calibrating against legform data.
* **The van throws too far** (17 m at 40 km/h, above Searle's band). The flat front rebounds the
  body more than real fronts do; restitution needs calibrating.
* **Unchecked sources.** Searle's bands and the HIC figures are cross-checked from secondary
  sources: SAE, IRCOBI and Euro NCAP primary documents were not reachable from this environment.
  The Drillis–Contini fractions are quoted from Winter's textbook figure.
* **Next:**
  * a stepping balance controller (capture-point foot placement, after Coros et al. 2010) and
    protective behaviours (bracing with the arms, catching a fall);
  * get-ups from a settled pose;
  * Unity and browser bindings and a lab page;
  * windscreen cracks through the Glass module, from the `HeadImpact` events;
  * wheel run-over;
  * person-to-person contact.

## Build

```sh
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
./build/tests/ped_tests
```

`PED_WARNINGS_AS_ERRORS=ON` turns warnings into errors.
