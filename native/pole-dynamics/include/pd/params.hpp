// Pole Dynamics — utility poles and overhead wires.
// SPDX-License-Identifier: MIT
//
// Every tunable, in one place, with the real-world value it is based on.
// docs/README.md lists them with sources. Units: SI (N, m, kg, s, rad).
//
// The pole strengths start from ABNT NBR 8451 (concrete poles for
// distribution networks): an 11 m double-T pole is sold by its nominal load,
// applied 0.10 m below the top on the strong face (face B); the weak face (A)
// takes half; the breaking load is at least twice the nominal. The impact
// (shear) capacities and the soil are game-tuned so that a 1,300 kg car at
// bumper height gives the outcomes in docs/README.md; they are plausible, not
// measured.
#pragma once

#include <array>

#include "pd/math.hpp"

namespace pd {

inline constexpr int kPoleClasses = 4;  // 0 residential, 1 tertiary, 2 secondary, 3 primary/trunk

// ---- Pole geometry (heights above ground; see docs/README.md, "Heights") ----
struct PoleGeometry {
    Real lengthAbove = 11.0;   // m above ground (the game's 11 m pole)
    Real buried = 1.5;         // m below ground
    Real baseWidth = 0.26;     // m, flange width at the ground line
    Real baseDepth = 0.30;     // m, section depth at the ground line
    Real topScale = 0.45;      // top dimensions as a fraction of the base
    Real fill = 0.78;          // concrete fraction of the bounding rectangle (double-T)
    Real density = 2500.0;     // kg/m^3, reinforced concrete
    // Attachments.
    Real mvCrossarm = 10.55;
    Real mvHeight = 10.96;
    std::array<Real, 3> mvOffset{-1.65, -0.55, 0.55};  // m, lateral, + toward the street
    std::array<Real, 4> lvHeight{7.40, 7.62, 7.84, 8.06};
    Real lvOffset = 0.20;
    Real telecomHeight = 5.92;   // centre of the 5.55-6.30 m bundle
    Real telecomOffset = -0.18;
    Real transformerHeight = 8.9;
    Real transformerOffset = 0.55;
    Real transformerMass = 350.0;  // kg (a 45-75 kVA pole-top tank)
    Real lightLow = 8.6, lightHigh = 9.4;
    // Hardware masses (kg).
    Real crossarmMass = 45.0;      // crossarm, 3 pin insulators, braces
    Real lvRackMass = 25.0;        // 4-position secondary rack, insulators
    Real lightMass = 18.0;         // LED luminaire and arm
    // Pivot of the embedded pole in the soil: about 2/3 of the embedment.
    Real pivotDepth = 1.0;
};

// ---- Pole strength per class ----
struct PoleClassStrength {
    Real nominalLoad;     // N at 0.10 m below the top, strong face (NBR 8451 rating)
    Real impactCrack;     // N of impact force at bumper height that cracks/spalls the section
    Real impactUltimate;  // N that shears it through
};

struct PoleStrength {
    // NBR 8451 ratings for poles carrying a 13.8 kV crossarm: 11 m / 600 daN on
    // residential and tertiary streets, 1000 and 1500 daN on the bigger roads.
    std::array<PoleClassStrength, kPoleClasses> cls{{
        {6000.0, 150e3, 420e3},
        {6000.0, 170e3, 480e3},
        {10000.0, 220e3, 600e3},
        {15000.0, 280e3, 760e3},
    }};
    Real ultimateFactor = 2.0;    // breaking moment / nominal moment (NBR 8451: >= 2)
    Real crackingFactor = 1.0;    // cracking moment / nominal moment (cracks open near service load)
    Real weakAxisFactor = 0.5;    // weak face / strong face
    Real residualFactor = 0.10;   // rebar hinge moment after snapping, / ultimate
    Real rebarTear = 60e3;        // N the rebar hinge carries before it tears free
    Real variability = 0.12;      // +/- fraction, per pole from its seed
    Real crackTilt = 0.6 * kDeg;  // permanent tilt left by cracking
    // Energy loss of a broken piece: air drag, the wires' hysteresis and, on
    // the ground, rolling/scraping resistance (1/s).
    Real pieceDamping = 0.4;
    Real groundDamping = 3.0;
    // Under a 10-20 ms impact the ground-line section holds several times its
    // static moment: strain-rate strength gain of concrete and steel (DIF
    // ~1.2-1.5, fib Model Code 2010) and, mostly, the flexible pole, whose top
    // does not follow the base within the pulse. Game-tuned.
    Real impactMomentFactor = 5.0;
};

// ---- Soil (rotational spring + damper at the pivot) ----
struct SoilParams {
    Real stiffness = 30e6;      // N*m/rad, elastic rotational stiffness (dry)
    Real yieldMoment = 50e3;    // N*m before the soil yields (dry fill; below the section's weak-axis strength, so poles lean first)
    Real hardening = 0.001;     // post-yield stiffness / elastic stiffness
    Real damping = 0.25e6;      // N*m*s/rad
    Real failAngle = 20 * kDeg; // tilt past which the foundation lets go
    Real failResidual = 0.15;   // fraction of the yield moment left after it lets go
    Real wetStiffness = 0.5;    // multipliers at wetness 1 (saturated ground)
    Real wetYield = 0.55;
    Real leanAngle = 5 * kDeg;  // plastic tilt that counts as LEANING (less: a small permanent tilt)
};

// ---- Conductors ----
enum class WireKind : int { MV = 0, LV = 1, Telecom = 2, Drop = 3 };
inline constexpr int kWireKinds = 4;

struct WireType {
    Real massPerMetre;  // kg/m
    Real ea;            // N, axial stiffness E*A
    Real breaking;      // N, rated tensile strength
    Real tension;       // N, horizontal tension at rest (sets the sag)
    Real radius;        // m, for collision and rendering
    Real clampBreak;    // N, insulator tie / clamp holding force
    Real voltage;       // V phase-to-phase (0: not energised)
};

struct WireParams {
    // MV: 1/0 AWG ACSR "Raven" (6/1): 0.216 kg/m, 19.5 kN RTS, 10.1 mm; 13.8 kV feeder.
    // LV: 1/0 AWG AAC "Poppy": 0.147 kg/m, 8.9 kN RTS, 9.35 mm; 220/127 V.
    // Telecom: the lashed bundle on a 6.3 mm messenger; mass grows with the cable count.
    // Drop: 6 AWG aluminium duplex service drop.
    std::array<WireType, kWireKinds> type{{
        {0.216, 4.7e6, 19.5e3, 2400.0, 0.0051, 8000.0, 13800.0},
        {0.147, 3.2e6, 8.9e3, 1000.0, 0.0047, 4000.0, 220.0},
        {0.40, 2.0e6, 6.0e3, 1200.0, 0.030, 5000.0, 0.0},
        {0.10, 1.0e6, 2.6e3, 300.0, 0.006, 800.0, 220.0},
    }};
    Real telecomPerCable = 0.15;  // kg/m added to the bundle per cable
    Real nodeSpacing = 3.5;       // m between rope nodes (a 35 m span: 10 nodes)
    int minNodes = 10;
    int maxNodes = 24;
    Real damping = 0.6;           // 1/s velocity damping (air, internal)
    Real groundFriction = 0.6;
    Real recoil = 0.25;           // fraction of stored elastic energy released as whip speed at a snap
    Real clampVariability = 0.45; // +/- fraction per fitting, from the pole's seed
    Real deadEndFactor = 0.9;     // a conductor ending on a pole is in a strain clamp: this fraction of its strength
    Real liftFactor = 0.3;        // a tie pulled off the top of its insulator (along the pole) holds this fraction
    Real carriedFactor = 3.0;     // a pole going over keeps its conductors (the crossarm comes down with them)
    Real dropLength = 12.0;       // m from the pole to the house
    Real dropHeight = 4.2;        // m at the house end
};

// ---- Simulation control ----
struct SimParams {
    int substeps = 4;              // per 60 Hz host step: 240 Hz internal
    Real hostStep = 1.0 / 60.0;    // s (the game's fixed physics step)
    Real wakeDisplacement = 0.02;  // m of pole-top movement that wakes its spans
    int wakeNeighbours = 2;        // poles away whose spans wake too
    Real sleepSpeed = 0.03;        // m/s
    Real sleepTime = 1.0;          // s below sleepSpeed before a span or pole sleeps
    Real quietTime = 4.0;          // s after the last structural event: creeping bodies settle
    Real impactStep = 5e-5;        // s, sub-step of the impact pulse
    Real carCrushStiffness = 4.0e5;  // N/m default car stiffness against a pole (narrow impact)
    Real hazardRadius = 4.0;       // m around a live MV wire on the ground
    Real arcInterval = 0.6;        // s mean between arcs on a live wire on the ground
    Real mvTripTime = 0.0;         // s until protection trips a downed MV wire (0: stays live)
    Real laneWidth = 3.2;          // m per lane
    Real laneClearance = 2.2;      // m: a wire lower than this over a lane blocks it
    Real transformerDetachTilt = 35 * kDeg;
    Real transformerDetachAccel = 40.0;  // m/s^2 at the tank that tears its brackets (~4 g: 14 kN on a 350 kg tank)
    Real wetness = 0.0;            // 0 dry .. 1 saturated
};

struct Params {
    PoleGeometry geometry;
    PoleStrength strength;
    SoilParams soil;
    WireParams wires;
    SimParams sim;
};

}  // namespace pd
