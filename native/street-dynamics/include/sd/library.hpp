// Street Dynamics — destructible street furniture.
// SPDX-License-Identifier: MIT
//
// The object library: every kind of post, sign, bollard and rack is data
// (data/street_library.json), not code. A type gives its post (section,
// material, height), how it meets the ground (its base), and what hangs on
// it (sign panels). The physical properties the solver needs (stiffness,
// plastic moment, the moment-rotation law of its hinges, masses) are derived
// here from the section's geometry and the material, so a new type is a new
// JSON entry. Units: SI (m, kg, N, Pa, rad).
#pragma once

#include <array>
#include <string>
#include <string_view>
#include <vector>

#include "sd/math.hpp"

namespace sd {

enum class SectionKind : int {
    RoundTube = 0,   // d outer diameter, t wall
    SquareTube = 1,  // d side, t wall
    RectTube = 2,    // d depth, b width, t wall (bends about its weak axis)
    SolidRound = 3,  // d diameter (steel bar, timber, cast iron, concrete)
    SolidSquare = 4, // d side
};

struct Material {
    std::string id;
    Real youngs = 200e9;     // Pa
    Real yield = 250e6;      // Pa (steel); modulus of rupture for brittle materials
    Real density = 7850;     // kg/m^3
    bool brittle = false;    // concrete, cast iron, timber: cracks instead of yielding
    Real residual = 0;       // brittle: share of the rupture moment the cracked section keeps
    Real hingeDuctility = 1.6;  // rad of plastic rotation a hinge takes before it tears
};

struct Section {
    SectionKind kind = SectionKind::RoundTube;
    Real d = 0.0603, b = 0, t = 0.003;  // m
    bool concreteFilled = false;
};

enum class BaseKind : int {
    Embedded = 0,  // set in concrete: the post itself hinges at the ground line
    SlipBase = 1,  // breakaway slip base: lets go at a shear force
    Socket = 2,    // in a ground sleeve: tips out of it past an angle
    Bolted = 3,    // base plate on anchor bolts: the bolts fail at a moment
    Footing = 4,   // a stiff post in a concrete block in the soil: the block rotates
};

struct BaseDef {
    BaseKind kind = BaseKind::Embedded;
    Real release = 0;           // N, slip base shear
    Real boltMoment = 0;        // N*m, bolted base
    Real footingStiffness = 0;  // N*m/rad, rotation of the footing in the soil (0: rigid)
    Real footingYield = 0;      // N*m, the soil gives (0: never)
    Real footingFail = 0.35;    // rad of footing rotation past which it lets go
    Real socketAngle = 0.25;    // rad, socket: tilt past which the post leaves its sleeve
};

enum class PanelShape : int { Rect = 0, Circle = 1, Octagon = 2, Triangle = 3, Diamond = 4 };

struct PanelDef {
    PanelShape shape = PanelShape::Rect;
    Real width = 0.6, height = 0.6, thickness = 0.002;  // m
    Real density = 2700;     // kg/m^3 (aluminium)
    Real centre = 2.3;       // m above ground, the panel's centre
    Real offset = 0.04;      // m in front of the post axis (along the facing direction)
    Real yaw = 0;            // rad, panel facing relative to the object's heading
    Real clamp = 3000;       // N the clamps hold before they let go
    std::string legend;      // for the renderer ("PARE", "60", street name...)
};

// Derived from the section and material: what the solver uses.
struct PostMechanics {
    Real area = 0, inertia = 0, plasticModulus = 0, elasticModulus = 0;  // m^2, m^4, m^3, m^3
    Real massPerMetre = 0;     // kg/m
    Real bendStiffness = 0;    // E I, N*m^2
    Real yieldMoment = 0;      // N*m, first yield (f_y W)
    Real plasticMoment = 0;    // N*m (f_y Z), or the rupture moment for brittle materials
    Real peakMoment = 0;       // N*m the hinge reaches (hardening, or less for slender walls)
    Real residualMoment = 0;   // N*m after local buckling (or cracking)
    Real buckleRotation = 0;   // rad of plastic rotation at the peak (local buckling begins)
    Real softenRotation = 0;   // rad over which it falls to the residual
    Real fractureRotation = 0; // rad of plastic rotation at which the hinge tears
    int crossSectionClass = 1; // EN 1993-1-1 style class (1..4) of the wall slenderness
    // The hinge's moment capacity after `plastic` rad of plastic rotation.
    Real capacity(Real plastic) const;
};

struct Archetype {
    std::string id, name, family, source;
    Real height = 2.8;           // m above ground (top of the post)
    Section section;
    std::string materialId;
    Material material;
    BaseDef base;
    std::vector<PanelDef> panels;
    int segments = 0;            // rigid segments along the post (0: from the height)
    int legs = 1;                // 2: a frame (an inverted-U bike rack): two posts and a top bar
    Real legSpacing = 0;         // m between the legs (along the object's heading)
    PostMechanics mech;
    Real postMass = 0, totalMass = 0;  // kg
};

class Library {
public:
    // Loads (or adds to) the library from JSON; returns "" or an error.
    std::string load(std::string_view json);
    const Archetype* find(std::string_view id) const;
    int index(std::string_view id) const;
    const Archetype& at(int i) const { return types_[static_cast<std::size_t>(i)]; }
    int size() const { return static_cast<int>(types_.size()); }
    const std::vector<Material>& materials() const { return materials_; }

private:
    std::vector<Material> materials_;
    std::vector<Archetype> types_;
};

// Section properties and the hinge law (exposed for the tests and the README).
PostMechanics deriveMechanics(const Section& s, const Material& m);
Real panelArea(const PanelDef& p);

}  // namespace sd
