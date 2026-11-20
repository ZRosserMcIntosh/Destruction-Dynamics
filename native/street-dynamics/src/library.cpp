// Street Dynamics — destructible street furniture.
// SPDX-License-Identifier: MIT
#include "sd/library.hpp"

#include <algorithm>
#include <cmath>

#include "sd/json.hpp"

namespace sd {

namespace {

constexpr Real kConcreteDensity = 2400;
constexpr Real kConcreteStrength = 25e6;  // Pa, C25 fill
constexpr Real kRefYield = 235e6;         // EN 1993-1-1: epsilon^2 = 235 MPa / f_y

}  // namespace

Real PostMechanics::capacity(Real plastic) const {
    const Real start = std::min(yieldMoment, peakMoment);
    if (plastic <= buckleRotation)
        return buckleRotation > 0 ? start + (peakMoment - start) * plastic / buckleRotation : peakMoment;
    const Real past = plastic - buckleRotation;
    if (past < softenRotation) return peakMoment + (residualMoment - peakMoment) * past / softenRotation;
    return residualMoment;
}

PostMechanics deriveMechanics(const Section& s, const Material& m) {
    PostMechanics r;
    const Real D = s.d, t = std::min(s.t, 0.5 * s.d);
    Real coreArea = 0, slenderness = 0;
    switch (s.kind) {
        case SectionKind::RoundTube: {
            const Real di = D - 2 * t;
            r.area = 0.25 * kPi * (D * D - di * di);
            r.inertia = kPi / 64 * (D * D * D * D - di * di * di * di);
            r.elasticModulus = r.inertia / (0.5 * D);
            r.plasticModulus = (D * D * D - di * di * di) / 6;
            coreArea = 0.25 * kPi * di * di;
            slenderness = (D / t) * (m.yield / kRefYield);  // class limits 50, 70, 90 (Table 5.2)
            r.crossSectionClass = slenderness <= 50 ? 1 : slenderness <= 70 ? 2 : slenderness <= 90 ? 3 : 4;
            break;
        }
        case SectionKind::SquareTube:
        case SectionKind::RectTube: {
            // Bending about the weak axis: B across the bending plane.
            const Real depth = D, width = s.kind == SectionKind::RectTube && s.b > 0 ? s.b : D;
            const Real B = std::min(depth, width), H = std::max(depth, width);
            const Real bi = B - 2 * t, hi = H - 2 * t;
            r.area = B * H - bi * hi;
            r.inertia = (H * B * B * B - hi * bi * bi * bi) / 12;
            r.elasticModulus = r.inertia / (0.5 * B);
            r.plasticModulus = (H * B * B - hi * bi * bi) / 4;
            coreArea = bi * hi;
            // internal compression wall c/t against 33, 38, 42 epsilon
            slenderness = (H - 3 * t) / t / std::sqrt(kRefYield / m.yield);
            r.crossSectionClass = slenderness <= 33 ? 1 : slenderness <= 38 ? 2 : slenderness <= 42 ? 3 : 4;
            slenderness *= 50.0 / 33.0;  // on the round tube's scale, for class 4 below
            break;
        }
        case SectionKind::SolidRound:
            r.area = 0.25 * kPi * D * D;
            r.inertia = kPi * D * D * D * D / 64;
            r.elasticModulus = kPi * D * D * D / 32;
            r.plasticModulus = D * D * D / 6;
            break;
        case SectionKind::SolidSquare:
            r.area = D * D;
            r.inertia = D * D * D * D / 12;
            r.elasticModulus = D * D * D / 6;
            r.plasticModulus = D * D * D / 4;
            break;
    }
    r.massPerMetre = r.area * m.density + (s.concreteFilled ? coreArea * kConcreteDensity : 0);
    r.bendStiffness = m.youngs * r.inertia;
    r.yieldMoment = m.yield * r.elasticModulus;
    r.fractureRotation = m.hingeDuctility;
    if (m.brittle) {
        // Concrete, cast iron, timber: elastic to the rupture moment (modulus of rupture x W),
        // then the section cracks through and keeps only its residual share.
        r.plasticMoment = r.peakMoment = r.yieldMoment;
        r.residualMoment = m.residual * r.peakMoment;
        r.buckleRotation = 0;
        r.softenRotation = 0.01;
        r.fractureRotation = m.residual > 0 ? m.hingeDuctility : 0.012;
        r.crossSectionClass = 0;
        return r;
    }
    r.plasticMoment = m.yield * r.plasticModulus;
    if (s.concreteFilled && coreArea > 0) {
        // The concrete core takes compression and holds the wall out against local buckling
        // (concrete-filled steel tubes; estimated: + 40 % of the core's squash load ratio, at
        // most + 50 %).
        const Real ratio = (kConcreteStrength * coreArea) / (m.yield * r.area);
        r.plasticMoment *= 1.0 + std::min(0.5, 0.4 * ratio);
        r.crossSectionClass = 1;
    }
    // The hinge law by wall slenderness (estimated, after tube bending tests): stocky walls
    // harden past M_p and keep turning; slender walls buckle locally (ovalise and kink) near
    // first yield, and the kink carries much less once it forms.
    switch (r.crossSectionClass) {
        case 1:
            r.peakMoment = 1.15 * r.plasticMoment;
            r.buckleRotation = 0.15;
            r.softenRotation = 0.35;
            r.residualMoment = 0.40 * r.plasticMoment;
            break;
        case 2:
            r.peakMoment = r.plasticMoment;
            r.buckleRotation = 0.06;
            r.softenRotation = 0.30;
            r.residualMoment = 0.35 * r.plasticMoment;
            break;
        case 3:
            r.peakMoment = r.yieldMoment;
            r.buckleRotation = 0.02;
            r.softenRotation = 0.25;
            r.residualMoment = 0.30 * r.plasticMoment;
            break;
        default:
            r.peakMoment = r.yieldMoment * std::min(1.0, 90.0 / std::max(slenderness, 1.0));
            r.buckleRotation = 0.01;
            r.softenRotation = 0.20;
            r.residualMoment = 0.25 * r.plasticMoment;
            break;
    }
    if (s.concreteFilled) r.residualMoment = std::max(r.residualMoment, 0.6 * r.plasticMoment);
    return r;
}

Real panelArea(const PanelDef& p) {
    switch (p.shape) {
        case PanelShape::Circle: return 0.25 * kPi * p.width * p.width;
        case PanelShape::Octagon: return 2.0 * (std::sqrt(2.0) - 1.0) * p.width * p.width;  // flat to flat
        case PanelShape::Triangle: return 0.25 * std::sqrt(3.0) * p.width * p.width;       // equilateral, side
        case PanelShape::Diamond: return 0.5 * p.width * p.width;                          // across the corners
        case PanelShape::Rect: break;
    }
    return p.width * p.height;
}

namespace {

Real num(const json::Value& o, const char* key, Real fallback) {
    const json::Value* v = o.find(key);
    return v && v->isNumber() ? v->asNumber() : fallback;
}
std::string str(const json::Value& o, const char* key, const std::string& fallback = "") {
    const json::Value* v = o.find(key);
    return v && v->isString() ? v->asString() : fallback;
}
bool flag(const json::Value& o, const char* key, bool fallback) {
    const json::Value* v = o.find(key);
    return v && v->isBool() ? v->asBool() : fallback;
}

}  // namespace

std::string Library::load(std::string_view text) {
    const json::ParseResult pr = json::parse(std::string(text));
    if (!pr.ok()) return pr.error;
    const json::Value& root = pr.value;
    if (const json::Value* mats = root.find("materials"); mats && mats->isObject()) {
        for (const auto& [id, v] : mats->members()) {
            Material m;
            m.id = id;
            m.youngs = num(v, "youngs", m.youngs);
            m.yield = num(v, "yield", m.yield);
            m.density = num(v, "density", m.density);
            m.brittle = flag(v, "brittle", false);
            m.residual = num(v, "residual", 0);
            m.hingeDuctility = num(v, "hinge_ductility", m.hingeDuctility);
            auto it = std::find_if(materials_.begin(), materials_.end(), [&](const Material& x) { return x.id == id; });
            if (it != materials_.end()) *it = m;
            else materials_.push_back(m);
        }
    }
    const json::Value* types = root.find("types");
    if (!types || !types->isObject()) return types ? "types must be an object" : "";
    for (const auto& [id, v] : types->members()) {
        Archetype a;
        a.id = id;
        a.name = str(v, "name", id);
        a.family = str(v, "family", "post");
        a.source = str(v, "source");
        a.height = num(v, "height", a.height);
        a.segments = static_cast<int>(num(v, "segments", 0));
        a.legs = std::clamp(static_cast<int>(num(v, "legs", 1)), 1, 2);
        a.legSpacing = num(v, "leg_spacing", 0);
        if (const json::Value* s = v.find("section"); s && s->isObject()) {
            const std::string k = str(*s, "kind", "round_tube");
            a.section.kind = k == "square_tube" ? SectionKind::SquareTube
                           : k == "rect_tube"   ? SectionKind::RectTube
                           : k == "solid_round" ? SectionKind::SolidRound
                           : k == "solid_square" ? SectionKind::SolidSquare
                                                 : SectionKind::RoundTube;
            a.section.d = num(*s, "d", a.section.d);
            a.section.b = num(*s, "b", 0);
            a.section.t = num(*s, "t", a.section.t);
            a.section.concreteFilled = flag(*s, "concrete_filled", false);
        }
        a.materialId = str(v, "material", "steel_s235");
        auto mat = std::find_if(materials_.begin(), materials_.end(), [&](const Material& x) { return x.id == a.materialId; });
        if (mat == materials_.end()) return "type " + id + ": unknown material " + a.materialId;
        a.material = *mat;
        if (const json::Value* b = v.find("base"); b && b->isObject()) {
            const std::string k = str(*b, "kind", "embedded");
            a.base.kind = k == "slip_base" ? BaseKind::SlipBase
                        : k == "socket"    ? BaseKind::Socket
                        : k == "bolted"    ? BaseKind::Bolted
                        : k == "footing"   ? BaseKind::Footing
                                           : BaseKind::Embedded;
            a.base.release = num(*b, "release", 0);
            a.base.boltMoment = num(*b, "bolt_moment", 0);
            a.base.footingStiffness = num(*b, "footing_stiffness", 0);
            a.base.footingYield = num(*b, "footing_yield", 0);
            a.base.footingFail = num(*b, "footing_fail", a.base.footingFail);
            a.base.socketAngle = num(*b, "socket_angle", a.base.socketAngle);
        }
        if (const json::Value* ps = v.find("panels"); ps && ps->isArray()) {
            for (const json::Value& p : ps->items()) {
                PanelDef d;
                const std::string sh = str(p, "shape", "rect");
                d.shape = sh == "circle" ? PanelShape::Circle
                        : sh == "octagon" ? PanelShape::Octagon
                        : sh == "triangle" ? PanelShape::Triangle
                        : sh == "diamond" ? PanelShape::Diamond
                                          : PanelShape::Rect;
                d.width = num(p, "width", d.width);
                d.height = num(p, "height", d.shape == PanelShape::Rect ? d.height : d.width);
                d.thickness = num(p, "thickness", d.thickness);
                d.density = num(p, "density", d.density);
                d.centre = num(p, "centre", d.centre);
                d.offset = num(p, "offset", d.offset);
                d.yaw = num(p, "yaw_deg", 0) * kDeg;
                d.clamp = num(p, "clamp", d.clamp);
                d.legend = str(p, "legend");
                if (a.panels.size() < 4) a.panels.push_back(d);
            }
        }
        a.mech = deriveMechanics(a.section, a.material);
        if (a.segments <= 0) a.segments = std::clamp(static_cast<int>(std::lround(a.height / 0.35)), 2, 10);
        a.postMass = a.mech.massPerMetre * a.height * a.legs + (a.legs > 1 ? a.mech.massPerMetre * a.legSpacing : 0);
        a.totalMass = a.postMass;
        for (const PanelDef& p : a.panels) a.totalMass += panelArea(p) * p.thickness * p.density;
        auto it = std::find_if(types_.begin(), types_.end(), [&](const Archetype& x) { return x.id == id; });
        if (it != types_.end()) *it = a;
        else types_.push_back(a);
    }
    return "";
}

const Archetype* Library::find(std::string_view id) const {
    const int i = index(id);
    return i >= 0 ? &types_[static_cast<std::size_t>(i)] : nullptr;
}

int Library::index(std::string_view id) const {
    for (std::size_t i = 0; i < types_.size(); ++i)
        if (types_[i].id == id) return static_cast<int>(i);
    return -1;
}

}  // namespace sd
