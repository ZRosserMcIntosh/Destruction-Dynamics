// Pedestrian Dynamics — physically simulated people.
// SPDX-License-Identifier: MIT
#include "ped/body.hpp"

#include <cmath>

#include "ped/json.hpp"

namespace ped {

namespace {

Real num(const json::Value& o, const char* key, Real fallback) {
    const json::Value* v = o.find(key);
    return v && v->isNumber() ? v->asNumber() : fallback;
}

bool vec(const json::Value& o, const char* key, Vec3& out) {
    const json::Value* v = o.find(key);
    if (!v || !v->isArray() || v->items().size() != 3) return false;
    for (const auto& it : v->items())
        if (!it.isNumber()) return false;
    out = {v->items()[0].asNumber(), v->items()[1].asNumber(), v->items()[2].asNumber()};
    return true;
}

bool range(const json::Value& o, const char* key, Real& lo, Real& hi) {
    const json::Value* v = o.find(key);
    if (!v || !v->isArray() || v->items().size() != 2) return false;
    lo = v->items()[0].asNumber() * kDeg;
    hi = v->items()[1].asNumber() * kDeg;
    return lo <= hi;
}

// Reflection in the sagittal plane: points (x, -y, z); rotation axes are pseudovectors
// (-x, y, -z), so a mirrored joint keeps the meaning of its angles (flexion stays flexion).
Vec3 mirrorPoint(const Vec3& p) { return {p.x, -p.y, p.z}; }
Vec3 mirrorAxis(const Vec3& a) { return {-a.x, a.y, -a.z}; }

std::string mirrorName(const std::string& n) {
    if (n.size() > 2 && n.compare(n.size() - 2, 2, "_l") == 0) return n.substr(0, n.size() - 2) + "_r";
    return n;
}

// A solid capsule's principal moments: (transverse, transverse, axial).
void capsuleInertia(Real m, Real r, Real len, Real& trans, Real& axial) {
    const Real vc = kPi * r * r * len, vh = 4.0 / 3.0 * kPi * r * r * r;
    const Real mc = m * vc / (vc + vh), mh = m * vh / (vc + vh);
    axial = mc * r * r / 2 + mh * 2.0 / 5.0 * r * r;
    const Real d = len / 2 + 3.0 * r / 8.0;
    trans = mc * (r * r / 4 + len * len / 12) + mh * (83.0 / 320.0 * r * r + d * d);
}

}  // namespace

int BodyDef::index(std::string_view name) const {
    for (std::size_t i = 0; i < segments_.size(); ++i)
        if (segments_[i].name == name) return static_cast<int>(i);
    return -1;
}

std::string BodyDef::load(std::string_view text) {
    const json::ParseResult pr = json::parse(std::string(text));
    if (!pr.ok()) return pr.error;
    const json::Value& root = pr.value;
    if (const json::Value* ref = root.find("reference")) {
        refStature = num(*ref, "stature", refStature);
        refMass = num(*ref, "mass", refMass);
    }
    const json::Value* segs = root.find("segments");
    if (!segs || !segs->isArray()) return "segments missing";
    std::vector<SegmentDef> out;
    for (const json::Value& s : segs->items()) {
        SegmentDef d;
        const json::Value* name = s.find("name");
        if (!name || !name->isString()) return "segment without a name";
        d.name = name->asString();
        if (const json::Value* p = s.find("parent"); p && p->isString()) {
            for (std::size_t i = 0; i < out.size(); ++i)
                if (out[i].name == p->asString()) d.parent = static_cast<int>(i);
            if (d.parent < 0) return d.name + ": parent " + p->asString() + " must come first";
        }
        d.massPct = num(s, "mass_pct", 0);
        if (!vec(s, "from", d.from) || !vec(s, "to", d.to)) return d.name + ": from/to";
        d.radius = num(s, "radius", 0);
        d.tissueStiffness = num(s, "tissue_stiffness", 1e6);
        if (d.massPct <= 0 || d.radius <= 0) return d.name + ": mass and radius must be positive";
        if (d.parent >= 0) {
            const json::Value* j = s.find("joint");
            if (!j || !j->isObject()) return d.name + ": joint missing";
            JointDef& jd = d.joint;
            if (const json::Value* jn = j->find("name"); jn && jn->isString()) jd.name = jn->asString();
            if (!vec(*j, "at", jd.at) || !vec(*j, "twist_axis", jd.twistAxis) || !vec(*j, "flex_axis", jd.flexAxis) ||
                !vec(*j, "abd_axis", jd.abdAxis))
                return d.name + ": joint axes";
            if (!range(*j, "flex", jd.flexMin, jd.flexMax) || !range(*j, "abd", jd.abdMin, jd.abdMax) ||
                !range(*j, "twist", jd.twistMin, jd.twistMax))
                return d.name + ": joint ranges";
            jd.twistAxis = normalized(jd.twistAxis);
            jd.flexAxis = normalized(jd.flexAxis);
            jd.abdAxis = normalized(jd.abdAxis);
            if (std::abs(dot(jd.twistAxis, jd.flexAxis)) > 1e-9 || std::abs(dot(jd.twistAxis, jd.abdAxis)) > 1e-9 ||
                std::abs(dot(jd.flexAxis, jd.abdAxis)) > 1e-9)
                return d.name + ": joint axes must be perpendicular";
            jd.kp = num(*j, "kp", 0);
            jd.torque = num(*j, "torque", 0);
            jd.limitStiffness = num(*j, "limit_stiffness", 1000);
            jd.tearTorque = num(*j, "tear_torque", 1e9);
        }
        out.push_back(d);
        if (const json::Value* m = s.find("mirror"); m && m->isString()) {
            SegmentDef r = d;
            r.name = m->asString();
            if (d.parent >= 0) {
                const std::string pn = mirrorName(out[static_cast<std::size_t>(d.parent)].name);
                r.parent = -1;
                for (std::size_t i = 0; i < out.size(); ++i)
                    if (out[i].name == pn) r.parent = static_cast<int>(i);
                if (r.parent < 0) return r.name + ": mirrored parent " + pn + " not found";
            }
            r.from = mirrorPoint(d.from);
            r.to = mirrorPoint(d.to);
            r.joint.name = mirrorName(d.joint.name);
            r.joint.at = mirrorPoint(d.joint.at);
            r.joint.twistAxis = mirrorAxis(d.joint.twistAxis);
            r.joint.flexAxis = mirrorAxis(d.joint.flexAxis);
            r.joint.abdAxis = mirrorAxis(d.joint.abdAxis);
            out.push_back(r);
        }
    }
    if (out.empty() || out[0].parent >= 0) return "the first segment must be the root";
    if (static_cast<int>(out.size()) > kMaxSegments) return "too many segments";
    Real total = 0;
    for (const auto& d : out) total += d.massPct;
    if (std::abs(total - 100.0) > 0.5) return "segment masses add up to " + std::to_string(total) + " %";
    segments_ = std::move(out);
    return "";
}

BuiltBody buildBody(const BodyDef& def, Real stature, Real mass) {
    BuiltBody b;
    b.count = def.size();
    b.stature = stature;
    b.mass = mass;
    const Real gainScale = (mass * stature) / (def.refMass * def.refStature);
    Real total = 0;
    for (int i = 0; i < def.size(); ++i) total += def.at(i).massPct;
    Vec3 com;
    for (int i = 0; i < def.size(); ++i) {
        const SegmentDef& d = def.at(i);
        BuiltSegment& s = b.seg[static_cast<std::size_t>(i)];
        s.a = d.from * stature;
        s.b = d.to * stature;
        s.centre = (s.a + s.b) * 0.5;
        s.radius = d.radius * stature;
        s.halfLength = 0.5 * length(s.b - s.a);
        s.mass = mass * d.massPct / total;
        Real trans = 0, axial = 0;
        capsuleInertia(s.mass, s.radius, 2 * s.halfLength, trans, axial);
        // the capsule's axis is (nearly) one of the body axes: that one gets the axial moment
        const Vec3 ax = s.b - s.a;
        const Real ex = std::abs(ax.x), ey = std::abs(ax.y), ez = std::abs(ax.z);
        s.inertia = (ex >= ey && ex >= ez) ? Vec3{axial, trans, trans} : (ey >= ez ? Vec3{trans, axial, trans} : Vec3{trans, trans, axial});
        s.tissueStiffness = d.tissueStiffness;
        s.parent = d.parent;
        if (d.parent >= 0) {
            s.jointAt = d.joint.at * stature;
            s.kp = d.joint.kp * gainScale;
            s.torque = d.joint.torque * gainScale;
            s.limitStiffness = d.joint.limitStiffness * gainScale;
            s.tearTorque = d.joint.tearTorque * gainScale;
        }
        com += s.centre * s.mass;
    }
    b.com = com / mass;
    return b;
}

}  // namespace ped
