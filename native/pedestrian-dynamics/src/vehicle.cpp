// Pedestrian Dynamics — physically simulated people.
// SPDX-License-Identifier: MIT
#include "ped/vehicle.hpp"

#include <algorithm>
#include <cmath>

#include "ped/json.hpp"
#include "ped/rot.hpp"

namespace ped {

namespace {

const char* const kRegionNames[] = {"bumper", "grille", "bonnet", "cowl", "windscreen", "a_pillar",
                                    "roof_header", "roof", "body", "ground"};

Real num(const json::Value& o, const char* key, Real fallback) {
    const json::Value* v = o.find(key);
    return v && v->isNumber() ? v->asNumber() : fallback;
}

bool numbers(const json::Value* v, std::size_t n, Real* out) {
    if (!v || !v->isArray() || v->items().size() != n) return false;
    for (std::size_t i = 0; i < n; ++i) {
        if (!v->items()[i].isNumber()) return false;
        out[i] = v->items()[i].asNumber();
    }
    return true;
}

}  // namespace

const char* regionName(Region r) {
    const int i = static_cast<int>(r);
    return i >= 0 && i < static_cast<int>(Region::Count) ? kRegionNames[i] : "?";
}

int VehicleLibrary::index(std::string_view id) const {
    for (std::size_t i = 0; i < profiles_.size(); ++i)
        if (profiles_[i].id == id) return static_cast<int>(i);
    return -1;
}

std::string VehicleLibrary::load(std::string_view text) {
    const json::ParseResult pr = json::parse(std::string(text));
    if (!pr.ok()) return pr.error;
    const json::Value* profs = pr.value.find("profiles");
    if (!profs || !profs->isObject()) return "profiles missing";
    for (const auto& [id, p] : profs->members()) {
        VehicleProfile vp;
        vp.id = id;
        if (const json::Value* n = p.find("name"); n && n->isString()) vp.name = n->asString();
        vp.mass = num(p, "mass", vp.mass);
        if (const json::Value* k = p.find("key")) {
            vp.front = num(*k, "front", vp.front);
            vp.roofFront = num(*k, "roof_front", vp.roofFront);
            vp.roofHeight = num(*k, "roof_height", vp.roofHeight);
            vp.halfWidth = num(*k, "half_width", vp.halfWidth);
            vp.bleHeight = num(*k, "ble_height", vp.bleHeight);
        }
        const json::Value* parts = p.find("parts");
        if (!parts || !parts->isArray()) return id + ": parts missing";
        for (const json::Value& q : parts->items()) {
            VehiclePart part;
            const json::Value* rg = q.find("region");
            if (!rg || !rg->isString()) return id + ": part without a region";
            int r = -1;
            for (int i = 0; i < static_cast<int>(Region::Ground); ++i)
                if (rg->asString() == kRegionNames[i]) r = i;
            if (r < 0) return id + ": unknown region " + rg->asString();
            part.region = static_cast<Region>(r);
            if (const json::Value* box = q.find("box")) {
                Real mn[3], mx[3];
                if (!numbers(box->find("min"), 3, mn) || !numbers(box->find("max"), 3, mx)) return id + ": box min/max";
                part.centre = {0.5 * (mn[0] + mx[0]), 0.5 * (mn[1] + mx[1]), 0.5 * (mn[2] + mx[2])};
                part.half = {0.5 * (mx[0] - mn[0]), 0.5 * (mx[1] - mn[1]), 0.5 * (mx[2] - mn[2])};
            } else if (const json::Value* slab = q.find("slab")) {
                // top surface from (x1, z1) to (x2, z2) in side view; the box hangs below it
                const json::Value* line = slab->find("line");
                Real p1[2], p2[2], yr[2];
                if (!line || !line->isArray() || line->items().size() != 2 || !numbers(&line->items()[0], 2, p1) ||
                    !numbers(&line->items()[1], 2, p2) || !numbers(slab->find("y"), 2, yr))
                    return id + ": slab line/y";
                Real dx = p2[0] - p1[0], dz = p2[1] - p1[1];
                if (dx < 0) dx = -dx, dz = -dz;  // local x points forward
                const Real len = std::sqrt(dx * dx + dz * dz);
                if (len <= 0) return id + ": empty slab";
                dx /= len, dz /= len;
                const Real thick = num(*slab, "thickness", 0.3);
                const Vec3 up{-dz, 0, dx};  // perpendicular to the line, upward
                const Vec3 mid{0.5 * (p1[0] + p2[0]), 0.5 * (yr[0] + yr[1]), 0.5 * (p1[1] + p2[1])};
                part.centre = mid - up * (0.5 * thick);
                part.half = {0.5 * len, 0.5 * (yr[1] - yr[0]), 0.5 * thick};
                part.orientation = axisAngle({0, 1, 0}, dm::atan2(-dz, dx));
            } else {
                return id + ": a part needs a box or a slab";
            }
            if (part.half.x <= 0 || part.half.y <= 0 || part.half.z <= 0) return id + ": empty part";
            part.stiffness = num(q, "stiffness", part.stiffness);
            part.damping = num(q, "damping", part.damping);
            part.friction = num(q, "friction", part.friction);
            vp.parts.push_back(part);
        }
        // a bounding sphere about the parts
        Vec3 lo{1e9, 1e9, 1e9}, hi{-1e9, -1e9, -1e9};
        for (const auto& part : vp.parts) {
            const Real r = length(part.half);
            lo = {std::min(lo.x, part.centre.x - r), std::min(lo.y, part.centre.y - r), std::min(lo.z, part.centre.z - r)};
            hi = {std::max(hi.x, part.centre.x + r), std::max(hi.y, part.centre.y + r), std::max(hi.z, part.centre.z + r)};
        }
        vp.boundCentre = (lo + hi) * 0.5;
        vp.boundRadius = 0.5 * length(hi - lo);
        const int existing = index(id);
        if (existing >= 0) profiles_[static_cast<std::size_t>(existing)] = vp;
        else profiles_.push_back(vp);
    }
    return "";
}

}  // namespace ped
