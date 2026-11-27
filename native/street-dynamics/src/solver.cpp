// Street Dynamics — destructible street furniture.
// SPDX-License-Identifier: MIT
//
// The solver: XPBD rigid bodies (Müller, Macklin, Chentanez, Jeschke, Kim
// 2020). Per substep: predict (gravity, air on loose plates), solve the
// joints, the ground and the host bodies once each, derive velocities, then
// let the hinges yield, buckle and tear. Everything iterates in index order:
// deterministic.
#include <algorithm>
#include <chrono>
#include <cmath>

#include "sd/dmath.hpp"
#include "sd/world.hpp"

namespace sd {

namespace {

Quat axisAngle(const Vec3& axis, Real angle) {
    Real s, c;
    dm::sincos(0.5 * angle, s, c);
    return {c, axis.x * s, axis.y * s, axis.z * s};
}

// The rotation vector of a unit quaternion (shortest way round).
Vec3 rotationVector(Quat d) {
    if (d.w < 0) d = {-d.w, -d.x, -d.y, -d.z};
    const Real s = std::sqrt(d.x * d.x + d.y * d.y + d.z * d.z);
    if (s < 1e-12) return {2 * d.x, 2 * d.y, 2 * d.z};
    const Real angle = 2.0 * dm::atan2(s, d.w);
    return Vec3{d.x, d.y, d.z} * (angle / s);
}

// Adds a small rotation (world vector) to q.
Quat addRotation(const Quat& q, const Vec3& v) {
    const Quat dq = Quat{0, v.x, v.y, v.z} * q;
    return normalized(Quat{q.w + 0.5 * dq.w, q.x + 0.5 * dq.x, q.y + 0.5 * dq.y, q.z + 0.5 * dq.z});
}

// Signed distance from p to an oriented box (negative inside) and the outward normal there.
// `approach` (optional) is the box's velocity relative to the point: a point inside a moving box
// leaves through the face it came in by (the one that reached it soonest), not the nearest one,
// so a fast box never flips a post over its top or out of its back.
constexpr Real kMaxEntryTime = 0.05;  // s

Real boxDistance(const Vec3& p, const Vec3& c, const Quat& q, const Vec3& half, Vec3& normal, const Vec3& approach = {}) {
    const Vec3 l = rotateInv(q, p - c);
    const Vec3 d{std::abs(l.x) - half.x, std::abs(l.y) - half.y, std::abs(l.z) - half.z};
    if (d.x > 0 || d.y > 0 || d.z > 0) {
        const Vec3 o{maxr(d.x, 0) * (l.x < 0 ? -1 : 1), maxr(d.y, 0) * (l.y < 0 ? -1 : 1), maxr(d.z, 0) * (l.z < 0 ? -1 : 1)};
        const Real len = length(o);
        normal = rotate(q, o / len);
        return len;
    }
    // inside: out through the face it entered by, else the nearest
    const Vec3 v = rotateInv(q, approach);
    const Real vl[3] = {v.x, v.y, v.z}, ll[3] = {l.x, l.y, l.z}, dl[3] = {d.x, d.y, d.z};
    int best = -1;
    Real bestT = 1e300;
    for (int ax = 0; ax < 3; ++ax) {
        // the face on the point's side leads when the box moves toward the point through it
        const Real s = ll[ax] < 0 ? -1.0 : 1.0;
        const Real closing = vl[ax] * s;
        if (closing <= 1e-3) continue;
        const Real tEntry = -dl[ax] / closing;
        // only a face crossed in the last moments: a part riding along with the box closes on
        // its true face at almost nothing, and a slow drift toward a far face must not win
        if (tEntry > kMaxEntryTime) continue;  // how long ago that face passed the point
        if (tEntry < bestT) bestT = tEntry, best = ax;
    }
    if (best >= 0) {
        Vec3 nl{};
        (best == 0 ? nl.x : best == 1 ? nl.y : nl.z) = ll[best] < 0 ? -1.0 : 1.0;
        normal = rotate(q, nl);
        return dl[best];
    }
    if (d.x >= d.y && d.x >= d.z) normal = rotate(q, {l.x < 0 ? -1.0 : 1.0, 0, 0});
    else if (d.y >= d.z) normal = rotate(q, {0, l.y < 0 ? -1.0 : 1.0, 0});
    else normal = rotate(q, {0, 0, l.z < 0 ? -1.0 : 1.0});
    return maxr(d.x, maxr(d.y, d.z));
}

// The point of segment a-b nearest the box (signed distance is convex along a line: a
// ternary search finds its minimum), with the distance, the normal and the parameter.
Real segmentBox(const Vec3& a, const Vec3& b, const Vec3& c, const Quat& q, const Vec3& half, Vec3& point, Vec3& normal,
                Real& t, const Vec3& approach = {}) {
    Real lo = 0, hi = 1;
    Vec3 n;
    for (int it = 0; it < 28; ++it) {
        const Real m1 = lo + (hi - lo) / 3, m2 = hi - (hi - lo) / 3;
        if (boxDistance(a + (b - a) * m1, c, q, half, n) <= boxDistance(a + (b - a) * m2, c, q, half, n)) hi = m2;
        else lo = m1;
    }
    t = 0.5 * (lo + hi);
    point = a + (b - a) * t;
    return boxDistance(point, c, q, half, normal, approach);
}

// Nearest points of segments p1-q1 and p2-q2 (Ericson, Real-Time Collision Detection 5.1.9).
void segmentSegment(const Vec3& p1, const Vec3& q1, const Vec3& p2, const Vec3& q2, Real& s, Real& t) {
    const Vec3 d1 = q1 - p1, d2 = q2 - p2, r = p1 - p2;
    const Real a = dot(d1, d1), e = dot(d2, d2), f = dot(d2, r);
    if (a <= 1e-12 && e <= 1e-12) { s = t = 0; return; }
    if (a <= 1e-12) { s = 0; t = clamp(f / e, 0, 1); return; }
    const Real c = dot(d1, r);
    if (e <= 1e-12) { t = 0; s = clamp(-c / a, 0, 1); return; }
    const Real b = dot(d1, d2), denom = a * e - b * b;
    s = denom > 1e-12 ? clamp((b * f - c * e) / denom, 0, 1) : 0;
    t = (b * s + f) / e;
    if (t < 0) { t = 0; s = clamp(-c / a, 0, 1); }
    else if (t > 1) { t = 1; s = clamp((b - c) / a, 0, 1); }
}

}  // namespace

// ---------------------------------------------------------------- XPBD primitives

Vec3 World::invInertiaWorld(const Body& b, const Vec3& v) const {
    const Vec3 l = rotateInv(b.q, v);
    return rotate(b.q, {l.x * b.invInertia.x, l.y * b.invInertia.y, l.z * b.invInertia.z});
}

Real World::genInvMass(const Body& b, const Vec3& r, const Vec3& n) const {
    const Vec3 rn = cross(r, n);
    return b.invMass + dot(rn, invInertiaWorld(b, rn));
}

// Moves a by +corr relative to b at the attachment offsets ra, rb (world, from the centres).
void World::applyPositional(Body* a, Body* b, const Vec3& ra, const Vec3& rb, const Vec3& corr, Real alpha, Real& lambda) {
    const Real c = length(corr);
    if (c < 1e-12) return;
    const Vec3 n = corr / c;
    const Real wa = a ? genInvMass(*a, ra, n) : 0, wb = b ? genInvMass(*b, rb, n) : 0;
    const Real dl = (c - alpha * lambda) / (wa + wb + alpha);
    lambda += dl;
    const Vec3 p = n * dl;
    if (a) {
        a->x += p * a->invMass;
        a->q = addRotation(a->q, invInertiaWorld(*a, cross(ra, p)));
    }
    if (b) {
        b->x -= p * b->invMass;
        b->q = addRotation(b->q, invInertiaWorld(*b, cross(rb, p)) * -1.0);
    }
}

// Joints: the same, with the multiplier kept as a vector. A joint is solved more than once a
// substep and its error can change direction between passes; a scalar multiplier would then
// push against the new direction with the old one's force and feed energy in.
void World::applyPositional(Body* a, Body* b, const Vec3& ra, const Vec3& rb, const Vec3& corr, Real alpha, Vec3& lambda) {
    const Real c = length(corr);
    if (c < 1e-12) return;
    const Vec3 n = corr / c;
    const Real wa = a ? genInvMass(*a, ra, n) : 0, wb = b ? genInvMass(*b, rb, n) : 0;
    const Real dl = (c - alpha * dot(lambda, n)) / (wa + wb + alpha);
    lambda += n * dl;
    const Vec3 p = n * dl;
    if (a) {
        a->x += p * a->invMass;
        a->q = addRotation(a->q, invInertiaWorld(*a, cross(ra, p)));
    }
    if (b) {
        b->x -= p * b->invMass;
        b->q = addRotation(b->q, invInertiaWorld(*b, cross(rb, p)) * -1.0);
    }
}

void World::applyAngular(Body* a, Body* b, const Vec3& dphi, Real alpha, Vec3& lambda) {
    const Real th = length(dphi);
    if (th < 1e-12) return;
    const Vec3 n = dphi / th;
    const Real wa = a ? dot(n, invInertiaWorld(*a, n)) : 0, wb = b ? dot(n, invInertiaWorld(*b, n)) : 0;
    const Real dl = (th - alpha * dot(lambda, n)) / (wa + wb + alpha);
    lambda += n * dl;
    const Vec3 p = n * dl;
    if (a) a->q = addRotation(a->q, invInertiaWorld(*a, p));
    if (b) b->q = addRotation(b->q, invInertiaWorld(*b, p) * -1.0);
}

// Rotates a by +dphi relative to b.
void World::applyAngular(Body* a, Body* b, const Vec3& dphi, Real alpha, Real& lambda) {
    const Real th = length(dphi);
    if (th < 1e-12) return;
    const Vec3 n = dphi / th;
    const Real wa = a ? dot(n, invInertiaWorld(*a, n)) : 0, wb = b ? dot(n, invInertiaWorld(*b, n)) : 0;
    const Real dl = (th - alpha * lambda) / (wa + wb + alpha);
    lambda += dl;
    const Vec3 p = n * dl;
    if (a) a->q = addRotation(a->q, invInertiaWorld(*a, p));
    if (b) b->q = addRotation(b->q, invInertiaWorld(*b, p) * -1.0);
}

// ---------------------------------------------------------------- joints

namespace {
// Where body a should be turned to for the joint's rest: q_b = q_a * rest (ground: q_b = 1).
Vec3 jointError(const Quat& qa, const Quat* qb, const Quat& rest) {
    const Quat target = (qb ? *qb : Quat{}) * conjugate(rest);
    return rotationVector(target * conjugate(qa));
}
}  // namespace

void World::solveJoint(Joint& j, Real h) {
    if (j.broken) return;
    Body& A = bodies_[static_cast<std::size_t>(j.a)];
    Body* B = j.b >= 0 ? &bodies_[static_cast<std::size_t>(j.b)] : nullptr;
    const Real h2 = h * h;
    const Vec3 ra = rotate(A.q, j.ra), pa = A.x + ra;
    const Vec3 rb = B ? rotate(B->q, j.rb) : Vec3{}, pb = B ? B->x + rb : j.rb;
    applyPositional(&A, B, ra, rb, pb - pa, j.posCompliance / h2, j.lambdaPos);
    const Vec3 dphi = jointError(A.q, B ? &B->q : nullptr, j.rest);
    applyAngular(&A, B, dphi, j.compliance / h2, j.lambdaAng);
}

// After the substep's solve: the hinges that carry more than they can yield; a hinge past its
// ductility tears; bases, clamps and sleeves let go.
void World::plasticity(Object& o, Real h) {
    const Archetype& a = library.at(o.type);
    const Real h2 = h * h;
    // per object +/- 10 % on strengths (its own steel, welds and footing)
    const Real vary = 0.9 + 0.2 * static_cast<Real>(o.variation % 1000u) / 999.0;
    const Real wetSoil = 1.0 - 0.45 * clamp(wetness, 0, 1);
    bool changed = false;
    for (int k = 0; k < o.jointCount; ++k) {
        Joint& j = joints_[static_cast<std::size_t>(o.firstJoint + k)];
        if (j.broken) continue;
        j.lastForce = length(j.lambdaPos) / h2;
        j.lastTorque = length(j.lambdaAng) / h2;
        if (j.kind == 2) {
            // a sign's clamps: pulled or twisted off
            const PanelDef& p = a.panels[static_cast<std::size_t>(j.panel)];
            const Real lever = 0.5 * std::max(p.width, p.height);
            if (j.lastForce > p.clamp * vary || j.lastTorque > p.clamp * vary * lever) {
                j.broken = true;
                changed = true;
                const Body& pb = bodies_[static_cast<std::size_t>(j.b)];
                emit(EventType::PanelDetached, o, j.panel, pb.x, j.lastForce);
            }
            continue;
        }
        Body& A = bodies_[static_cast<std::size_t>(j.a)];
        const Body* B = j.b >= 0 ? &bodies_[static_cast<std::size_t>(j.b)] : nullptr;
        if (j.kind == 1) {
            // the base: a slip base slides off at its shear; bolts fail at their moment; a
            // post in a sleeve tips out
            const Real horizontal = std::hypot(j.lambdaPos.x, j.lambdaPos.y) / h2;
            bool release = false;
            if (a.base.kind == BaseKind::SlipBase && a.base.release > 0 && horizontal > a.base.release * vary) release = true;
            if (a.base.kind == BaseKind::Bolted && a.base.boltMoment > 0 && j.lastTorque > a.base.boltMoment * vary) release = true;
            if (a.base.kind == BaseKind::Socket) {
                const Vec3 ax = rotate(A.q, {0, 0, 1});
                if (dm::acos(clamp(ax.z, -1, 1)) > a.base.socketAngle) release = true;
            }
            if ((a.base.kind == BaseKind::Footing || a.base.kind == BaseKind::Embedded) && a.base.footingYield > 0 &&
                j.soilPlastic > a.base.footingFail)
                release = true;
            if (release) {
                j.broken = true;
                changed = true;
                emit(EventType::BaseReleased, o, 0, j.rb,
                     a.base.kind == BaseKind::SlipBase || a.base.kind == BaseKind::Bolted ? 0.05 : 0.0);
                continue;
            }
        }
        // the hinge law: the moment the joint carried this substep (its multiplier: lambda / h^2)
        // against what the section (and, at the base, the footing) can carry after the plastic
        // rotation so far; the excess turns into plastic rotation, about the way it is bent
        const Vec3 dphi = jointError(A.q, B ? &B->q : nullptr, j.rest);
        const Real e = length(dphi);
        if (e < 1e-12 || j.compliance <= 0) continue;
        const Real torque = j.lastTorque;
        const Real capPost = a.mech.capacity(j.plastic) * vary;
        const Real capSoil = j.kind == 1 && a.base.footingYield > 0 ? a.base.footingYield * vary * wetSoil : 1e300;
        const Real cap = std::min(capPost, capSoil);
        if (torque <= cap) continue;
        const Real excess = std::min(e, (torque - cap) * j.compliance);
        // move the rest toward where the joint is now by the excess
        const Quat target = (B ? B->q : Quat{}) * conjugate(j.rest);
        const Quat shifted = axisAngle(dphi / e, -excess) * target;
        j.rest = conjugate(shifted) * (B ? B->q : Quat{});
        if (capSoil < capPost) j.soilPlastic += excess;
        else j.plastic += excess;
        if (j.plastic > a.mech.fractureRotation * vary) {
            j.broken = true;
            changed = true;
            const Vec3 at = A.x + rotate(A.q, j.ra);
            emit(EventType::PostBroken, o, k, at, std::max(0.0, at.z - o.groundZ));  // the section tore (at the ground too)
        }
    }
    if (!changed) return;
    // Which parts are still held to the ground through unbroken joints?
    for (int k = 0; k < o.bodyCount; ++k) bodies_[static_cast<std::size_t>(o.firstBody + k)].loose = true;
    for (bool grew = true; grew;) {
        grew = false;
        for (int k = 0; k < o.jointCount; ++k) {
            const Joint& j = joints_[static_cast<std::size_t>(o.firstJoint + k)];
            if (j.broken) continue;
            Body& A = bodies_[static_cast<std::size_t>(j.a)];
            if (j.b < 0) {
                if (A.loose) { A.loose = false; grew = true; }
                continue;
            }
            Body& B = bodies_[static_cast<std::size_t>(j.b)];
            if (A.loose != B.loose) {
                A.loose = B.loose = false;
                grew = true;
            }
        }
    }
}

// ---------------------------------------------------------------- contacts

constexpr Real kMaxDepenetration = 1.0;  // m/s
constexpr int kFrameSubsteps = 4;

void World::solveGround(int bi, Real h) {
    Body& b = bodies_[static_cast<std::size_t>(bi)];
    const Object& o = objects_[static_cast<std::size_t>(b.object)];
    const Real gz = o.groundZ;
    std::array<Vec3, 9> pts{};
    int np = 0;
    Real radius = 0;
    if (b.kind == 1) {
        for (int c = 0; c < 8; ++c)
            pts[static_cast<std::size_t>(np++)] =
                b.x + rotate(b.q, {(c & 1 ? 1 : -1) * b.half.x, (c & 2 ? 1 : -1) * b.half.y, (c & 4 ? 1 : -1) * b.half.z});
    } else {
        const Vec3 ax = rotate(b.q, {0, 0, b.half.z});
        pts[0] = b.x - ax;
        pts[1] = b.x;
        pts[2] = b.x + ax;
        np = 3;
        radius = b.half.x;
    }
    // A post's base is in the ground: the first segment's bottom end, held there, does not touch it.
    const int local = bi - o.firstBody;
    const bool baseSegment = b.kind == 0 && !b.loose && local < library.at(o.type).legs * o.segmentsPerLeg &&
                             local % o.segmentsPerLeg == 0;
    b.grounded = false;
    for (int k = 0; k < np; ++k) {
        if (baseSegment && k == 0) continue;
        const Vec3 p = pts[static_cast<std::size_t>(k)] - Vec3{0, 0, radius};
        Real depth = gz - p.z;
        if (depth <= 0) continue;
        b.grounded = true;
        const Vec3 r = p - b.x;
        // What this substep's motion pushed in is undone at once; a penetration that was already
        // there (a released base end, a part set down inside the kerb) is eased out at a walking
        // pace, or XPBD would turn it into a launch velocity of depth / h.
        const Vec3 rl0 = rotateInv(b.q, r);
        const Real before = gz - (b.xPrev + rotate(b.qPrev, rl0)).z;
        if (before > 0) depth -= before - std::min(before, kMaxDepenetration * h);
        if (depth <= 0) continue;
        Real lambda = 0;
        applyPositional(&b, nullptr, r, {}, {0, 0, depth}, 0, lambda);
        // friction: undo the contact point's slide this substep, up to mu x the push
        const Vec3 rl = rotateInv(b.q, p - b.x);
        const Vec3 now = b.x + rotate(b.q, rl), was = b.xPrev + rotate(b.qPrev, rl);
        Vec3 slide = now - was;
        slide.z = 0;
        const Real s = length(slide), lim = groundFriction * depth;
        if (s > 1e-12) {
            Real lt = 0;
            applyPositional(&b, nullptr, now - b.x, {}, slide * (-(s <= lim ? 1.0 : lim / s)), 0, lt);
        }
    }
}

void World::hostPose(int hi, Real tFrac, Vec3& c, Quat& q) const {
    const HostBody& hb = hosts_[static_cast<std::size_t>(hi)];
    const Real t = tFrac * hostStep;
    c = hb.centre + hb.velocity * t;
    q = integrate(hb.orientation, hb.angularVelocity, t);
}

void World::solveHosts(int bi, Real h, Real tFrac) {
    Body& b = bodies_[static_cast<std::size_t>(bi)];
    Object& o = objects_[static_cast<std::size_t>(b.object)];
    const Real h2 = h * h;
    for (int hi = 0; hi < static_cast<int>(hosts_.size()); ++hi) {
        const HostBody& hb = hosts_[static_cast<std::size_t>(hi)];
        Vec3 c;
        Quat q;
        hostPose(hi, tFrac, c, q);
        const Real reach = length(hb.half) + std::max(b.half.x, std::max(b.half.y, b.half.z)) + 0.1;
        if (length(b.x - c) > reach + b.half.z) continue;
        std::array<Vec3, 9> pts{};
        std::array<Real, 9> rad{};
        int np = 0;
        if (b.kind == 1) {
            for (int k = 0; k < 8; ++k)
                pts[static_cast<std::size_t>(np++)] =
                    b.x + rotate(b.q, {(k & 1 ? 1 : -1) * b.half.x, (k & 2 ? 1 : -1) * b.half.y, (k & 4 ? 1 : -1) * b.half.z});
            pts[static_cast<std::size_t>(np++)] = b.x;
        } else {
            const Vec3 ax = rotate(b.q, {0, 0, b.half.z});
            Vec3 p, n;
            Real t = 0;
            segmentBox(b.x - ax, b.x + ax, c, q, hb.half, p, n, t, hb.velocity - b.v);
            pts[0] = p;
            rad[0] = b.half.x;
            np = 1;
        }
        for (int k = 0; k < np; ++k) {
            Vec3 n;
            const Real d = boxDistance(pts[static_cast<std::size_t>(k)], c, q, hb.half, n, hb.velocity - b.v);
            const Real depth = rad[static_cast<std::size_t>(k)] - d;
            if (depth <= 0) continue;
            const Vec3 p = pts[static_cast<std::size_t>(k)] - n * rad[static_cast<std::size_t>(k)];
            Real lambda = 0;
            applyPositional(&b, nullptr, p - b.x, {}, n * depth, 0, lambda);
            // the host box takes the opposite push (an impulse lambda / h on it)
            hostImpulse_[static_cast<std::size_t>(hi)] -= n * (lambda / h);
            o.contactForce = std::max(o.contactForce, lambda / h2);
            // friction against the box's moving surface
            const Vec3 rl = rotateInv(b.q, p - b.x);
            const Vec3 now = b.x + rotate(b.q, rl), was = b.xPrev + rotate(b.qPrev, rl);
            const Vec3 hostMove = (hb.velocity + cross(hb.angularVelocity, p - c)) * h;
            Vec3 slide = (now - was) - hostMove;
            slide -= n * dot(slide, n);
            const Real s = length(slide), lim = hostFriction * depth;
            if (s > 1e-12) {
                Real lt = 0;
                applyPositional(&b, nullptr, now - b.x, {}, slide * (-(s <= lim ? 1.0 : lim / s)), 0, lt);
            }
            if (!o.hitReported) {
                o.hitReported = true;
                emit(EventType::ObjectHit, o, b.kind == 1 ? b.panel : -1, p, length(hb.velocity), hb.id);
            }
            if (b.loose && !b.struck && lambda / h > 2.0) {
                b.struck = true;
                // a loose part (a sign plate, a broken post) striking the car: the windscreen
                // and the roof are the host's to break (Glass Dynamics)
                emit(EventType::DebrisHitBody, o, b.kind == 1 ? b.panel : -1, p, lambda / h, hb.id);
            }
        }
    }
}

// ---------------------------------------------------------------- step

void World::wake(int oi) {
    Object& o = objects_[static_cast<std::size_t>(oi)];
    if (o.awake || !o.active) return;
    o.awake = true;
    o.still = 0;
    awake_.push_back(oi);
}

void World::applyImpact(std::int32_t id, const Vec3& point, const Vec3& normal, Real impulse) {
    const int oi = objectIndex(id);
    if (oi < 0 || impulse <= 0) return;
    const Object& o = objects_[static_cast<std::size_t>(oi)];
    if (!o.active) return;
    // the part nearest the point takes it
    int best = -1;
    Real bestD = 1e300;
    for (int k = 0; k < o.bodyCount; ++k) {
        const ObstacleHit h = hitBody(o.firstBody + k, point, 1.0);
        const Real d = h.hit ? -h.depth : 1e299;
        if (d < bestD) {
            bestD = d;
            best = o.firstBody + k;
        }
    }
    if (best < 0) return;
    if (impacts_.size() < 256) impacts_.push_back(PendingImpact{best, point, normal * -impulse});
}

int World::step(Real dt) {
    const auto t0 = std::chrono::steady_clock::now();
    hostStep = dt;
    for (auto& im : hostImpulse_) im = {};
    // The vehicle's impulses: wake the objects and give the parts their kick.
    for (const PendingImpact& pi : impacts_) {
        Body& b = bodies_[static_cast<std::size_t>(pi.body)];
        Object& o = objects_[static_cast<std::size_t>(b.object)];
        wake(b.object);
        b.v += pi.impulse * b.invMass;
        b.w += invInertiaWorld(b, cross(pi.point - b.x, pi.impulse));
        if (impactSpeedCap > 0) {   // Patch 2026-11 (see world.hpp)
            const Real sv = length(b.v);
            if (sv > impactSpeedCap) b.v = b.v * (impactSpeedCap / sv);
            const Real wCap = impactSpeedCap / std::max(b.half.z, Real(0.15));
            const Real sw = length(b.w);
            if (sw > wCap) b.w = b.w * (wCap / sw);
        }
        o.contactForce = std::max(o.contactForce, length(pi.impulse) / dt);
        if (!o.hitReported) {
            o.hitReported = true;
            emit(EventType::ObjectHit, o, b.kind == 1 ? b.panel : -1, pi.point, length(pi.impulse) * b.invMass, 0);
        } else if (b.loose && !b.struck && length(pi.impulse) > 2.0) {
            b.struck = true;
            // a loose part (a sign plate, a broken post) striking the car (the windscreen and the
            // roof are the host's to break: Glass Dynamics)
            emit(EventType::DebrisHitBody, o, b.kind == 1 ? b.panel : -1, pi.point, length(pi.impulse), 0);
        }
    }
    impacts_.clear();
    // Wake whatever a host body is reaching.
    for (const HostBody& hb : hosts_) {
        const Real r = length(hb.half) + length(hb.velocity) * dt + 1.0;
        const auto cx0 = static_cast<std::int64_t>(std::floor((hb.centre.x - r) / 8.0));
        const auto cx1 = static_cast<std::int64_t>(std::floor((hb.centre.x + r) / 8.0));
        const auto cy0 = static_cast<std::int64_t>(std::floor((hb.centre.y - r) / 8.0));
        const auto cy1 = static_cast<std::int64_t>(std::floor((hb.centre.y + r) / 8.0));
        for (auto cx = cx0 - 1; cx <= cx1 + 1; ++cx)
            for (auto cy = cy0 - 1; cy <= cy1 + 1; ++cy) {
                const auto it = grid_.find((cx << 32) ^ (cy & 0xffffffff));
                if (it == grid_.end()) continue;
                for (int oi : it->second) {
                    const Object& o = objects_[static_cast<std::size_t>(oi)];
                    const Real reach = library.at(o.type).height + o.cellR + r;
                    if (std::hypot(o.x - hb.centre.x, o.y - hb.centre.y) < reach) wake(oi);
                }
            }
    }
    // a stable order whatever the order things woke in
    std::sort(awake_.begin(), awake_.end());
    if (awake_.empty()) {
        stats_.awake = 0;
        stats_.lastStepMs = 0;
        return 0;
    }
    for (int oi : awake_) {
        Object& o = objects_[static_cast<std::size_t>(oi)];
        o.lastContactForce = o.contactForce;
    }
    // Closed frames (a rack's two legs and bar, both feet in the ground) are a loop that a
    // substep's sequential pass never quite closes; once their hinges have yielded it rings and
    // ratchets. They are light, so they take kFrameSubsteps finer substeps of their own.
    plain_.clear();
    frames_.clear();
    for (int oi : awake_) (library.at(objects_[static_cast<std::size_t>(oi)].type).legs > 1 ? frames_ : plain_).push_back(oi);
    const int n = std::max(1, substepsPerStep);
    const Real h = dt / n;
    for (int s = 0; s < n; ++s) {
        if (!plain_.empty()) substep(h, static_cast<Real>(s + 1) / n, plain_);
        for (int m = 0; m < kFrameSubsteps && !frames_.empty(); ++m)
            substep(h / kFrameSubsteps, (s + static_cast<Real>(m + 1) / kFrameSubsteps) / n, frames_);
        ++substeps_;
    }
    // states, events, sleep
    int awakeCount = 0;
    for (std::size_t k = 0; k < awake_.size();) {
        const int oi = awake_[k];
        Object& o = objects_[static_cast<std::size_t>(oi)];
        updateStates(o, oi);
        // Asleep when it is still, or when it only trembles in place: a bent post lying on the
        // pavement, or a closed frame left with residual stress (the few iterations a substep
        // gets never quite settle it), keeps a fast jitter of up to a few m/s but goes nowhere,
        // so what counts is how far each part has drifted since the object was last in motion.
        Real vmax = 0, drift = 0;
        for (int b = 0; b < o.bodyCount; ++b) {
            const Body& bd = bodies_[static_cast<std::size_t>(o.firstBody + b)];
            vmax = std::max(vmax, length(bd.v) + 0.1 * length(bd.w));
            const Quat dq = conjugate(bd.settleQ) * bd.q;
            drift = std::max(drift, length(bd.x - bd.settleX) + 0.2 * length(Vec3{dq.x, dq.y, dq.z}));
        }
        const bool quiet = o.contactForce == 0 &&
                           (vmax < sleepSpeed || (vmax < 200 * sleepSpeed && drift < sleepSpeed * sleepTime));
        if (quiet) o.still += dt;
        else {
            o.still = 0;
            for (int b = 0; b < o.bodyCount; ++b) {
                Body& bd = bodies_[static_cast<std::size_t>(o.firstBody + b)];
                bd.settleX = bd.x, bd.settleQ = bd.q;
            }
        }
        o.contactForce = 0;
        if (o.still > sleepTime) {
            o.awake = false;
            for (int b = 0; b < o.bodyCount; ++b) {
                Body& bd = bodies_[static_cast<std::size_t>(o.firstBody + b)];
                bd.v = bd.w = {};
            }
            awake_.erase(awake_.begin() + static_cast<std::ptrdiff_t>(k));
            continue;
        }
        ++awakeCount;
        ++k;
    }
    const auto t1 = std::chrono::steady_clock::now();
    stats_.awake = awakeCount;
    stats_.lastStepMs = std::chrono::duration<double, std::milli>(t1 - t0).count();
    stats_.maxStepMs = std::max(stats_.maxStepMs, stats_.lastStepMs);
    stats_.steps += 1;
    return n;
}

void World::substep(Real h, Real tFrac, const std::vector<int>& set) {
    const Vec3 g{0, 0, -kGravity};
    // ---- predict ----
    for (int oi : set) {
        const Object& o = objects_[static_cast<std::size_t>(oi)];
        for (int k = 0; k < o.bodyCount; ++k) {
            Body& b = bodies_[static_cast<std::size_t>(o.firstBody + k)];
            b.xPrev = b.x;
            b.qPrev = b.q;
            Vec3 f = g;
            Vec3 torque{};
            if (b.kind == 1 && b.loose) {
                // A flat plate in still air: the normal force of a plate at incidence alpha,
                // C_N ~ 1.2 sin(alpha), at a centre of pressure that moves from the quarter
                // chord (edge on) to the centre (face on): plates flutter and tumble. Plus
                // the drag of the plate rotating (estimated).
                const Real speed = length(b.v);
                const Vec3 nrm = rotate(b.q, {0, 1, 0});
                if (speed > 0.05) {
                    const Vec3 u = b.v / speed;
                    const Real sa = dot(u, nrm);  // sin of the incidence
                    const Real area = 4.0 * b.half.x * b.half.z;
                    const Real fn = 0.5 * airDensity * speed * speed * area * 1.2 * sa;
                    const Vec3 force = nrm * (-fn);
                    f += force * b.invMass;
                    Vec3 inPlane = u - nrm * sa;
                    const Real ip = length(inPlane);
                    if (ip > 1e-6) {
                        inPlane = inPlane / ip;
                        const Real chord = 2.0 * std::max(b.half.x, b.half.z);
                        const Real xcp = 0.25 * chord * (1.0 - std::abs(sa));
                        torque += cross(inPlane * xcp, force);
                    }
                }
                const Real span = 2.0 * std::max(b.half.x, b.half.z);
                torque -= b.w * (0.05 * airDensity * span * span * span * span * span * length(b.w));
            }
            b.v += f * h;
            b.v *= std::max(0.0, 1.0 - 0.02 * h);
            b.x += b.v * h;
            b.w += invInertiaWorld(b, torque) * h;
            b.w *= std::max(0.0, 1.0 - 0.05 * h);
            b.q = integrate(b.q, b.w, h);
        }
        for (int k = 0; k < o.jointCount; ++k) {
            Joint& j = joints_[static_cast<std::size_t>(o.firstJoint + k)];
            j.lambdaPos = j.lambdaAng = Vec3{};
        }
    }
    // ---- solve: joints, then the ground and the host bodies, then the joints again (so the
    // structure carries what the contacts put on it before the hinges are judged) ----
    auto joints = [&] {
        for (int oi : set) {
            const Object& o = objects_[static_cast<std::size_t>(oi)];
            for (int k = 0; k < o.jointCount; ++k) solveJoint(joints_[static_cast<std::size_t>(o.firstJoint + k)], h);
        }
    };
    joints();
    for (int oi : set) {
        const Object& o = objects_[static_cast<std::size_t>(oi)];
        for (int k = 0; k < o.bodyCount; ++k) {
            solveHosts(o.firstBody + k, h, tFrac);
            solveGround(o.firstBody + k, h);
        }
    }
    joints();
    // ---- velocities ----
    for (int oi : set) {
        Object& o = objects_[static_cast<std::size_t>(oi)];
        for (int k = 0; k < o.bodyCount; ++k) {
            Body& b = bodies_[static_cast<std::size_t>(o.firstBody + k)];
            b.v = (b.x - b.xPrev) / h;
            b.w = rotationVector(b.q * conjugate(b.qPrev)) / h;
            if (b.grounded) {  // rolling and scraping on the ground
                b.v *= std::max(0.0, 1.0 - 4.0 * h);
                b.w *= std::max(0.0, 1.0 - 6.0 * h);
            }
        }
        // joint damping (Mueller et al. 2020, 3.6): the relative spin across each joint decays,
        // a stand-in for the steel's own damping; a part turning as one piece is untouched
        const Real damp = std::min(1.0, jointDamping * h);
        for (int k = 0; k < o.jointCount && damp > 0; ++k) {
            const Joint& j = joints_[static_cast<std::size_t>(o.firstJoint + k)];
            if (j.broken) continue;
            Body& A = bodies_[static_cast<std::size_t>(j.a)];
            if (j.b < 0) {  // the base: spin relative to the ground
                A.w -= A.w * damp;
                continue;
            }
            Body& B = bodies_[static_cast<std::size_t>(j.b)];
            const Real wa = std::max(A.invInertia.x, std::max(A.invInertia.y, A.invInertia.z));
            const Real wb = std::max(B.invInertia.x, std::max(B.invInertia.y, B.invInertia.z));
            const Vec3 dw = (B.w - A.w) * damp;
            A.w += dw * (wa / (wa + wb));
            B.w -= dw * (wb / (wa + wb));
        }
        plasticity(o, h);
    }
}

void World::updateStates(Object& o, int oi) {
    const Archetype& a = library.at(o.type);
    (void)oi;
    const Real plastic = objectMaxPlastic(oi);
    const Vec3 at{o.x, o.y, o.groundZ};
    if (!o.bentReported && plastic > 0.03) {
        o.bentReported = true;
        if (o.state == ObjectState::Intact) o.state = ObjectState::Bent;
        emit(EventType::PostBent, o, -1, at, plastic);
    }
    bool broken = false;
    for (int k = 0; k < o.jointCount; ++k) {
        const Joint& j = joints_[static_cast<std::size_t>(o.firstJoint + k)];
        if (j.broken && j.kind != 2) broken = true;
    }
    if (broken) {
        o.state = ObjectState::Broken;
        // The stub: what is still anchored, at its highest (a slip base or a bolted plate leaves
        // its own hardware, about 50 mm).
        Real top = -1;
        for (int k = 0; k < o.bodyCount; ++k) {
            const Body& b = bodies_[static_cast<std::size_t>(o.firstBody + k)];
            if (b.loose || b.kind == 1) continue;
            const Vec3 ax = rotate(b.q, {0, 0, b.half.z});
            top = std::max(top, std::max(b.x.z + ax.z, b.x.z - ax.z) - o.groundZ);
        }
        const bool hardware = a.base.kind == BaseKind::SlipBase || a.base.kind == BaseKind::Bolted;
        o.stub = top >= 0 ? top : (hardware ? 0.05 : 0.0);
    }
    if (!o.downReported && objectTopHeight(oi) < 0.5 * a.height) {
        o.downReported = true;
        if (o.state != ObjectState::Broken) o.state = ObjectState::Down;
        emit(EventType::PostDown, o, -1, at, objectTopHeight(oi));
    }
    for (int k = 0; k < o.bodyCount; ++k) {
        const Body& b = bodies_[static_cast<std::size_t>(o.firstBody + k)];
        if (b.kind != 1 || b.panel < 0 || b.panel > 3 || !b.loose || !b.grounded) continue;
        bool& landed = o.panelLanded[static_cast<std::size_t>(b.panel)];
        if (!landed) {
            landed = true;
            emit(EventType::PanelLanded, o, b.panel, b.x, length(b.v));
        }
    }
    // how far the object's parts reach from its base (for the broadphase)
    Real reach = 0;
    for (int k = 0; k < o.bodyCount; ++k) {
        const Body& b = bodies_[static_cast<std::size_t>(o.firstBody + k)];
        reach = std::max(reach, std::hypot(b.x.x - o.x, b.x.y - o.y) + length(b.half));
    }
    o.cellR = std::max(o.cellR, reach);
}

// ---------------------------------------------------------------- the vehicle's query

Real World::bodyEffectiveMass(const Body& b, const Vec3& p, const Vec3& n) const {
    const Real w = genInvMass(b, p - b.x, n);
    return w > 0 ? 1.0 / w : 0.0;
}

// The most the object can resist at p without its structure giving way: the weakest hinge
// between p and the ground, carrying the force on its arm (the height above it).
static Real kMinArm = 0.30;
Real World::staticResistance(const Object& o, const Vec3& p) const {
    const Archetype& a = library.at(o.type);
    const Real vary = 0.9 + 0.2 * static_cast<Real>(o.variation % 1000u) / 999.0;
    Real best = 1e300;
    bool held = false;
    for (int k = 0; k < o.jointCount; ++k) {
        const Joint& j = joints_[static_cast<std::size_t>(o.firstJoint + k)];
        if (j.broken || j.kind == 2) continue;
        const Body& A = bodies_[static_cast<std::size_t>(j.a)];
        const Vec3 at = A.x + rotate(A.q, j.ra);
        const Real arm = p.z - at.z;
        if (arm <= 0.02) continue;
        Real cap = a.mech.capacity(j.plastic) * vary;
        if (j.kind == 1 && a.base.footingYield > 0) cap = std::min(cap, a.base.footingYield * vary);
        if (j.kind == 1 && a.base.kind == BaseKind::SlipBase && a.base.release > 0) cap = std::min(cap, a.base.release * vary * arm);
        if (j.kind == 1 && a.base.kind == BaseKind::Bolted && a.base.boltMoment > 0) cap = std::min(cap, a.base.boltMoment * vary);
        best = std::min(best, cap / std::max(arm, kMinArm));   // Patch 2026-11: a part folded flat near the ground has almost no lever; without a floor the force went to cap / 2 cm
        if (j.kind == 1) held = true;
    }
    return held ? best : 0.0;
}

ObstacleHit World::hitBody(int bi, const Vec3& p, Real margin, const Vec3& approach) const {
    const Body& b = bodies_[static_cast<std::size_t>(bi)];
    ObstacleHit r;
    if (b.kind == 1) {
        Vec3 n;
        const Real d = boxDistance(p, b.x, b.q, b.half, n);
        if (d < margin) {
            r.hit = true;
            r.depth = -d;
            r.normal = n;
        }
        return r;
    }
    const Vec3 axis = rotate(b.q, {0, 0, 1});
    const Vec3 a0 = b.x - axis * b.half.z;
    const Real t = clamp(dot(p - a0, axis), 0, 2 * b.half.z);
    const Vec3 c = a0 + axis * t;
    const Vec3 off = p - c;
    // The car's approach, across the post: once the point is past the axis along it (a thin
    // post between two steps of a fast car), the post still pushes it back where it came from.
    Vec3 ap = approach - axis * dot(approach, axis);
    const Real apl = length(ap);
    if (apl > 0.1) {
        ap = ap / apl;
        const Real along = dot(off, ap);
        const Vec3 side = off - ap * along - axis * dot(off, axis);
        if (along > 0 && length(side) < b.half.x) {
            const Real depth = b.half.x + along;
            if (depth > -margin && along < b.half.x + 2.0 * margin + 0.1) {
                r.hit = true;
                r.depth = depth;
                r.normal = ap * -1.0;
            }
            return r;
        }
    }
    const Real dist = length(off);
    if (dist - b.half.x < margin && dist > 1e-9) {
        r.hit = true;
        r.depth = b.half.x - dist;
        r.normal = off / dist;
    }
    return r;
}

bool World::anyNear(const Vec3& c, Real radius) const {
    const auto cx = static_cast<std::int64_t>(std::floor(c.x / 8.0)), cy = static_cast<std::int64_t>(std::floor(c.y / 8.0));
    for (auto x = cx - 1; x <= cx + 1; ++x)
        for (auto y = cy - 1; y <= cy + 1; ++y) {
            const auto it = grid_.find((x << 32) ^ (y & 0xffffffff));
            if (it == grid_.end()) continue;
            for (int oi : it->second) {
                const Object& o = objects_[static_cast<std::size_t>(oi)];
                if (!o.active) continue;
                const Real reach = std::max(library.at(o.type).height, o.cellR) + 0.5;
                if (std::hypot(o.x - c.x, o.y - c.y) < reach + radius && c.z - radius < o.groundZ + reach) return true;
            }
        }
    return false;
}

ObstacleHit World::obstaclePoint(const Vec3& p, Real margin, const Vec3& pointVelocity, Real stepTime) const {
    ObstacleHit best;
    best.depth = -1e9;
    int bestBody = -1;
    const auto cx = static_cast<std::int64_t>(std::floor(p.x / 8.0)), cy = static_cast<std::int64_t>(std::floor(p.y / 8.0));
    for (auto x = cx - 1; x <= cx + 1; ++x)
        for (auto y = cy - 1; y <= cy + 1; ++y) {
            const auto it = grid_.find((x << 32) ^ (y & 0xffffffff));
            if (it == grid_.end()) continue;
            for (int oi : it->second) {
                const Object& o = objects_[static_cast<std::size_t>(oi)];
                if (!o.active) continue;
                for (int k = 0; k < o.bodyCount; ++k) {
                    const Body& bd = bodies_[static_cast<std::size_t>(o.firstBody + k)];
                    const Vec3 rel = pointVelocity - bd.v;
                    const Real rl = length(rel);
                    const ObstacleHit h = hitBody(o.firstBody + k, p, margin, rl > 0.1 ? rel / rl : Vec3{});
                    if (h.hit && h.depth > best.depth) {
                        best = h;
                        best.id = o.id;
                        bestBody = o.firstBody + k;
                    }
                }
            }
        }
    if (bestBody < 0) return ObstacleHit{};
    const Body& b = bodies_[static_cast<std::size_t>(bestBody)];
    const Object& o = objects_[static_cast<std::size_t>(b.object)];
    // What it can push back with: the structure's resistance (or the clamps, for a sign plate),
    // plus what it takes to set this part moving with the car in one step; never less than
    // what it pushed with last step.
    Real structure = b.loose ? 0.0 : staticResistance(o, p);
    if (b.kind == 1 && !b.loose) structure = library.at(o.type).panels[static_cast<std::size_t>(b.panel)].clamp;
    if (b.kind != 1) {   // Patch 2026-11: a segment lying nearly flat is being driven over, not pushed against: its hinge moments no longer resist the car's forward motion
        const Real up = std::abs(rotate(b.q, Vec3{0, 0, 1}).z);
        const Real t = std::clamp((up - 0.15) / 0.45, 0.0, 1.0);
        structure *= t * t * (3 - 2 * t);
    }
    const Vec3 r = p - b.x;
    const Vec3 partVel = b.v + cross(b.w, r);
    const Real closing = std::max(0.0, dot(pointVelocity - partVel, best.normal * -1.0));
    const Real inertia = stepTime > 0 ? bodyEffectiveMass(b, p, best.normal) * closing / stepTime : 0.0;
    best.yieldForce = std::max(structure + inertia, 50.0)   /* Patch 2026-11: no 1.2 x lastContactForce term (it let the force climb 20 % a step for as long as the contact lasted) */;
    return best;
}

ObstacleHit World::obstacleEdge(const Vec3& a, const Vec3& b, Real margin, const Vec3& pointVelocity, Real stepTime,
                                Real& tOut) const {
    ObstacleHit best;
    best.depth = -1e9;
    tOut = 0;
    const Vec3 mid = (a + b) * 0.5;
    const Real half = 0.5 * length(b - a);
    Real bestT = 0;
    int bestBody = -1;
    const auto cx = static_cast<std::int64_t>(std::floor(mid.x / 8.0)), cy = static_cast<std::int64_t>(std::floor(mid.y / 8.0));
    for (auto x = cx - 1; x <= cx + 1; ++x)
        for (auto y = cy - 1; y <= cy + 1; ++y) {
            const auto it = grid_.find((x << 32) ^ (y & 0xffffffff));
            if (it == grid_.end()) continue;
            for (int oi : it->second) {
                const Object& o = objects_[static_cast<std::size_t>(oi)];
                if (!o.active) continue;
                if (std::hypot(o.x - mid.x, o.y - mid.y) > std::max(library.at(o.type).height, o.cellR) + half + 1.0) continue;
                for (int k = 0; k < o.bodyCount; ++k) {
                    const Body& bd = bodies_[static_cast<std::size_t>(o.firstBody + k)];
                    if (bd.kind == 1) continue;  // plates: the node contacts have them
                    const Vec3 ax = rotate(bd.q, {0, 0, bd.half.z});
                    Real s = 0, t = 0;
                    segmentSegment(a, b, bd.x - ax, bd.x + ax, s, t);
                    const Vec3 pe = a + (b - a) * s;
                    const Vec3 rel = pointVelocity - bd.v;
                    const Real rl = length(rel);
                    const ObstacleHit hb = hitBody(o.firstBody + k, pe, margin, rl > 0.1 ? rel / rl : Vec3{});
                    if (!hb.hit) continue;
                    const Real depth = hb.depth;
                    if (depth > best.depth) {
                        best.hit = true;
                        best.depth = depth;
                        best.normal = hb.normal;
                        best.id = o.id;
                        bestT = s;
                        bestBody = o.firstBody + k;
                    }
                }
            }
        }
    if (bestBody < 0) return ObstacleHit{};
    tOut = bestT;
    const Body& bd = bodies_[static_cast<std::size_t>(bestBody)];
    const Object& o = objects_[static_cast<std::size_t>(bd.object)];
    const Vec3 p = a + (b - a) * bestT;
    Real structure = bd.loose ? 0.0 : staticResistance(o, p);
    if (bd.kind != 1) {   // Patch 2026-11: as in obstaclePoint, a segment lying nearly flat does not hold the car back
        const Real up = std::abs(rotate(bd.q, Vec3{0, 0, 1}).z);
        const Real t = std::clamp((up - 0.15) / 0.45, 0.0, 1.0);
        structure *= t * t * (3 - 2 * t);
    }
    const Vec3 partVel = bd.v + cross(bd.w, p - bd.x);
    const Real closing = std::max(0.0, dot(pointVelocity - partVel, best.normal * -1.0));
    const Real inertia = stepTime > 0 ? bodyEffectiveMass(bd, p, best.normal) * closing / stepTime : 0.0;
    best.yieldForce = std::max(structure + inertia, 50.0)   /* Patch 2026-11: no 1.2 x lastContactForce term (it let the force climb 20 % a step for as long as the contact lasted) */;
    return best;
}

}  // namespace sd
