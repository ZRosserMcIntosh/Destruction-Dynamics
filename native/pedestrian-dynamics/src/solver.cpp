// Pedestrian Dynamics — physically simulated people.
// SPDX-License-Identifier: MIT
//
// The solver: XPBD on rigid bodies (Mueller, Macklin, Chentanez, Jeschke, Kim 2020, "Detailed
// Rigid Body Simulation with Extended Position Based Dynamics"), with the damped compliant
// constraint of Macklin, Mueller, Chentanez 2016 ("XPBD") for contacts.
#include <algorithm>
#include <chrono>
#include <cmath>

#include "ped/limits.hpp"
#include "ped/rot.hpp"
#include "ped/world.hpp"

namespace ped {

namespace {

// Signed distance from a point to an axis-aligned box about the origin (negative inside) and the
// outward normal there, in the box's frame. `approach` is the box's velocity relative to the
// point: a point inside a moving box leaves through the face it came in by, not the nearest one
// (a fast bonnet never flips a leg out of its back), as in Street Dynamics, but only a face
// crossed in the last moments: a part riding along with the box closes on its true face at
// almost nothing, and a slow drift toward a far face must not win.
constexpr Real kMaxEntryTime = 0.05;  // s

Real localBoxDistance(const Vec3& l, const Vec3& half, Vec3& normal, const Vec3& approach) {
    const Vec3 d{std::abs(l.x) - half.x, std::abs(l.y) - half.y, std::abs(l.z) - half.z};
    if (d.x > 0 || d.y > 0 || d.z > 0) {
        const Vec3 o{maxr(d.x, 0) * (l.x < 0 ? -1 : 1), maxr(d.y, 0) * (l.y < 0 ? -1 : 1), maxr(d.z, 0) * (l.z < 0 ? -1 : 1)};
        const Real len = length(o);
        normal = o / len;
        return len;
    }
    const Real vl[3] = {approach.x, approach.y, approach.z}, ll[3] = {l.x, l.y, l.z}, dl[3] = {d.x, d.y, d.z};
    int best = -1;
    Real bestT = 1e300;
    for (int ax = 0; ax < 3; ++ax) {
        const Real s = ll[ax] < 0 ? -1.0 : 1.0;
        const Real closing = vl[ax] * s;
        if (closing <= 1e-3) continue;
        const Real tEntry = -dl[ax] / closing;
        if (tEntry > kMaxEntryTime) continue;
        if (tEntry < bestT) bestT = tEntry, best = ax;
    }
    if (best < 0) best = (d.x >= d.y && d.x >= d.z) ? 0 : (d.y >= d.z ? 1 : 2);
    normal = {};
    (best == 0 ? normal.x : best == 1 ? normal.y : normal.z) = ll[best] < 0 ? -1.0 : 1.0;
    return dl[best];
}

// The cheap form for the search: the signed distance only.
Real localBoxSdf(const Vec3& l, const Vec3& half) {
    const Real dx = std::abs(l.x) - half.x, dy = std::abs(l.y) - half.y, dz = std::abs(l.z) - half.z;
    const Real ox = maxr(dx, 0), oy = maxr(dy, 0), oz = maxr(dz, 0);
    const Real outside = ox * ox + oy * oy + oz * oz;
    return outside > 0 ? std::sqrt(outside) : maxr(dx, maxr(dy, dz));
}

// The point of segment a-b (box frame) nearest the box: the signed distance is convex along a
// line, so a ternary search finds its minimum.
Real localSegmentBox(const Vec3& a, const Vec3& b, const Vec3& half, Vec3& point, Vec3& normal, const Vec3& approach) {
    Real lo = 0, hi = 1;
    const Vec3 ab = b - a;
    for (int it = 0; it < 22; ++it) {
        const Real m1 = lo + (hi - lo) / 3, m2 = hi - (hi - lo) / 3;
        if (localBoxSdf(a + ab * m1, half) <= localBoxSdf(a + ab * m2, half)) hi = m2;
        else lo = m1;
    }
    point = a + ab * (0.5 * (lo + hi));
    return localBoxDistance(point, half, normal, approach);
}

void closestSegments(const Vec3& p1, const Vec3& q1, const Vec3& p2, const Vec3& q2, Real& s, Real& t) {
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

// The damping term of a contact (XPBD 2016's gamma). A damper in parallel with the spring
// (Kelvin-Voigt) pushes back c v the instant two bodies touch: a head reaching the asphalt at
// 5 m/s would take a 400 g jolt in the first millisecond. Hunt and Crossley (1975) make the
// damping grow with the penetration, so the force starts at zero; here it ramps in over the
// penetration an undamped impact at this closing speed would reach.
Real contactGamma(Real alpha, Real w, Real zeta, Real depth, Real closingSpeed, Real h) {
    const Real k = 1.0 / alpha;
    const Real beta = 2.0 * zeta * std::sqrt(k / w);
    const Real reach = std::max(closingSpeed, 0.2) * std::sqrt(1.0 / (w * k));
    return alpha * beta * std::min(1.0, depth / reach) / h;
}

bool upperSurface(Region r) {
    return r == Region::Bonnet || r == Region::Cowl || r == Region::Windscreen || r == Region::APillar ||
           r == Region::RoofHeader || r == Region::Roof;
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

// A contact: moves a by up to `corr` relative to b (ra, rb: offsets from the centres, world).
// alpha is the compliance over h^2; gamma the damping term (XPBD 2016, eq. 26) acting on
// `approach`, how far the contact closed this substep. Contacts only push. Returns the
// multiplier (force = lambda / h^2).
Real World::applyPositional(Body* a, Body* b, const Vec3& ra, const Vec3& rb, const Vec3& corr, Real alpha, Real gamma,
                            Real approach) {
    const Real c = length(corr);
    if (c < 1e-12) return 0;
    const Vec3 n = corr / c;
    const Real wa = a ? genInvMass(*a, ra, n) : 0, wb = b ? genInvMass(*b, rb, n) : 0;
    const Real dl = std::max(0.0, (c + gamma * approach) / ((1 + gamma) * (wa + wb) + alpha));
    if (dl <= 0) return 0;
    const Vec3 p = n * dl;
    if (a) {
        a->x += p * a->invMass;
        a->q = addRotation(a->q, invInertiaWorld(*a, cross(ra, p)));
    }
    if (b) {
        b->x -= p * b->invMass;
        b->q = addRotation(b->q, invInertiaWorld(*b, cross(rb, p)) * -1.0);
    }
    return dl;
}

// A rigid joint's positional part, with the multiplier kept as a vector (the error can change
// direction between passes; see Street Dynamics).
void World::applyPositionalVec(Body& a, Body* b, const Vec3& ra, const Vec3& rb, const Vec3& corr, Vec3& lambda) {
    const Real c = length(corr);
    if (c < 1e-12) return;
    const Vec3 n = corr / c;
    const Real wa = genInvMass(a, ra, n), wb = b ? genInvMass(*b, rb, n) : 0;
    const Real dl = c / (wa + wb);
    lambda += n * dl;
    const Vec3 p = n * dl;
    a.x += p * a.invMass;
    a.q = addRotation(a.q, invInertiaWorld(a, cross(ra, p)));
    if (b) {
        b->x -= p * b->invMass;
        b->q = addRotation(b->q, invInertiaWorld(*b, cross(rb, p)) * -1.0);
    }
}

// Rotates a by +dphi relative to b, compliance alpha (over h^2), multiplier capped at maxLambda
// (a muscle's torque times h^2; 0: no cap).
void World::applyAngular(Body& a, Body* b, const Vec3& dphi, Real alpha, Vec3& lambda, Real maxLambda) {
    const Real th = length(dphi);
    if (th < 1e-12) return;
    const Vec3 n = dphi / th;
    const Real wa = dot(n, invInertiaWorld(a, n)), wb = b ? dot(n, invInertiaWorld(*b, n)) : 0;
    const Real dl = (th - alpha * dot(lambda, n)) / (wa + wb + alpha);
    Vec3 next = lambda + n * dl;
    if (maxLambda > 0) {
        const Real m = length(next);
        if (m > maxLambda) next = next * (maxLambda / m);
    }
    const Vec3 p = next - lambda;
    lambda = next;
    a.q = addRotation(a.q, invInertiaWorld(a, p));
    if (b) b->q = addRotation(b->q, invInertiaWorld(*b, p) * -1.0);
}

// ---------------------------------------------------------------- the substep

void World::predict(Pedestrian& p, Real h) {
    const Vec3 g{0, 0, -kGravity};
    for (int s = 0; s < p.count; ++s) {
        Body& b = p.body[static_cast<std::size_t>(s)];
        b.xPrev = b.x;
        b.qPrev = b.q;
        b.v += g * h;
        b.x += b.v * h;
        b.q = integrate(b.q, b.w, h);
        b.wasGrounded = b.grounded;
        b.wasVehicleRegion = b.vehicleRegion;
        b.grounded = false;
        b.vehicleRegion = -1;
    }
    for (int s = 1; s < p.count; ++s) {
        Joint& j = p.joint[static_cast<std::size_t>(s)];
        j.lambdaPos = j.lambdaLimit = j.lambdaDrive = {};
    }
    p.assistLambda[0] = p.assistLambda[1] = {};
}

void World::solveJoints(Pedestrian& p, Real h, bool drives) {
    const Real h2 = h * h;
    const Real s = p.strength;
    for (int k = 1; k < p.count; ++k) {
        Joint& j = p.joint[static_cast<std::size_t>(k)];
        Body& P = p.body[static_cast<std::size_t>(j.parent)];
        Body& C = p.body[static_cast<std::size_t>(j.child)];
        // the ball
        const Vec3 rp = rotate(P.q, j.rp), rc = rotate(C.q, j.rc);
        applyPositionalVec(C, &P, rc, rp, (P.x + rp) - (C.x + rc), j.lambdaPos);
        // the range: four quarter-ellipses of swing about the neutral pose, and a twist range
        const Quat r = conjugate(P.q) * C.q;
        Quat swing;
        Real tw = swingTwist(r, j.t, swing);
        const Vec3 phi = rotationVector(swing);
        Real fa = dot(phi, j.a), fb = dot(phi, j.b);
        if (clampToRange(fa, fb, tw, j.flexMin, j.flexMax, j.abdMin, j.abdMax, j.twistMin, j.twistMax)) {
            const Quat allowed = fromRotationVector(j.a * fa + j.b * fb) * axisAngle(j.t, tw);
            const Vec3 dphi = rotationVector((P.q * allowed) * conjugate(C.q));
            applyAngular(C, &P, dphi, j.limitCompliance / h2, j.lambdaLimit, 0);
        }
        // the muscles: toward the target pose, compliance 1 / (kp s), torque capped
        if (drives && s > 0 && j.kp > 0) {
            const Vec3 dphi = rotationVector((P.q * j.target) * conjugate(C.q));
            applyAngular(C, &P, dphi, 1.0 / (j.kp * s) / h2, j.lambdaDrive, j.torque * s * h2);
        }
    }
    // balance assist: pelvis and chest held upright (and facing the heading)
    if (drives && p.assist > 0) {
        const int segs[2] = {pelvisSeg_, thoraxSeg_};
        for (int k = 0; k < 2; ++k) {
            if (segs[k] < 0) continue;
            Body& b = p.body[static_cast<std::size_t>(segs[k])];
            const Vec3 dphi = rotationVector(p.assistTarget * conjugate(b.q));
            applyAngular(b, nullptr, dphi, 1.0 / (assistStiffness * p.assist) / h2, p.assistLambda[k], assistTorque * p.assist * h2);
        }
    }
}

void World::solveGround(Pedestrian& p, Real h) {
    const Vec3 up{0, 0, 1};
    for (int s = 0; s < p.count; ++s) {
        Body& b = p.body[static_cast<std::size_t>(s)];
        const Vec3 pts[3] = {b.la, (b.la + b.lb) * 0.5, b.lb};
        for (const Vec3& lp : pts) {
            const Vec3 c = b.x + rotate(b.q, lp) - up * b.radius;
            const Real depth = p.groundZ - c.z;
            if (depth <= 0) continue;
            b.grounded = true;
            const Vec3 ra = c - b.x;
            const Vec3 lr = rotateInv(b.q, ra);
            const Vec3 was = b.xPrev + rotate(b.qPrev, lr);
            const Real approach = was.z - c.z;
            // the body's tissue on the (rigid) pavement, damped
            const Real w = genInvMass(b, ra, up);
            const Real alpha = b.tissueCompliance;
            const Real gamma = contactGamma(alpha, w, groundDamping, depth, approach / h, h);
            if (s == headSeg_ && !b.wasGrounded && approach / h > 2.0 && p.report.struck) {
                p.report.groundHeadSpeed = std::max(p.report.groundHeadSpeed, approach / h);
                emit(EventType::HeadImpact, p, s, static_cast<int>(Region::Ground), c, approach / h);
            }
            const Real dl = applyPositional(&b, nullptr, ra, {}, up * depth, alpha / (h * h), gamma, approach);
            // Coulomb friction: undo the slide, up to mu x the normal multiplier
            const Vec3 now = b.x + rotate(b.q, lr);
            Vec3 slide = now - was;
            slide.z = 0;
            const Real sl = length(slide);
            if (sl > 1e-12 && dl > 0) {
                const Vec3 dir = slide / sl;
                const Real wt = genInvMass(b, now - b.x, dir);
                const Real lt = std::min(sl / wt, groundFriction * dl);
                applyPositional(&b, nullptr, now - b.x, {}, dir * (-lt * wt), 0, 0, 0);
            }
        }
    }
}

void World::vehiclePose(int v, Real tFrac, Vec3& c, Quat& q) const {
    const VehicleState& vs = veh_[static_cast<std::size_t>(v)];
    const Real t = tFrac * stepDt_;
    c = vs.position + vs.velocity * t;
    q = integrate(vs.orientation, vs.angularVelocity, t);
}

void World::markStruck(Pedestrian& p, int vehicle, Real speed, int segment, int region, const Vec3& at) {
    Report& r = p.report;
    r.struck = true;
    r.vehicle = veh_[static_cast<std::size_t>(vehicle)].id;
    r.firstContactTime = subTime_;
    r.impactSpeed = speed;
    r.throwOrigin = p.body[static_cast<std::size_t>(pelvisSeg_)].x;
    r.maxPelvisHeight = r.pelvisHeightAtImpact = r.throwOrigin.z - p.groundZ;
    r.vehiclePhase = true;
    r.lastVehicleContact = subTime_;
    const int prof = veh_[static_cast<std::size_t>(vehicle)].profile;
    r.bleHeight = prof >= 0 ? vehicles.at(prof).bleHeight : 0;
    p.vehicleIndex = vehicle;
    emit(EventType::VehicleContact, p, segment, region, at, speed);
    if (speed > limpOnContactSpeed) {
        p.strength = std::min(p.strength, limpStrength);
        p.assist = 0;
    }
}

void World::solveVehicles(Pedestrian& p, Real h, Real tFrac) {
    for (int v = 0; v < vehCount_; ++v) {
        const VehicleState& vs = veh_[static_cast<std::size_t>(v)];
        if (vs.profile < 0) continue;
        const VehicleProfile& prof = vehicles.at(vs.profile);
        Vec3 c;
        Quat q;
        vehiclePose(v, tFrac, c, q);
        const Vec3 boundCentre = c + rotate(q, prof.boundCentre);
        if (length(p.body[static_cast<std::size_t>(pelvisSeg_)].x - boundCentre) > prof.boundRadius + 1.2 * p.stature) continue;
        // the parts' poses this substep, once for every segment
        const int np = std::min(static_cast<int>(prof.parts.size()), kMaxParts);
        for (int k = 0; k < np; ++k) {
            const VehiclePart& part = prof.parts[static_cast<std::size_t>(k)];
            partC_[static_cast<std::size_t>(k)] = c + rotate(q, part.centre);
            partQ_[static_cast<std::size_t>(k)] = q * part.orientation;
            partR_[static_cast<std::size_t>(k)] = length(part.half);
        }
        for (int s = 0; s < p.count; ++s) {
            Body& b = p.body[static_cast<std::size_t>(s)];
            const Real reachB = b.halfLength + b.radius + 0.02;
            if (length(b.x - boundCentre) > prof.boundRadius + reachB) continue;
            Real bestDl = 0;
            for (int k = 0; k < np; ++k) {
                const VehiclePart& part = prof.parts[static_cast<std::size_t>(k)];
                const Vec3& pc = partC_[static_cast<std::size_t>(k)];
                if (length(b.x - pc) > partR_[static_cast<std::size_t>(k)] + reachB) continue;
                const Quat& pq = partQ_[static_cast<std::size_t>(k)];
                const Vec3 lc = rotateInv(pq, b.x - pc);
                if (localBoxSdf(lc, part.half) > reachB) continue;
                const Vec3 axisL = rotateInv(pq, rotate(b.q, b.lb - b.la)) * 0.5;
                const Vec3 surfV = vs.velocity + cross(vs.angularVelocity, b.x - c);
                Vec3 ptL, nL;
                const Real dist = localSegmentBox(lc - axisL, lc + axisL, part.half, ptL, nL, rotateInv(pq, surfV - b.v));
                const Real depth = b.radius - dist;
                if (depth <= 0) continue;
                const Vec3 n = rotate(pq, nL);
                const Vec3 pt = pc + rotate(pq, ptL);
                const Vec3 cp = pt - n * b.radius;
                const Vec3 ra = cp - b.x;
                const Vec3 lr = rotateInv(b.q, ra);
                const Vec3 was = b.xPrev + rotate(b.qPrev, lr);
                const Vec3 vsur = vs.velocity + cross(vs.angularVelocity, cp - c);
                // how far the contact closed this substep, against the moving surface
                const Real approach = dot(((cp - was) - vsur * h), n * -1.0);
                const Real closing = dot(vsur - b.v, n);
                // tissue in series with the panel, damped by the panel's ratio
                const Real w = genInvMass(b, ra, n);
                const Real alpha = b.tissueCompliance + 1.0 / part.stiffness;
                const Real gamma = contactGamma(alpha, w, part.damping, depth, std::max(closing, approach / h), h);
                const Region region = part.region;
                if (!p.report.struck && closing > 0.3) markStruck(p, v, closing, s, static_cast<int>(region), cp);
                if (s == headSeg_ && b.wasVehicleRegion < 0 && b.vehicleRegion < 0 && closing > 1.0) {
                    emit(EventType::HeadImpact, p, s, static_cast<int>(region), cp, closing);
                    if (p.report.struck && closing > p.report.headImpactSpeed) {  // the hardest strike
                        p.report.headRegion = static_cast<int>(region);
                        p.report.headImpactSpeed = closing;
                        p.report.headImpactTime = subTime_ - p.report.firstContactTime;
                        p.report.headImpactPoint = rotateInv(q, cp - c);
                    }
                }
                if (s == headSeg_ && upperSurface(region) && p.report.struck) p.report.headOnUpperSurface = true;
                const Real dl = applyPositional(&b, nullptr, ra, {}, n * depth, alpha / (h * h), gamma, approach);
                if (dl <= 0) continue;
                if (dl > bestDl) bestDl = dl, b.vehicleRegion = static_cast<int>(region);
                // the vehicle takes the opposite impulse
                Vec3 J = n * (-dl / h);
                // friction against the moving panel
                const Vec3 now = b.x + rotate(b.q, lr);
                Vec3 slide = (now - was) - vsur * h;
                slide -= n * dot(slide, n);
                const Real sl = length(slide);
                if (sl > 1e-12) {
                    const Vec3 dir = slide / sl;
                    const Real wt = genInvMass(b, now - b.x, dir);
                    const Real lt = std::min(sl / wt, part.friction * dl);
                    applyPositional(&b, nullptr, now - b.x, {}, dir * (-lt * wt), 0, 0, 0);
                    J += dir * (lt / h);
                }
                vehJ_[static_cast<std::size_t>(v)] += J;
                vehL_[static_cast<std::size_t>(v)] += cross(cp - vs.position, J);
            }
        }
    }
}

void World::solveSelf(Pedestrian& p, Real h) {
    for (int k = 0; k < p.pairCount; ++k) {
        Body& a = p.body[p.pairs[static_cast<std::size_t>(k)][0]];
        Body& b = p.body[p.pairs[static_cast<std::size_t>(k)][1]];
        const Real reach = a.halfLength + a.radius + b.halfLength + b.radius;
        if (length(a.x - b.x) > reach) continue;
        const Vec3 A0 = a.x + rotate(a.q, a.la), A1 = a.x + rotate(a.q, a.lb);
        const Vec3 B0 = b.x + rotate(b.q, b.la), B1 = b.x + rotate(b.q, b.lb);
        Real s = 0, t = 0;
        closestSegments(A0, A1, B0, B1, s, t);
        const Vec3 pa = A0 + (A1 - A0) * s, pb = B0 + (B1 - B0) * t;
        const Vec3 d = pa - pb;
        const Real dist = length(d);
        Real overlap = a.radius + b.radius - dist;
        if (overlap <= 0) continue;
        // Two limbs crossing fast can carry one axis past the other's in a substep; the nearest
        // way out is then the far side, and pushing that way would pass the foot through the
        // shank. A deep pair is pushed back the way it came instead (as the vehicle contacts are).
        const Vec3 rel = (a.x - a.xPrev) - (b.x - b.xPrev);
        Vec3 n = dist > 1e-9 ? d / dist : normalized(rel * -1.0);
        if (length(n) < 0.5) continue;
        if (dist < 0.5 * (a.radius + b.radius) && dot(n, rel) > 0) {
            const Vec3 back = normalized(rel * -1.0), axis = B1 - B0;
            n = normalized(back - axis * (dot(back, axis) / std::max(dot(axis, axis), 1e-12)));
            if (length(n) < 0.5) continue;
            overlap = a.radius + b.radius - dot(d, n);
        }
        // tissue on tissue (a flailing arm against the head is soft, not a steel bar)
        const Vec3 ra = (pa - n * a.radius) - a.x, rb = (pb + n * b.radius) - b.x;
        const Vec3 la = rotateInv(a.q, ra), lb = rotateInv(b.q, rb);
        const Vec3 wasA = a.xPrev + rotate(a.qPrev, la), wasB = b.xPrev + rotate(b.qPrev, lb);
        const Real approach = dot(((a.x + ra) - wasA) - ((b.x + rb) - wasB), n * -1.0);
        const Real w = genInvMass(a, ra, n) + genInvMass(b, rb, n);
        const Real alpha = a.tissueCompliance + b.tissueCompliance;
        const Real gamma = contactGamma(alpha, w, groundDamping, overlap, approach / h, h);
        applyPositional(&a, &b, ra, rb, n * overlap, alpha / (h * h), gamma, approach);
    }
}

void World::finishSubstep(Pedestrian& p, Real h) {
    const Vec3 g{0, 0, -kGravity};
    for (int s = 0; s < p.count; ++s) {
        Body& b = p.body[static_cast<std::size_t>(s)];
        b.v = (b.x - b.xPrev) / h;
        b.w = rotationVector(b.q * conjugate(b.qPrev)) / h;
    }
    // joint damping: the muscles' (critical for the drive's stiffness) plus a little friction;
    // a part turning with its neighbour is untouched
    for (int k = 1; k < p.count; ++k) {
        const Joint& j = p.joint[static_cast<std::size_t>(k)];
        Body& P = p.body[static_cast<std::size_t>(j.parent)];
        Body& C = p.body[static_cast<std::size_t>(j.child)];
        const Real kp = j.kp * p.strength;
        const Real rate = jointFriction + (kp > 0 ? 2.0 * dampingRatio * std::sqrt(kp / j.inertia) : 0.0);
        const Real damp = std::min(1.0, rate * h);
        const Real wp = std::max(P.invInertia.x, std::max(P.invInertia.y, P.invInertia.z));
        const Real wc = std::max(C.invInertia.x, std::max(C.invInertia.y, C.invInertia.z));
        const Vec3 dw = (C.w - P.w) * damp;
        P.w += dw * (wp / (wp + wc));
        C.w -= dw * (wc / (wp + wc));
    }
    // ligaments: a joint forced past its range harder than its tissue takes tears, and its range
    // opens (a torn knee bends sideways)
    for (int k = 1; k < p.count; ++k) {
        Joint& j = p.joint[static_cast<std::size_t>(k)];
        if (j.torn) continue;
        const Real torque = length(j.lambdaLimit) / (h * h);
        if (torque > j.tearTorque) {
            j.torn = true;
            const Real more = 20 * kDeg;
            j.flexMin -= more, j.flexMax += more, j.abdMin -= more, j.abdMax += more, j.twistMin -= more, j.twistMax += more;
            j.limitCompliance *= 4;
            p.report.torn |= 1u << k;
            emit(EventType::LigamentTear, p, k, j.parent, p.body[static_cast<std::size_t>(k)].x + rotate(p.body[static_cast<std::size_t>(k)].q, j.rc), torque);
        }
    }
    Report& r = p.report;
    if (r.struck && !r.atRest) {
        // the head: specific acceleration (what an accelerometer reads), in g; HIC15
        if (headSeg_ >= 0) {
            const Body& hd = p.body[static_cast<std::size_t>(headSeg_)];
            const Real a = length((hd.v - hd.vLast) / h - g) / kGravity;
            r.headPeakG = std::max(r.headPeakG, a);
            if (!r.vehiclePhase) r.groundHeadPeakG = std::max(r.groundHeadPeakG, a);
            p.hicCum += a * h;
            const int idx = (p.hicHead + p.hicCount) % kHicSamples;
            p.hicT[static_cast<std::size_t>(idx)] = subTime_;
            p.hicS[static_cast<std::size_t>(idx)] = p.hicCum;
            if (p.hicCount < kHicSamples) ++p.hicCount;
            else p.hicHead = (p.hicHead + 1) % kHicSamples;
            // every window ending now and no longer than 15 ms; the cumulative sum at the
            // window's start is the sample before it
            for (int m = 0; m < p.hicCount - 1; ++m) {
                const int i0 = (p.hicHead + m) % kHicSamples;
                const Real dt = subTime_ - p.hicT[static_cast<std::size_t>(i0)];
                if (dt > 0.015 + 1e-9 || dt <= 0) continue;
                const Real dS = p.hicCum - p.hicS[static_cast<std::size_t>(i0)];
                if (dS <= 0) continue;
                const Real hic = dm::pow(dS / dt, 2.5) * dt;
                if (hic > r.hic15) {
                    r.hic15 = hic;
                    r.hicT1 = p.hicT[static_cast<std::size_t>(i0)];
                    r.hicT2 = subTime_;
                }
                if (r.vehiclePhase) r.hicVehicle = std::max(r.hicVehicle, hic);
                else r.hicGround = std::max(r.hicGround, hic);
            }
        }
        // the legs: upper-tibia acceleration and the knee's lateral bending, in the bumper phase
        for (int side = 0; side < 2 && subTime_ - r.firstContactTime < 0.08; ++side) {
            const int sh = shankSeg_[side];
            if (sh < 0) continue;
            const Body& b = p.body[static_cast<std::size_t>(sh)];
            r.tibiaPeakG[side] = std::max(r.tibiaPeakG[side], length((b.v - b.vLast) / h - g) / kGravity);
            const Joint& j = p.joint[static_cast<std::size_t>(sh)];
            const Quat rel = conjugate(p.body[static_cast<std::size_t>(j.parent)].q) * b.q;
            Quat swing;
            swingTwist(rel, j.t, swing);
            r.kneeBendPeakDeg[side] = std::max(r.kneeBendPeakDeg[side], std::abs(dot(rotationVector(swing), j.b)) / kDeg);
        }
        // contact and flight
        bool touching = false, onVehicle = false;
        for (int s = 0; s < p.count; ++s) {
            const Body& b = p.body[static_cast<std::size_t>(s)];
            touching = touching || b.grounded || b.vehicleRegion >= 0;
            onVehicle = onVehicle || b.vehicleRegion >= 0;
        }
        if (onVehicle) r.carryTime += h, r.lastVehicleContact = subTime_;
        else if (r.vehiclePhase && subTime_ - r.lastVehicleContact > 0.2) r.vehiclePhase = false;
        if (!touching) {
            if (!p.airborne) p.airborne = true, p.airborneSince = subTime_;
            r.airTime += h;
        } else if (p.airborne) {
            p.airborne = false;
            const Real flight = subTime_ - p.airborneSince;
            if (flight > 0.2 && !p.landedReported) {
                p.landedReported = true;
                emit(EventType::Landed, p, pelvisSeg_, -1, p.body[static_cast<std::size_t>(pelvisSeg_)].x, flight);
            }
        }
        const Body& pel = p.body[static_cast<std::size_t>(pelvisSeg_)];
        r.maxPelvisHeight = std::max(r.maxPelvisHeight, pel.x.z - p.groundZ);
        // where the pelvis went relative to the vehicle that struck
        if (p.vehicleIndex >= 0 && p.vehicleIndex < vehCount_ && veh_[static_cast<std::size_t>(p.vehicleIndex)].id == r.vehicle &&
            veh_[static_cast<std::size_t>(p.vehicleIndex)].profile >= 0) {
            const VehicleState& vs = veh_[static_cast<std::size_t>(p.vehicleIndex)];
            const VehicleProfile& prof = vehicles.at(vs.profile);
            Vec3 c;
            Quat q;
            vehiclePose(p.vehicleIndex, (subTime_ - (time_ - stepDt_)) / stepDt_, c, q);
            const Vec3 pv = rotateInv(q, pel.x - c);
            const bool within = std::abs(pv.y) < prof.halfWidth;
            if (within && pv.x < prof.roofFront && pv.z > prof.roofHeight - 0.25) r.overRoof = true;
            if (within && pv.x < prof.front - 0.3 && pv.z > prof.bleHeight - 0.1) r.onTop = true;
            if (!within && std::abs(pv.y) > prof.halfWidth + 0.1 && pv.x > prof.roofFront && pv.z > prof.bleHeight - 0.1 && onVehicle)
                r.offSide = true;
        }
    }
    for (int s = 0; s < p.count; ++s) p.body[static_cast<std::size_t>(s)].vLast = p.body[static_cast<std::size_t>(s)].v;
}

void World::classify(Pedestrian& p) {
    Report& r = p.report;
    if (!r.struck) r.trajectory = Trajectory::None;
    else if (r.overRoof) r.trajectory = Trajectory::RoofVault;
    else if (r.offSide) r.trajectory = Trajectory::FenderVault;
    // Ravani et al. (1981): wrap when the front strikes below the person's centre of mass and the
    // upper body folds onto the bonnet; forward projection when the front reaches the centre of
    // mass (an SUV, a van, or a child in front of a car) and pushes the person ahead
    else if (r.onTop || (r.headOnUpperSurface && r.pelvisHeightAtImpact > r.bleHeight)) r.trajectory = Trajectory::Wrap;
    else r.trajectory = Trajectory::ForwardProjection;
}

int World::step(Real dt) {
    const auto t0 = std::chrono::steady_clock::now();
    stepDt_ = dt;
    for (int v = 0; v < vehCount_; ++v) vehJ_[static_cast<std::size_t>(v)] = vehL_[static_cast<std::size_t>(v)] = {};
    // wake the sleepers a vehicle is about to reach; follow the vehicle that struck
    int awakeCount = 0;
    for (Pedestrian& p : peds_) {
        p.vehicleIndex = -1;
        for (int v = 0; v < vehCount_; ++v) {
            const VehicleState& vs = veh_[static_cast<std::size_t>(v)];
            if (vs.profile < 0) continue;
            if (vs.id == p.report.vehicle) p.vehicleIndex = v;
            // a parked car beside someone lying still does not wake them; a moving one does
            if (p.awake || length(vs.velocity) + length(vs.angularVelocity) < 0.05) continue;
            const VehicleProfile& prof = vehicles.at(vs.profile);
            const Vec3 centre = vs.position + rotate(vs.orientation, prof.boundCentre);
            const Real reach = prof.boundRadius + 1.2 * p.stature + length(vs.velocity) * dt;
            if (length(centre - p.body[static_cast<std::size_t>(pelvisSeg_)].x) < reach) wake(p);
        }
        awakeCount += p.awake ? 1 : 0;
    }
    const int n = std::max(1, substepsPerStep);
    const Real h = dt / n;
    const Real tStart = time_;
    time_ += dt;
    if (awakeCount == 0) {
        stats_.awake = 0;
        stats_.lastStepMs = 0;
        subTime_ = time_;
        return 0;
    }
    for (int s = 0; s < n; ++s) {
        subTime_ = tStart + h * (s + 1);
        const Real tFrac = static_cast<Real>(s + 1) / n;
        for (Pedestrian& p : peds_) {
            if (!p.awake) continue;
            // joints, the vehicles and the body's own parts, the joints again (so the skeleton
            // carries what the contacts put on it), and the pavement last: it is what a person
            // stands on, and friction undoes the whole substep's slide only if nothing moves
            // the feet after it
            predict(p, h);
            solveJoints(p, h, true);
            solveVehicles(p, h, tFrac);
            solveSelf(p, h);
            solveJoints(p, h, true);
            solveGround(p, h);
            finishSubstep(p, h);
        }
    }
    // sleep: still, or only trembling in place (drift over the window, as in Street Dynamics)
    awakeCount = 0;
    for (Pedestrian& p : peds_) {
        if (!p.awake) continue;
        Real vmax = 0, drift = 0;
        bool onVehicle = false;
        for (int s = 0; s < p.count; ++s) {
            const Body& b = p.body[static_cast<std::size_t>(s)];
            vmax = std::max(vmax, length(b.v) + 0.1 * length(b.w));
            const Quat dq = conjugate(b.settleQ) * b.q;
            drift = std::max(drift, length(b.x - b.settleX) + 0.2 * length(Vec3{dq.x, dq.y, dq.z}));
            onVehicle = onVehicle || b.vehicleRegion >= 0;
        }
        const bool quiet = !onVehicle && (vmax < sleepSpeed || (vmax < 200 * sleepSpeed && drift < sleepSpeed * sleepTime));
        if (quiet) p.still += dt;
        else {
            p.still = 0;
            for (int s = 0; s < p.count; ++s) {
                Body& b = p.body[static_cast<std::size_t>(s)];
                b.settleX = b.x, b.settleQ = b.q;
            }
        }
        if (p.still > sleepTime) {
            p.awake = false;
            for (int s = 0; s < p.count; ++s) {
                Body& b = p.body[static_cast<std::size_t>(s)];
                b.v = b.w = b.vLast = {};
            }
            Report& r = p.report;
            if (r.struck && !r.atRest) {
                r.atRest = true;
                r.restPosition = p.body[static_cast<std::size_t>(pelvisSeg_)].x;
                const Vec3 d = r.restPosition - r.throwOrigin;
                r.throwDistance = std::sqrt(d.x * d.x + d.y * d.y);
                classify(p);
                emit(EventType::AtRest, p, pelvisSeg_, static_cast<int>(r.trajectory), r.restPosition, r.throwDistance);
            }
            continue;
        }
        ++awakeCount;
    }
    // a report reads sensibly while still in flight too
    for (Pedestrian& p : peds_)
        if (p.report.struck && !p.report.atRest) classify(p);
    const auto t1 = std::chrono::steady_clock::now();
    stats_.awake = awakeCount;
    stats_.lastStepMs = std::chrono::duration<double, std::milli>(t1 - t0).count();
    stats_.maxStepMs = std::max(stats_.maxStepMs, stats_.lastStepMs);
    stats_.steps += 1;
    return n;
}

}  // namespace ped
