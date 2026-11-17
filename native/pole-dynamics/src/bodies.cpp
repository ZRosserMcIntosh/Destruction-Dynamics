// Pole Dynamics — utility poles and overhead wires.
// SPDX-License-Identifier: MIT
//
// Poles as rigid bodies, and the impact.
//
// A standing pole that has been disturbed is one rigid body (the whole pole,
// buried part included) on a ball joint at its pivot in the soil, about 2/3 of
// the embedment down. The soil is an elastic-plastic rotational spring with a
// damper: below its yield moment the pole springs back; past it the plastic
// tilt grows (LEANING), and past the failure angle the foundation lets go.
// The section at the ground line carries the moment the soil reacts; when that
// passes the pole's ultimate moment (strong or weak axis by the moment's
// direction, an ellipse between) the pole SNAPS: the stub stays, the upper
// piece becomes its own body hinged on the rebar (a ball joint with a small
// residual moment), which tears free past its strength. A detached
// transformer becomes a body of its own.
//
// The car's hit is a pulse, resolved when the vehicle module reports it: the
// car is a mass on its crush spring, the pole a rigid body pivoting in the
// soil. Stepped at 50 µs until they separate, it gives the peak force (shear
// at bumper height cracks or shears the pole through), the moment at the
// ground line (corrected for the inertia of the pole above it), the soil's
// plastic tilt, and the impulse each side took.
//
// Bodies are integrated semi-implicitly at the internal step (240 Hz), with
// velocity-level impulses for the joint and the contacts (ground plane, host
// capsules).
#include <algorithm>

#include "pd/dmath.hpp"
#include "pd/world.hpp"

namespace pd {

namespace {

constexpr int kIterations = 8;
constexpr Real kBaumgarte = 0.2;

struct Mat3 {
    Real m[3][3]{};
    Vec3 mul(const Vec3& v) const {
        return {m[0][0] * v.x + m[0][1] * v.y + m[0][2] * v.z, m[1][0] * v.x + m[1][1] * v.y + m[1][2] * v.z,
                m[2][0] * v.x + m[2][1] * v.y + m[2][2] * v.z};
    }
    Mat3 inverse() const {
        const auto& a = m;
        const Real det = a[0][0] * (a[1][1] * a[2][2] - a[1][2] * a[2][1]) - a[0][1] * (a[1][0] * a[2][2] - a[1][2] * a[2][0]) +
                         a[0][2] * (a[1][0] * a[2][1] - a[1][1] * a[2][0]);
        Mat3 r;
        if (std::abs(det) < 1e-300) return r;
        const Real id = 1.0 / det;
        r.m[0][0] = (a[1][1] * a[2][2] - a[1][2] * a[2][1]) * id;
        r.m[0][1] = (a[0][2] * a[2][1] - a[0][1] * a[2][2]) * id;
        r.m[0][2] = (a[0][1] * a[1][2] - a[0][2] * a[1][1]) * id;
        r.m[1][0] = (a[1][2] * a[2][0] - a[1][0] * a[2][2]) * id;
        r.m[1][1] = (a[0][0] * a[2][2] - a[0][2] * a[2][0]) * id;
        r.m[1][2] = (a[0][2] * a[1][0] - a[0][0] * a[1][2]) * id;
        r.m[2][0] = (a[1][0] * a[2][1] - a[1][1] * a[2][0]) * id;
        r.m[2][1] = (a[0][1] * a[2][0] - a[0][0] * a[2][1]) * id;
        r.m[2][2] = (a[0][0] * a[1][1] - a[0][1] * a[1][0]) * id;
        return r;
    }
};

// Rotation vector (axis * angle) taking unit vector a to unit vector b.
Vec3 tiltVector(const Vec3& a, const Vec3& b) {
    const Vec3 c = cross(a, b);
    const Real s = length(c);
    if (s < 1e-12) return {};
    return c * (dm::atan2(s, dot(a, b)) / s);
}

Quat axisAngle(const Vec3& rv) {
    const Real ang = length(rv);
    if (ang < 1e-15) return {};
    Real s, c;
    dm::sincos(0.5 * ang, s, c);
    const Vec3 ax = rv / ang;
    return {c, ax.x * s, ax.y * s, ax.z * s};
}

// Closest points between segments p1-q1 and p2-q2 (Ericson, RTCD 5.1.9).
void closestSegments(const Vec3& p1, const Vec3& q1, const Vec3& p2, const Vec3& q2, Vec3& c1, Vec3& c2) {
    const Vec3 d1 = q1 - p1, d2 = q2 - p2, r = p1 - p2;
    const Real a = dot(d1, d1), e = dot(d2, d2), f = dot(d2, r);
    Real s = 0, t = 0;
    if (a <= 1e-12 && e <= 1e-12) {
        c1 = p1;
        c2 = p2;
        return;
    }
    if (a <= 1e-12) {
        t = clamp(f / e, 0.0, 1.0);
    } else {
        const Real c = dot(d1, r);
        if (e <= 1e-12) {
            s = clamp(-c / a, 0.0, 1.0);
        } else {
            const Real b = dot(d1, d2), denom = a * e - b * b;
            s = denom > 1e-12 ? clamp((b * f - c * e) / denom, 0.0, 1.0) : 0.0;
            t = (b * s + f) / e;
            if (t < 0) {
                t = 0;
                s = clamp(-c / a, 0.0, 1.0);
            } else if (t > 1) {
                t = 1;
                s = clamp((b - c) / a, 0.0, 1.0);
            }
        }
    }
    c1 = p1 + d1 * s;
    c2 = p2 + d2 * t;
}

}  // namespace

// ---------------------------------------------------------------- helpers

Vec3 World::bodyPoint(const Body& b, const Vec3& local) const { return b.x + rotate(b.q, local); }

Vec3 World::pieceLocal(const Body& b, int pole, int slot) const {
    const Vec3 l = attachmentLocal(pole, slot);
    return {l.x - b.comOffset.x, l.y - b.comOffset.y, l.z - b.comHeight};
}

Vec3 World::transformerPoint(int pole) const {
    const Pole& p = poles_[static_cast<std::size_t>(pole)];
    const Vec3 l{0, -params.geometry.transformerOffset, params.geometry.transformerHeight};
    if (p.body >= 0) {
        const Body& b = bodies_[static_cast<std::size_t>(p.body)];
        return bodyPoint(b, Vec3{l.x - b.comOffset.x, l.y - b.comOffset.y, l.z - b.comHeight});
    }
    return p.base + rotate(p.tilt * p.rest, l);
}

Vec3 World::invInertiaWorld(const Body& b, const Vec3& v) const {
    if (b.kind == 1) return v * (1.0 / b.iPerp);
    const Vec3 l = rotateInv(b.q, v);
    return rotate(b.q, Vec3{l.x / b.iPerp, l.y / b.iPerp, l.z / b.iAxial});
}

void World::applyImpulse(Body& b, const Vec3& j, const Vec3& at) {
    b.v += j * b.invMass;
    b.w += invInertiaWorld(b, cross(at - b.x, j));
}

int World::allocBody() {
    for (std::size_t i = 0; i < bodies_.size(); ++i)
        if (!bodies_[i].active) {
            bodies_[i] = Body{};
            bodies_[i].active = true;
            return static_cast<int>(i);
        }
    return -1;
}

Real World::momentCapacity(const Pole& p, const Vec3& momentDir, Real factor) const {
    // A moment about the street axis bends the pole along the line (the
    // flange: strong); about the line axis, across it (weak). An ellipse in between.
    const PoleStrength& st = params.strength;
    const Real nominal = st.cls[static_cast<std::size_t>(p.spec.cls)].nominalLoad * (params.geometry.lengthAbove - 0.1);
    const Real strong = nominal * factor * p.strength;
    const Real weak = strong * st.weakAxisFactor;
    const Vec3 d = normalized(Vec3{momentDir.x, momentDir.y, 0});
    if (length(d) < 0.5) return strong;
    const Real ms = dot(d, p.street), mf = dot(d, p.fwd);
    return 1.0 / std::sqrt(sq(ms / strong) + sq(mf / weak));
}

// ---------------------------------------------------------------- dynamic poles

void World::makeDynamic(int pole, int wakeDepth) {
    Pole& p = poles_[static_cast<std::size_t>(pole)];
    if (p.body >= 0) {
        Body& b = bodies_[static_cast<std::size_t>(p.body)];
        b.asleep = false;
        b.still = 0;
        return;
    }
    const int bi = allocBody();
    if (bi < 0) return;
    Body& b = bodies_[static_cast<std::size_t>(bi)];
    const PoleGeometry& g = params.geometry;
    const Real z0 = p.breakHeight > 0 ? p.breakHeight : -g.buried;
    const MassProps mp = massProperties(g, p.spec, z0, g.lengthAbove, p.transformerAttached);
    b.pole = pole;
    b.kind = 0;
    b.mass = mp.mass;
    b.invMass = 1.0 / mp.mass;
    b.iPerp = mp.inertiaPerp;
    b.iAxial = mp.inertiaAxial;
    b.z0 = z0;
    b.z1 = g.lengthAbove;
    b.comHeight = mp.comHeight;
    b.comOffset = mp.comOffset;
    b.q = p.tilt * p.rest;
    b.x = p.base + rotate(b.q, Vec3{mp.comOffset.x, mp.comOffset.y, mp.comHeight});
    b.joint = 1;
    b.anchorLocal = Vec3{-mp.comOffset.x, -mp.comOffset.y, -g.pivotDepth - mp.comHeight};
    b.anchor = bodyPoint(b, b.anchorLocal);
    b.restUp = {0, 0, 1};
    b.plastic = tiltVector(Vec3{0, 0, 1}, rotate(p.tilt, Vec3{0, 0, 1}));
    const Real wet = params.sim.wetness;
    const SoilParams& s = params.soil;
    b.stiffness = s.stiffness * (1.0 + (s.wetStiffness - 1.0) * wet);
    b.yield = s.yieldMoment * (1.0 + (s.wetYield - 1.0) * wet);
    p.body = bi;
    wakeAround(pole, wakeDepth < 0 ? params.sim.wakeNeighbours : wakeDepth);
}

void World::snapPole(int pole, Real height, const Vec3& kickImpulse, const Vec3& kickPoint) {
    Pole& p = poles_[static_cast<std::size_t>(pole)];
    if (p.breakHeight > 0 || p.state == PoleState::Snapped || p.state == PoleState::Fallen) return;
    makeDynamic(pole);
    if (p.body < 0) return;
    Body& b = bodies_[static_cast<std::size_t>(p.body)];
    const PoleGeometry& g = params.geometry;
    height = clamp(height, 0.02, g.lengthAbove - 1.0);
    // The pole's frame origin (its axis at the ground line) and velocity field.
    const Vec3 origin = bodyPoint(b, Vec3{-b.comOffset.x, -b.comOffset.y, -b.comHeight});
    const Vec3 oldCom = b.x;
    p.stubTilt = b.q * conjugate(p.rest);
    const MassProps mp = massProperties(g, p.spec, height, g.lengthAbove, p.transformerAttached);
    b.z0 = height;
    b.mass = mp.mass;
    b.invMass = 1.0 / mp.mass;
    b.iPerp = mp.inertiaPerp;
    b.iAxial = mp.inertiaAxial;
    b.comHeight = mp.comHeight;
    b.comOffset = mp.comOffset;
    b.x = origin + rotate(b.q, Vec3{mp.comOffset.x, mp.comOffset.y, mp.comHeight});
    b.v += cross(b.w, b.x - oldCom);
    b.joint = 2;
    b.anchorLocal = Vec3{-mp.comOffset.x, -mp.comOffset.y, height - mp.comHeight};
    b.anchor = bodyPoint(b, b.anchorLocal);
    b.restUp = rotate(b.q, Vec3{0, 0, 1});
    b.yield = momentCapacity(p, Vec3{1, 0, 0}, params.strength.residualFactor) * params.strength.weakAxisFactor;
    b.asleep = false;
    b.still = 0;
    if (length(kickImpulse) > 0) applyImpulse(b, kickImpulse, kickPoint);
    p.breakHeight = height;
    p.state = PoleState::Snapped;
    if (!p.snappedReported) {
        p.snappedReported = true;
        emit(EventType::PoleSnapped, pole, -1, -1, b.anchor, height, 0);
    }
    if (!p.crackedReported) {
        p.crackedReported = true;
        emit(EventType::PoleCracked, pole, -1, -1, b.anchor, height, 0);
    }
    powerDirty_ = true;
    wakeAround(pole, params.sim.wakeNeighbours);
}

void World::detachTransformer(int pole) {
    Pole& p = poles_[static_cast<std::size_t>(pole)];
    if (!p.transformerAttached) return;
    const int ti = allocBody();
    if (ti < 0) return;
    const Vec3 at = transformerPoint(pole);
    Vec3 vel;
    if (p.body >= 0) {
        Body& b = bodies_[static_cast<std::size_t>(p.body)];
        vel = b.v + cross(b.w, at - b.x);
        // The piece loses the tank: new mass properties, same motion.
        const Vec3 oldCom = b.x;
        const Vec3 origin = bodyPoint(b, Vec3{-b.comOffset.x, -b.comOffset.y, -b.comHeight});
        const MassProps mp = massProperties(params.geometry, p.spec, b.z0, b.z1, false);
        b.mass = mp.mass;
        b.invMass = 1.0 / mp.mass;
        b.iPerp = mp.inertiaPerp;
        b.iAxial = mp.inertiaAxial;
        b.comHeight = mp.comHeight;
        b.comOffset = mp.comOffset;
        b.x = origin + rotate(b.q, Vec3{mp.comOffset.x, mp.comOffset.y, mp.comHeight});
        b.v += cross(b.w, b.x - oldCom);
        b.anchorLocal = Vec3{-mp.comOffset.x, -mp.comOffset.y, (b.joint == 1 ? -params.geometry.pivotDepth : b.z0) - mp.comHeight};
    }
    p.transformerAttached = false;
    Body& t = bodies_[static_cast<std::size_t>(ti)];
    t.pole = pole;
    t.kind = 1;
    t.mass = params.geometry.transformerMass;
    t.invMass = 1.0 / t.mass;
    t.iPerp = t.iAxial = 0.4 * t.mass * sq(0.45);
    t.x = at;
    t.q = p.body >= 0 ? bodies_[static_cast<std::size_t>(p.body)].q : p.tilt * p.rest;
    t.v = vel;
    t.z0 = -0.45;
    t.z1 = 0.45;
    p.transformerBody = ti;
    emit(EventType::TransformerDetached, pole, -1, -1, at, length(vel), 0);
    powerDirty_ = true;
}

// ---------------------------------------------------------------- stepping

void World::stepBodies(Real h) {
    const Vec3 g{0, 0, -kGravity};
    for (std::size_t bi = 0; bi < bodies_.size(); ++bi) {
        Body& b = bodies_[bi];
        if (!b.active || b.asleep) continue;
        Pole& p = poles_[static_cast<std::size_t>(b.pole)];
        Vec3 force = g * b.mass;
        Vec3 torque;
        const Vec3 up = rotate(b.q, Vec3{0, 0, 1});
        if (b.kind == 0) {
            // Wire loads at the attachments on this piece; a standing pole's
            // guy (dead ends, corners) takes the pull along its direction.
            Vec3 pullH;
            for (int slot = 0; slot < 12; ++slot) {
                const Vec3 l = attachmentLocal(b.pole, slot);
                if (l.z < b.z0 || l.z > b.z1) continue;
                const Vec3 f = clampForce_[static_cast<std::size_t>(b.pole) * 12 + static_cast<std::size_t>(slot)];
                if (dot(f, f) == 0) continue;
                const Vec3 at = bodyPoint(b, pieceLocal(b, b.pole, slot));
                force += f;
                torque += cross(at - b.x, f);
                pullH += Vec3{f.x, f.y, 0};
            }
            if (b.joint == 1) {
                const Vec3 guy = guyReaction(p, pullH);
                if (dot(guy, guy) > 0) {
                    const Vec3 at = bodyPoint(b, Vec3{-b.comOffset.x, -b.comOffset.y, p.guyHeight - b.comHeight});
                    force += guy;
                    torque += cross(at - b.x, guy);
                }
            }
            // Soil: elastic-plastic rotational spring and damper about the pivot.
            b.jointMoment = {};
            if (b.joint == 1) {
                const Vec3 tilt = tiltVector(b.restUp, up);
                const Real ang = length(tilt);
                const SoilParams& s = params.soil;
                Real yield = b.yield * (1.0 + s.hardening * b.stiffness * length(b.plastic) / b.yield);
                if (ang > s.failAngle) yield = b.yield * s.failResidual;
                // The soil cannot push back harder than it yields, damping included.
                const Vec3 wPerp = b.w - up * dot(b.w, up);
                const Vec3 e = tilt - b.plastic;
                Vec3 m = e * b.stiffness + wPerp * s.damping;
                const Real mm = length(m);
                if (mm > yield) {
                    const Vec3 dir = m / mm;
                    const Vec3 elastic = e * b.stiffness;
                    const Real over = length(elastic) - yield;
                    if (over > 0) b.plastic += normalized(elastic) * (over / b.stiffness);
                    m = dir * yield;
                }
                torque -= m;
                b.jointMoment = m;
            }
        }
        // Integrate velocities.
        b.v += force * (b.invMass * h);
        b.w += invInertiaWorld(b, torque) * h;
        if (b.kind == 0) b.w -= up * (dot(b.w, up) * 0.5);  // poles do not spin about their axis
        if (b.joint != 1) {
            const Real c = params.strength.pieceDamping + (b.onGround ? params.strength.groundDamping : 0.0);
            const Real k = std::max(0.0, 1.0 - c * h);
            b.w *= k;
            b.v *= std::max(0.0, 1.0 - 0.5 * c * h);
        }
        // Velocity constraints: the joint, the ground, host capsules.
        bodyContacts(b, h);
        // Integrate positions.
        const Vec3 tankBefore = b.kind == 0 && p.transformerAttached ? b.v + cross(b.w, transformerPoint(b.pole) - b.x) : Vec3{};
        b.x += b.v * h;
        b.q = integrate(b.q, b.w, h);
        if (b.kind == 0 && p.transformerAttached) {
            const Vec3 tv = b.v + cross(b.w, transformerPoint(b.pole) - b.x);
            b.tankAccel = length(tv - tankBefore) / h;
            b.tankVel = tv;
        }
        // Sleep: judged on the net movement over the sleep window, so a pole
        // jittering in its wires or creeping a few millimetres still sleeps.
        b.still += h;
        if (b.still >= params.sim.sleepTime) {
            const Real moved = length(b.x - b.windowX);
            const Real dq = std::abs(b.q.w * b.windowQ.w + b.q.x * b.windowQ.x + b.q.y * b.windowQ.y + b.q.z * b.windowQ.z);
            const Real turned = 2.0 * dm::acos(clamp(dq, 0.0, 1.0));
            // Creeping (a pole sliding in its wires, the line slowly taking up
            // the load) for three windows in a row counts as settled too.
            const bool creeping = moved < 0.3 * params.sim.sleepTime && turned < 3.0 * kDeg * params.sim.sleepTime;
            b.calmWindows = creeping ? b.calmWindows + 1 : 0;
            const bool quiet = time() - lastStructural_ > params.sim.quietTime;
            if ((moved < 0.02 && turned < 0.3 * kDeg && (b.joint != 0 || b.onGround)) || b.calmWindows >= 3 ||
                (quiet && b.calmWindows >= 1)) {
                b.calmWindows = 0;
                b.asleep = true;
                b.v = b.w = {};
                p.settledMoment = p.wireMoment;
            }
            b.still = 0;
            b.windowX = b.x;
            b.windowQ = b.q;
        }
    }
}

void World::bodyContacts(Body& b, Real h) {
    // Contact points: the piece's ends, the crossarm tips and the tank (if it
    // is on), each a sphere of the local section radius.
    struct Pt { Vec3 local; Real radius; };
    Pt pts[6];
    int n = 0;
    if (b.kind == 1) {
        pts[n++] = {Vec3{}, 0.45};
    } else {
        const Pole& p = poles_[static_cast<std::size_t>(b.pole)];
        const Vec3 c{b.comOffset.x, b.comOffset.y, b.comHeight};
        if (b.joint != 1) pts[n++] = {Vec3{0, 0, b.z0} - c, sectionRadius(params.geometry, b.z0)};
        pts[n++] = {Vec3{0, 0, b.z1} - c, sectionRadius(params.geometry, b.z1)};
        pts[n++] = {Vec3{0, 0, 0.5 * (b.z0 + b.z1)} - c, sectionRadius(params.geometry, 0.5 * (b.z0 + b.z1))};
        if (params.geometry.mvCrossarm > b.z0) {
            pts[n++] = {attachmentLocal(b.pole, 0) - c, 0.08};
            pts[n++] = {attachmentLocal(b.pole, 2) - c, 0.08};
        }
        if (p.transformerAttached && params.geometry.transformerHeight > b.z0)
            pts[n++] = {Vec3{0, -params.geometry.transformerOffset, params.geometry.transformerHeight} - c, 0.45};
    }
    // Pole axis for capsule contacts.
    const Vec3 c{b.comOffset.x, b.comOffset.y, b.comHeight};
    const Vec3 axA = bodyPoint(b, Vec3{0, 0, b.z0} - c), axB = bodyPoint(b, Vec3{0, 0, b.z1} - c);
    const Real axR = b.kind == 1 ? 0.45 : sectionRadius(params.geometry, 0.5 * (b.z0 + b.z1));

    struct Contact { Vec3 r, n; Real bias; Real jn; Vec3 vOther; Real massN; int cap; };
    Contact cs[6 + 8];
    int nc = 0;
    b.onGround = false;
    for (int i = 0; i < n; ++i) {
        const Vec3 wp = bodyPoint(b, pts[i].local);
        const Real depth = pts[i].radius - wp.z;
        if (depth > -0.005) {
            Contact& k = cs[nc++];
            k.r = wp - b.x;
            k.n = {0, 0, 1};
            k.bias = depth > 0 ? kBaumgarte * depth / h : 0;
            k.jn = 0;
            k.vOther = {};
            k.cap = -1;
            b.onGround = true;
        }
    }
    for (int ci = 0; ci < capCount_ && nc < 14; ++ci) {
        const Capsule& cap = caps_[static_cast<std::size_t>(ci)];
        Vec3 p1, p2;
        if (b.kind == 1)
            closestSegments(b.x, b.x, cap.a, cap.b, p1, p2);
        else
            closestSegments(axA, axB, cap.a, cap.b, p1, p2);
        const Vec3 d = p1 - p2;
        const Real dist = length(d);
        const Real depth = axR + cap.radius - dist;
        if (depth <= 0) continue;
        Contact& k = cs[nc++];
        k.n = dist > 1e-9 ? d / dist : Vec3{0, 0, 1};
        k.r = (p1 - k.n * axR) - b.x;
        k.bias = kBaumgarte * depth / h;
        k.jn = 0;
        k.vOther = cap.velocity;
        k.cap = cap.id;
    }
    // A tank remembers how fast it was going when it first touched down.
    if (b.kind == 1 && nc > 0 && !b.landedReported && length(b.tankVel) == 0) b.tankVel = b.v;
    for (int i = 0; i < nc; ++i) {
        Contact& k = cs[i];
        const Vec3 rn = cross(k.r, k.n);
        k.massN = 1.0 / (b.invMass + dot(cross(invInertiaWorld(b, rn), k.r), k.n));
    }
    // Joint effective mass.
    Mat3 kInv;
    Vec3 jr;
    if (b.joint != 0) {
        jr = rotate(b.q, b.anchorLocal);
        Mat3 km;
        const Vec3 basis[3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
        for (int i = 0; i < 3; ++i) {
            const Vec3 col = basis[i] * b.invMass - cross(jr, invInertiaWorld(b, cross(jr, basis[i])));
            km.m[0][i] = col.x;
            km.m[1][i] = col.y;
            km.m[2][i] = col.z;
        }
        kInv = km.inverse();
    }
    b.jointImpulse = {};
    Vec3 hingeAngular;
    for (int it = 0; it < kIterations; ++it) {
        if (b.joint != 0) {
            const Vec3 pos = b.x + jr;
            const Vec3 err = pos - b.anchor;
            const Vec3 vp = b.v + cross(b.w, jr);
            const Vec3 j = kInv.mul(-(vp + err * (kBaumgarte / h)));
            b.v += j * b.invMass;
            b.w += invInertiaWorld(b, cross(jr, j));
            b.jointImpulse += j;
            if (b.joint == 2) {
                // Rebar hinge: rotational friction up to the residual moment.
                const Vec3 up = rotate(b.q, Vec3{0, 0, 1});
                const Vec3 wPerp = b.w - up * dot(b.w, up);
                const Real wp = length(wPerp);
                if (wp > 1e-9) {
                    const Real want = wp / (1.0 / b.iPerp);
                    const Real maxAng = b.yield * h / kIterations;
                    const Real ang = std::min(want, maxAng);
                    const Vec3 dL = wPerp * (-ang / wp);
                    b.w += invInertiaWorld(b, dL);
                    hingeAngular += dL;
                }
            }
        }
        for (int i = 0; i < nc; ++i) {
            Contact& k = cs[i];
            const Vec3 vrel = b.v + cross(b.w, k.r) - k.vOther;
            const Real vn = dot(vrel, k.n);
            Real dj = k.massN * (-(vn) + k.bias - 0.1 * std::min(0.0, vn + 2.0) * 0.0);
            const Real old = k.jn;
            k.jn = std::max(0.0, old + dj);
            dj = k.jn - old;
            applyImpulse(b, k.n * dj, b.x + k.r);
            // Friction (Coulomb, 0.6).
            const Vec3 vr2 = b.v + cross(b.w, k.r) - k.vOther;
            Vec3 vt = vr2 - k.n * dot(vr2, k.n);
            const Real vtl = length(vt);
            if (vtl > 1e-9) {
                const Vec3 t = vt / vtl;
                const Vec3 rt = cross(k.r, t);
                const Real mt = 1.0 / (b.invMass + dot(cross(invInertiaWorld(b, rt), k.r), t));
                const Real jt = std::min(vtl * mt, 0.6 * k.jn);
                applyImpulse(b, t * (-jt), b.x + k.r);
            }
        }
    }
    if (b.joint == 2 && length(b.jointImpulse) / h > params.strength.rebarTear) b.joint = 0;  // rebar tears
    for (int i = 0; i < nc; ++i) {
        const Contact& k = cs[i];
        if (k.cap >= 0 && k.jn > 50.0 && b.lastHitBody != k.cap) {
            b.lastHitBody = k.cap;
            emit(EventType::PoleHitBody, b.pole, k.cap, -1, b.x + k.r, k.jn, b.kind);
        }
    }
}

void World::updatePoleStates() {
    for (std::size_t bi = 0; bi < bodies_.size(); ++bi) {
        Body& b = bodies_[bi];
        if (!b.active) continue;
        const int pi = b.pole;
        Pole& p = poles_[static_cast<std::size_t>(pi)];
        if (b.kind == 1) {
            if (b.onGround && !b.landedReported) {
                b.landedReported = true;
                const Real v = length(b.tankVel);  // the speed it hit the ground at
                std::int64_t flags = 0;
                if (v > 3.0) flags |= 1;  // oil leak
                if (v > 8.0) flags |= 2;  // fire
                emit(EventType::TransformerLanded, pi, -1, -1, b.x, v, flags);
            }
            continue;
        }
        const Vec3 up = rotate(b.q, Vec3{0, 0, 1});
        const Real tilt = dm::acos(clamp(up.z, -1.0, 1.0));
        if (b.joint == 1) {
            const Real m = length(b.jointMoment);
            const Vec3 md = b.jointMoment;
            if (m > momentCapacity(p, md, params.strength.ultimateFactor)) {
                snapPole(pi, 0.0, Vec3{}, Vec3{});
                continue;
            }
            if (m > momentCapacity(p, md, params.strength.crackingFactor) && p.state == PoleState::Intact) {
                p.state = PoleState::Cracked;
                if (!p.crackedReported) {
                    p.crackedReported = true;
                    emit(EventType::PoleCracked, pi, -1, -1, p.base, m, 0);
                }
            }
            if (length(b.plastic) > params.soil.leanAngle && (p.state == PoleState::Intact || p.state == PoleState::Cracked)) {
                p.state = PoleState::Leaning;
                if (!p.leanReported) {
                    p.leanReported = true;
                    emit(EventType::PoleLeaning, pi, -1, -1, p.base, length(b.plastic) / kDeg, 0);
                }
            }
        }
        // Transformer brackets tear off when the tank swings too hard or the pole goes over.
        if (p.transformerAttached &&
            (tilt > params.sim.transformerDetachTilt || b.tankAccel > params.sim.transformerDetachAccel))
            detachTransformer(pi);
        // Down.
        if (tilt > 60 * kDeg && b.onGround && b.asleep && p.state != PoleState::Fallen) {
            p.state = PoleState::Fallen;
            if (!p.fellReported) {
                p.fellReported = true;
                emit(EventType::PoleFell, pi, -1, -1, bodyPoint(b, Vec3{0, 0, 0}), tilt / kDeg, 0);
                checkLanes(pi, -1);
            }
            powerDirty_ = true;
        }
        // A pole that settled standing (cracked or leaning) goes back to being static.
        if (b.joint == 1 && b.asleep && tilt < params.soil.failAngle) {
            p.tilt = b.q * conjugate(p.rest);
            b.active = false;
            p.body = -1;
        }
    }
}

// ---------------------------------------------------------------- impact

ImpactResult World::reportImpact(const ImpactInput& in) {
    ImpactResult r;
    const int pi = findPoleNear(in.point, 1.5);
    if (pi < 0) return r;
    Pole& p = poles_[static_cast<std::size_t>(pi)];
    r.pole = p.spec.id;
    r.state = static_cast<int>(p.state);
    if (p.state == PoleState::Snapped || p.state == PoleState::Fallen) return r;
    Vec3 n = normalized(Vec3{in.normal.x, in.normal.y, 0});
    if (length(n) < 0.5) n = normalized(Vec3{in.relVelocity.x, in.relVelocity.y, 0});
    const Real v0 = dot(in.relVelocity, n);
    if (v0 < 0.05) return r;
    const PoleGeometry& g = params.geometry;
    const PoleClassStrength& cs = params.strength.cls[static_cast<std::size_t>(p.spec.cls)];
    const Real hc = clamp(in.point.z - p.base.z, 0.05, g.lengthAbove - 1.0);
    const Real lever = hc + g.pivotDepth;
    const MassProps mp = massProperties(g, p.spec, -g.buried, g.lengthAbove, p.transformerAttached);
    const Real iPivot = mp.inertiaPerp + mp.mass * sq(mp.comHeight + g.pivotDepth);
    // Moment of the above-ground mass about the ground line per unit angular
    // acceleration about the pivot: inertia relief for the ground-line moment.
    Real relief = 0;
    {
        constexpr int kN = 40;
        const Real dz = g.lengthAbove / kN;
        for (int i = 0; i < kN; ++i) {
            const Real z = (i + 0.5) * dz;
            const MassProps slice = massProperties(g, p.spec, z - 0.5 * dz, z + 0.5 * dz, p.transformerAttached);
            relief += slice.mass * (z + g.pivotDepth) * z;
        }
    }
    const Real wet = params.sim.wetness;
    const SoilParams& sp = params.soil;
    const Real ks = sp.stiffness * (1.0 + (sp.wetStiffness - 1.0) * wet);
    const Real my = sp.yieldMoment * (1.0 + (sp.wetYield - 1.0) * wet);
    const Real kc = in.crushStiffness > 0 ? in.crushStiffness : params.sim.carCrushStiffness;
    const Real m = in.vehicleMass;
    // The current tilt along the hit direction (a cracked pole starts tilted).
    const Vec3 axis = cross(Vec3{0, 0, 1}, n);
    const Vec3 tilt0 = tiltVector(Vec3{0, 0, 1}, rotate(p.tilt, Vec3{0, 0, 1}));
    Real theta = dot(tilt0, axis), omega = 0, plastic = theta;
    Real xc = 0, vc = v0, crushMax = 0, fMax = 0;
    Real impulse = 0, peak = 0, t = 0, alphaMax = 0;
    bool contact = false, cracked = false;
    int snapAt = 0;  // 0 none, 1 impact height (shear), 2 ground line (moment)
    const Real dt = params.sim.impactStep;
    const Real vCrack = cs.impactCrack * p.strength, vUlt = cs.impactUltimate * p.strength;
    const Vec3 mdir = cross(Vec3{0, 0, 1}, n);  // the moment vector of a push along n
    const Real dyn = params.strength.impactMomentFactor;
    const Real mCrack = dyn * momentCapacity(p, mdir, params.strength.crackingFactor);
    const Real mUlt = dyn * momentCapacity(p, mdir, params.strength.ultimateFactor);
    const Real gravityK = mp.mass * kGravity * (mp.comHeight + g.pivotDepth);
    for (int it = 0; it < 20000; ++it) {
        // The car's front: loads on its crush stiffness (capped at the plateau),
        // unloads along a stiffer line (the crush is mostly permanent:
        // restitution ~ sqrt(kc / kUnload) ~ 0.25).
        const Real u = (theta - dot(tilt0, axis)) * lever;
        const Real delta = xc - u;
        const Real kUnload = 16.0 * kc;
        Real F;
        if (delta >= crushMax) {
            crushMax = delta;
            F = kc * delta;
            if (in.crushPlateau > 0 && F > in.crushPlateau) F = in.crushPlateau;
            fMax = F;
        } else {
            F = std::max(0.0, fMax - kUnload * (crushMax - delta));
        }
        if (F > 0) contact = true;
        if (contact && F <= 0 && vc < omega * lever) break;
        // Soil.
        Real ms = ks * (theta - plastic);
        if (std::abs(ms) > my) {
            const Real s = ms > 0 ? 1.0 : -1.0;
            plastic += (std::abs(ms) - my) / ks * s;
            ms = my * s;
        }
        const Real alpha = (F * lever - ms - sp.damping * omega + gravityK * theta) / iPivot;
        alphaMax = std::max(alphaMax, std::abs(alpha));
        // Ground-line moment of the push. (A rigid pole would add the whiplash of
        // its upper part, alpha * relief; a real 11 m pole flexes and its top
        // does not follow the base within a 10-20 ms pulse, so it is left out.)
        const Real mGround = F * hc;
        (void)relief;
        peak = std::max(peak, F);
        if (F > vCrack || std::abs(mGround) > mCrack) cracked = true;
        if (F > vUlt) {
            snapAt = 1;
            break;
        }
        if (std::abs(mGround) > mUlt) {
            snapAt = 2;
            break;
        }
        vc -= F / m * dt;
        xc += vc * dt;
        omega += alpha * dt;
        theta += omega * dt;
        impulse += F * dt;
        t += dt;
    }
    r.peakForce = peak;
    r.duration = t;
    // Hand the pole's motion to the body simulation.
    if (cracked && p.state == PoleState::Intact) {
        p.state = PoleState::Cracked;
        // A cracked section leaves a small permanent tilt along the push.
        plastic += params.strength.crackTilt;
        theta += params.strength.crackTilt;
        if (!p.crackedReported) {
            p.crackedReported = true;
            emit(EventType::PoleCracked, pi, -1, -1, in.point, peak, 0);
        }
    }
    p.tilt = axisAngle(axis * theta);
    makeDynamic(pi);
    if (p.body >= 0) {
        Body& b = bodies_[static_cast<std::size_t>(p.body)];
        b.q = p.tilt * p.rest;
        b.x = b.anchor - rotate(b.q, b.anchorLocal);
        b.w = axis * omega;
        b.v = cross(b.w, b.x - b.anchor);
        b.plastic = axis * plastic;
    }
    // The transformer's brackets see the pole's angular acceleration times
    // the tank's height above the pivot.
    if (p.transformerAttached &&
        alphaMax * (g.transformerHeight + g.pivotDepth) > params.sim.transformerDetachAccel)
        detachTransformer(pi);
    if (snapAt != 0) {
        // The section lets go. The car carries on into the piece above the
        // break: a perfectly plastic exchange with its effective mass there.
        const Real hb = snapAt == 1 ? hc : 0.0;
        const MassProps up = massProperties(g, p.spec, hb, g.lengthAbove, p.transformerAttached);
        const Real rr = up.comHeight - hc;
        const Real mEff = 1.0 / (1.0 / up.mass + rr * rr / up.inertiaPerp);
        const Real vBottom = omega * lever;
        const Real j2 = std::max(0.0, (vc - vBottom)) * m * mEff / (m + mEff);
        vc -= j2 / m;
        impulse += j2;
        r.broke = 1;
        snapPole(pi, hb, Vec3{}, Vec3{});
        if (snapAt == 1 && p.body >= 0) {
            // Sheared through at the bumper: the rebar is cut too. The piece
            // is free and the car kicks its base out from under it.
            Body& b = bodies_[static_cast<std::size_t>(p.body)];
            b.joint = 0;
            applyImpulse(b, n * j2, bodyPoint(b, b.anchorLocal));
        }
    }
    r.impulse = impulse;
    r.vehicleDeltaV = v0 - vc;
    r.state = static_cast<int>(p.state);
    return r;
}

}  // namespace pd
