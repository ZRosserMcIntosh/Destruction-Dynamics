// Pole Dynamics — utility poles and overhead wires.
// SPDX-License-Identifier: MIT
//
// The wires.
//
// At rest a span is analytic: each conductor is the parabola of its rest
// tension, s = w L^2 / (8 H) (within 0.1 % of the catenary at these sags).
// When a pole it hangs from moves, the span wakes: every conductor becomes an
// XPBD rope (Macklin, Müller & Chentanez 2016): 10-24 nodes by length, one
// distance constraint per segment with the compliance of its E*A. A rope's
// constraints form a chain, so each 240 Hz step solves them together exactly:
// the system (J M^-1 J^T + alpha/h^2) dlambda = -C is tridiagonal and the
// Thomas algorithm solves it in O(n) (the direct solve of Deul et al. 2018
// for rods). One pass is converged: a stiff, shallow wire (a 35 m MV span at
// 2.4 kN sags 13.5 cm) keeps its sag and its tension, which a Gauss-Seidel
// pass would not without many more steps. The rope starts on its own
// catenary with rest lengths that hold the rest tension, so waking does not
// jolt it, and T = -lambda / h^2 is the segment tension, which
//   - pulls on the poles at the attachments (bodies.cpp),
//   - breaks a segment past the conductor's rated strength: the two ends
//     whip back with the strain wave speed of the released tension,
//     v = T / sqrt(E A mu), and fall,
//   - and, summed over the spans meeting at an insulator, fails the tie or
//     clamp past its holding force: the conductor comes off the pole and the
//     two spans' end nodes are joined into one free point.
// Ropes collide with the ground (with friction) and with host capsules.
// A woken span that settles goes back to its catenary if nothing changed,
// or freezes as nodes for the session if it did (broken, off its
// insulators, on the street).
#include <algorithm>

#include "pd/dmath.hpp"
#include "pd/world.hpp"

namespace pd {

namespace {
constexpr int kWireRest = 24;  // substeps (0.1 s) without movement before a wire rests
}

Real World::segTension(int rope, int wire, int seg) const {
    const Real h = subDt();
    return -lambda_[static_cast<std::size_t>(rope * kNodesPerSlot + wire * kNodesPerWire + seg)] / (h * h);
}

void World::wakeSpan(int si) {
    Span& s = spans_[static_cast<std::size_t>(si)];
    if (s.mode == SpanMode::Awake) return;
    if (s.mode == SpanMode::Frozen && s.rope >= 0) {
        s.mode = SpanMode::Awake;
        s.still = 0;
        if (awakeCount_ < static_cast<int>(awake_.size())) awake_[static_cast<std::size_t>(awakeCount_++)] = si;
        return;
    }
    int slot = -1;
    for (std::size_t i = 0; i < ropes_.size(); ++i)
        if (!ropes_[i].used) {
            slot = static_cast<int>(i);
            break;
        }
    if (slot < 0 || awakeCount_ >= static_cast<int>(awake_.size())) return;  // pool full: it stays analytic
    Rope& r = ropes_[static_cast<std::size_t>(slot)];
    r.used = true;
    r.span = si;
    const WireParams& wp = params.wires;
    const Real h = subDt();
    Vec3 pts[kNodesPerWire];
    for (int w = 0; w < s.wires; ++w) {
        Wire& wi = r.wire[static_cast<std::size_t>(w)];
        wi = Wire{};
        wi.kind = s.kind[static_cast<std::size_t>(w)];
        const Vec3 pa = spanEnd(s, 0, w), pb = spanEnd(s, 1, w);
        const Real len = length(pb - pa);
        const int n = std::clamp(static_cast<int>(len / wp.nodeSpacing + 0.5) + 1, wp.minNodes, std::min(wp.maxNodes, kNodesPerWire));
        wi.n = n;
        restCatenary(s, w, pa, pb, n, pts);
        const WireType& t = wireType(s, w);
        const Real mu = wireMassPerMetre(s, w);
        const Real lh = std::sqrt(sq(pb.x - pa.x) + sq(pb.y - pa.y));
        const Real nodeMass = mu * std::max(len, 1.0) / (n - 1);
        const auto base = static_cast<std::size_t>(slot * kNodesPerSlot + w * kNodesPerWire);
        const int slotA = s.slotA[static_cast<std::size_t>(w)], slotB = s.slotB[static_cast<std::size_t>(w)];
        wi.pinnedA = (s.a < 0 || slotA < 0 || !poles_[static_cast<std::size_t>(s.a)].clampBroken[static_cast<std::size_t>(slotA)]) ? 1 : 0;
        wi.pinnedB = (s.b < 0 || slotB < 0 || !poles_[static_cast<std::size_t>(s.b)].clampBroken[static_cast<std::size_t>(slotB)]) ? 1 : 0;
        for (int i = 0; i < n; ++i) {
            x_[base + static_cast<std::size_t>(i)] = pts[i];
            prev_[base + static_cast<std::size_t>(i)] = pts[i];
            const bool pinned = (i == 0 && wi.pinnedA) || (i == n - 1 && wi.pinnedB);
            const Real m = (i == 0 || i == n - 1) ? 0.5 * nodeMass : nodeMass;
            invMass_[base + static_cast<std::size_t>(i)] = pinned ? 0.0 : 1.0 / m;
        }
        wi.restLength = 0;
        for (int i = 0; i + 1 < n; ++i) {
            const Vec3 d = pts[i + 1] - pts[i];
            const Real chord = length(d);
            const Real horiz = std::sqrt(d.x * d.x + d.y * d.y);
            const Real tension = lh > 1e-6 && horiz > 1e-9 ? t.tension * chord / horiz : t.tension;
            rest_[base + static_cast<std::size_t>(i)] = chord / (1.0 + tension / t.ea);
            lambda_[base + static_cast<std::size_t>(i)] = -tension * h * h;
            wi.restLength += rest_[base + static_cast<std::size_t>(i)];
        }
        wi.tension = t.tension;
    }
    s.rope = slot;
    s.mode = SpanMode::Awake;
    s.still = 0;
    awake_[static_cast<std::size_t>(awakeCount_++)] = si;
}

void World::wakeAround(int pole, int depth) {
    // Breadth-first over the line, waking every span of every pole within `depth`.
    int queue[64];
    int dist[64];
    int head = 0, tail = 0;
    queue[tail] = pole;
    dist[tail++] = 0;
    while (head < tail) {
        const int u = queue[head];
        const int du = dist[head++];
        const Pole& p = poles_[static_cast<std::size_t>(u)];
        for (int k = 0; k < p.spanCount; ++k) {
            const int si = p.span[k];
            wakeSpan(si);
            const Span& s = spans_[static_cast<std::size_t>(si)];
            const int v = s.a == u ? s.b : s.a;
            if (v < 0 || du + 1 > depth || tail >= 64) continue;
            bool seen = false;
            for (int i = 0; i < tail; ++i) seen = seen || queue[i] == v;
            if (seen) continue;
            queue[tail] = v;
            dist[tail++] = du + 1;
        }
    }
}

void World::sleepSpan(int si, bool toAnalytic) {
    Span& s = spans_[static_cast<std::size_t>(si)];
    for (int k = 0; k < awakeCount_; ++k)
        if (awake_[static_cast<std::size_t>(k)] == si) {
            awake_[static_cast<std::size_t>(k)] = awake_[static_cast<std::size_t>(--awakeCount_)];
            break;
        }
    if (toAnalytic) {
        for (Join& j : joins_)
            if (j.active && (s.rope == j.ropeA || s.rope == j.ropeB)) j.active = false;
        if (s.rope >= 0) ropes_[static_cast<std::size_t>(s.rope)].used = false;
        s.rope = -1;
        s.mode = SpanMode::Analytic;
    } else {
        s.mode = SpanMode::Frozen;
    }
}

void World::ropeForces() {
    // Tension at each pinned end, summed per pole attachment. Only the poles
    // touched last substep are cleared (no pass over the whole tile).
    for (int i = 0; i < touchedCount_; ++i) {
        const auto pi = static_cast<std::size_t>(touched_[static_cast<std::size_t>(i)]);
        touchedFlag_[pi] = 0;
        for (std::size_t slot = 0; slot < 12; ++slot) clampForce_[pi * 12 + slot] = {};
    }
    touchedCount_ = 0;
    auto touch = [&](int pi) {
        if (touchedFlag_[static_cast<std::size_t>(pi)]) return;
        touchedFlag_[static_cast<std::size_t>(pi)] = 1;
        touched_[static_cast<std::size_t>(touchedCount_++)] = pi;
    };
    for (int k = 0; k < awakeCount_; ++k) {
        const int si = awake_[static_cast<std::size_t>(k)];
        const Span& s = spans_[static_cast<std::size_t>(si)];
        const Rope& r = ropes_[static_cast<std::size_t>(s.rope)];
        for (int w = 0; w < s.wires; ++w) {
            const Wire& wi = r.wire[static_cast<std::size_t>(w)];
            const auto base = static_cast<std::size_t>(s.rope * kNodesPerSlot + w * kNodesPerWire);
            if (s.a >= 0) touch(s.a);
            if (s.b >= 0) touch(s.b);
            if (s.a >= 0 && wi.pinnedA && !(wi.broken & 1u)) {
                const Vec3 d = normalized(x_[base + 1] - x_[base]);
                clampForce_[static_cast<std::size_t>(s.a) * 12 + static_cast<std::size_t>(s.slotA[static_cast<std::size_t>(w)])] +=
                    d * std::max(0.0, segTension(s.rope, w, 0));
            }
            const int last = wi.n - 1;
            if (s.b >= 0 && wi.pinnedB && !(wi.broken & (1u << (last - 1)))) {
                const Vec3 d = normalized(x_[base + static_cast<std::size_t>(last - 1)] - x_[base + static_cast<std::size_t>(last)]);
                clampForce_[static_cast<std::size_t>(s.b) * 12 + static_cast<std::size_t>(s.slotB[static_cast<std::size_t>(w)])] +=
                    d * std::max(0.0, segTension(s.rope, w, last - 1));
            }
        }
    }
    // The spans of those poles that are not simulated still pull: analytic
    // spans with their rest load, frozen ones with their last tension.
    for (int ti = 0; ti < touchedCount_; ++ti) {
        const int pi = touched_[static_cast<std::size_t>(ti)];
        const Pole& p = poles_[static_cast<std::size_t>(pi)];
        for (int k = 0; k < p.spanCount; ++k) {
            const Span& s = spans_[static_cast<std::size_t>(p.span[k])];
            if (s.mode == SpanMode::Awake) continue;
            const int end = s.a == pi ? 0 : 1;
            for (int w = 0; w < s.wires; ++w) {
                const int slot = end == 0 ? s.slotA[static_cast<std::size_t>(w)] : s.slotB[static_cast<std::size_t>(w)];
                if (slot < 0 || p.clampBroken[static_cast<std::size_t>(slot)]) continue;
                Vec3 f = s.restEnd[end][static_cast<std::size_t>(w)];
                if (s.mode == SpanMode::Frozen) {
                    const Wire& wi = ropes_[static_cast<std::size_t>(s.rope)].wire[static_cast<std::size_t>(w)];
                    const bool pinned = end == 0 ? wi.pinnedA : wi.pinnedB;
                    if (!pinned || wi.broken) continue;
                    const auto base = static_cast<std::size_t>(s.rope * kNodesPerSlot + w * kNodesPerWire);
                    const int e0 = end == 0 ? 0 : wi.n - 1, e1 = end == 0 ? 1 : wi.n - 2;
                    f = normalized(x_[base + static_cast<std::size_t>(e1)] - x_[base + static_cast<std::size_t>(e0)]) *
                        std::max(0.0, segTension(s.rope, w, end == 0 ? 0 : wi.n - 2));
                }
                clampForce_[static_cast<std::size_t>(pi) * 12 + static_cast<std::size_t>(slot)] += f;
            }
        }
    }
    // A standing pole pulled out of balance hard enough starts moving.
    for (int ti = 0; ti < touchedCount_; ++ti) {
        const auto pi = static_cast<std::size_t>(touched_[static_cast<std::size_t>(ti)]);
        Pole& p = poles_[pi];
        if (!p.active || p.state == PoleState::Fallen) continue;
        Vec3 moment, pullH;
        bool any = false;
        for (int slot = 0; slot < 12; ++slot) {
            const Vec3 f = clampForce_[pi * 12 + static_cast<std::size_t>(slot)];
            if (dot(f, f) == 0) continue;
            any = true;
            const Vec3 l = attachmentLocal(static_cast<int>(pi), slot);
            moment += cross(Vec3{0, 0, l.z + params.geometry.pivotDepth}, Vec3{f.x, f.y, 0});
            pullH += Vec3{f.x, f.y, 0};
        }
        moment += cross(Vec3{0, 0, p.guyHeight + params.geometry.pivotDepth}, guyReaction(p, pullH));
        if (!any) continue;
        p.wireMoment = moment;
        // What wakes a pole is a change from the load it settled under, not
        // the load itself (a neighbour of a hanging pole carries it for good).
        const Vec3 change = moment - p.settledMoment;
        const Real cap = momentCapacity(p, change, params.strength.crackingFactor);
        // Stirred by its wires: only its own spans need waking.
        if (p.body < 0 && p.breakHeight == 0 && length(change) > 0.6 * cap) makeDynamic(static_cast<int>(pi), 0);
        if (p.body >= 0) {
            Body& b = bodies_[static_cast<std::size_t>(p.body)];
            const bool quiet = time() - lastStructural_ > params.sim.quietTime;
            if (b.asleep && length(change) > (quiet ? 0.5 : 0.15) * cap) {
                b.asleep = false;
                b.still = 0;
            }
        }
    }
}

void World::stepRopes(Real h) {
    const Vec3 g{0, 0, -kGravity};
    const Real damp = std::max(0.0, 1.0 - params.wires.damping * h);
    const Real mu = params.wires.groundFriction;
    for (int k = 0; k < awakeCount_; ++k) {
        const int si = awake_[static_cast<std::size_t>(k)];
        Span& s = spans_[static_cast<std::size_t>(si)];
        Rope& r = ropes_[static_cast<std::size_t>(s.rope)];
        for (int w = 0; w < s.wires; ++w) {
            Wire& wi = r.wire[static_cast<std::size_t>(w)];
            const WireType& t = wireType(s, w);
            const auto base = static_cast<std::size_t>(s.rope * kNodesPerSlot + w * kNodesPerWire);
            Vec3* x = &x_[base];
            Vec3* pv = &prev_[base];
            Real* im = &invMass_[base];
            Real* rest = &rest_[base];
            Real* lam = &lambda_[base];
            const int n = wi.n;
            // A wire that has not moved for a while, whose ends have not moved
            // either, rests (most of a woken span's conductors, most of the time).
            const Vec3 ea = wi.pinnedA ? spanEnd(s, 0, w) : x[0];
            const Vec3 eb = wi.pinnedB ? spanEnd(s, 1, w) : x[n - 1];
            if (wi.quiet >= kWireRest) {
                if (length(ea - wi.endA) < 1e-4 && length(eb - wi.endB) < 1e-4 && capCount_ == 0) continue;
                wi.quiet = 0;
            }
            // Predict.
            for (int i = 0; i < n; ++i) {
                if (im[i] == 0) continue;
                const Vec3 v = (x[i] - pv[i]) * damp;
                pv[i] = x[i];
                x[i] += v + g * (h * h);
            }
            if (wi.pinnedA) {
                pv[0] = x[0];
                x[0] = ea;
            }
            if (wi.pinnedB) {
                pv[n - 1] = x[n - 1];
                x[n - 1] = eb;
            }
            // Segments: the whole chain at once (tridiagonal, Thomas algorithm).
            {
                const Real at0 = 1.0 / (t.ea * h * h);
                Vec3 dir[kNodesPerWire];
                Real diag[kNodesPerWire], up[kNodesPerWire], rhs[kNodesPerWire], cp[kNodesPerWire];
                const int m = n - 1;
                for (int i = 0; i < m; ++i) {
                    const bool cut = (wi.broken >> i) & 1u;
                    const Vec3 d = x[i + 1] - x[i];
                    const Real len = length(d);
                    dir[i] = len > 1e-12 ? d / len : Vec3{0, 0, 1};
                    const Real at = at0 * rest[i];
                    diag[i] = cut ? 1.0 : im[i] + im[i + 1] + at;
                    rhs[i] = cut ? 0.0 : -(len - rest[i]);
                    lam[i] = 0;
                }
                for (int i = 0; i + 1 < m; ++i) {
                    const bool cut = ((wi.broken >> i) & 1u) || ((wi.broken >> (i + 1)) & 1u);
                    up[i] = cut ? 0.0 : -im[i + 1] * dot(dir[i], dir[i + 1]);
                }
                // Forward sweep.
                cp[0] = m > 1 ? up[0] / diag[0] : 0;
                rhs[0] = rhs[0] / diag[0];
                for (int i = 1; i < m; ++i) {
                    const Real den = diag[i] - up[i - 1] * cp[i - 1];
                    cp[i] = i + 1 < m ? up[i] / den : 0;
                    rhs[i] = (rhs[i] - up[i - 1] * rhs[i - 1]) / den;
                }
                for (int i = m - 2; i >= 0; --i) rhs[i] -= cp[i] * rhs[i + 1];
                // Apply: segment i pulls node i along +dir and node i+1 along -dir.
                for (int i = 0; i < m; ++i) {
                    if ((wi.broken >> i) & 1u) continue;
                    lam[i] = rhs[i];
                    x[i] -= dir[i] * (im[i] * rhs[i]);
                    x[i + 1] += dir[i] * (im[i + 1] * rhs[i]);
                }
            }
            // Ground and host capsules.
            const Real rad = t.radius;
            for (int i = 0; i < n; ++i) {
                if (im[i] == 0) continue;
                if (x[i].z < rad) {
                    const Real pen = rad - x[i].z;
                    x[i].z = rad;
                    // Static/kinetic friction on the horizontal step.
                    Vec3 dx{x[i].x - pv[i].x, x[i].y - pv[i].y, 0};
                    const Real dl = length(dx);
                    if (dl > 1e-12) {
                        const Real cut = std::min(dl, mu * pen + 0.2 * dl);
                        x[i].x -= dx.x / dl * cut;
                        x[i].y -= dx.y / dl * cut;
                    }
                }
                for (int ci = 0; ci < capCount_; ++ci) {
                    const Capsule& cap = caps_[static_cast<std::size_t>(ci)];
                    const Vec3 ab = cap.b - cap.a;
                    const Real l2 = dot(ab, ab);
                    const Real tt = l2 > 1e-12 ? clamp(dot(x[i] - cap.a, ab) / l2, 0.0, 1.0) : 0.0;
                    const Vec3 c = cap.a + ab * tt;
                    const Vec3 d = x[i] - c;
                    const Real dist = length(d);
                    const Real rr = cap.radius + rad;
                    if (dist < rr && dist > 1e-9) x[i] += d * ((rr - dist) / dist);
                }
            }
            Real tmax = 0, moved = 0;
            for (int i = 0; i + 1 < n; ++i)
                if (!((wi.broken >> i) & 1u)) tmax = std::max(tmax, -lam[i] / (h * h));
            for (int i = 0; i < n; ++i) moved = std::max(moved, std::abs(x[i].x - pv[i].x) + std::abs(x[i].y - pv[i].y) + std::abs(x[i].z - pv[i].z));
            wi.tension = tmax;
            if (moved < 2e-5) {
                if (++wi.quiet >= kWireRest) {
                    wi.endA = ea;
                    wi.endB = eb;
                }
            } else {
                wi.quiet = 0;
            }
        }
        // Breaking.
        for (int w = 0; w < s.wires; ++w) {
            Wire& wi = r.wire[static_cast<std::size_t>(w)];
            const WireType& t = wireType(s, w);
            const auto base = static_cast<std::size_t>(s.rope * kNodesPerSlot + w * kNodesPerWire);
            wi.maxTension = std::max(wi.maxTension, wi.tension);
            if (wi.tension > t.breaking && wi.broken == 0) {
                // It parts at the segment carrying the most.
                int i = 0;
                Real worst = -1;
                for (int k3 = 0; k3 + 1 < wi.n; ++k3) {
                    const Real tk = segTension(s.rope, w, k3);
                    if (tk > worst + 1e-9) {
                        worst = tk;
                        i = k3;
                    }
                }
                const Real T = worst;
                wi.broken |= 1u << i;
                // Whip: release the strain at the wave speed of the tension.
                const Real mu2 = wireMassPerMetre(s, w);
                const Real vWhip = params.wires.recoil * T / std::sqrt(t.ea * mu2);
                const Vec3 dir = normalized(x_[base + static_cast<std::size_t>(i + 1)] - x_[base + static_cast<std::size_t>(i)]);
                for (int k2 = 0; k2 < 4; ++k2) {
                    const Real f = 1.0 - 0.22 * k2;
                    if (i - k2 >= 0 && invMass_[base + static_cast<std::size_t>(i - k2)] > 0)
                        prev_[base + static_cast<std::size_t>(i - k2)] += dir * (vWhip * f * h);
                    if (i + 1 + k2 < wi.n && invMass_[base + static_cast<std::size_t>(i + 1 + k2)] > 0)
                        prev_[base + static_cast<std::size_t>(i + 1 + k2)] -= dir * (vWhip * f * h);
                }
                emit(EventType::WireSnapped, s.a, si, w, x_[base + static_cast<std::size_t>(i)], T, wi.kind);
                powerDirty_ = true;
            }
        }
    }
    // Wires that came off an insulator: the two spans' ends are one point.
    for (Join& j : joins_) {
        if (!j.active) continue;
        Vec3& a = x_[static_cast<std::size_t>(j.ropeA * kNodesPerSlot + j.nodeA)];
        Vec3& b = x_[static_cast<std::size_t>(j.ropeB * kNodesPerSlot + j.nodeB)];
        const Vec3 mid = (a + b) * 0.5;
        a = mid;
        b = mid;
    }
}

void World::checkClamps() {
    // An insulator tie or clamp holds the resultant of the pulls of the
    // spans meeting on it. Past its holding force the conductor comes off.
    for (int ti = 0; ti < touchedCount_; ++ti) {
        const auto pi = static_cast<std::size_t>(touched_[static_cast<std::size_t>(ti)]);
        Pole& p = poles_[pi];
        if (!p.active) continue;
        for (int slot = 0; slot < 12; ++slot) {
            if (p.clampBroken[static_cast<std::size_t>(slot)]) continue;
            const Vec3 f = clampForce_[pi * 12 + static_cast<std::size_t>(slot)];
            if (dot(f, f) == 0) continue;
            const int kind = slot < 3 ? 0 : (slot < 7 ? 1 : (slot == 7 ? 2 : 3));
            // Fittings differ (seeded per pole and slot); a pole going over
            // carries its conductors down with it (the ties are loaded along the pin).
            const auto mix = static_cast<std::uint32_t>(p.spec.id) * 2654435761u ^ (static_cast<std::uint32_t>(slot) * 40503u + 0x9e37u);
            const Real u = static_cast<Real>((mix >> 8) % 1000u) / 999.0;
            Real hold = params.wires.type[static_cast<std::size_t>(kind)].clampBreak *
                        (1.0 + params.wires.clampVariability * (2.0 * u - 1.0));
            if (p.body >= 0 && (p.state == PoleState::Snapped || p.state == PoleState::Fallen || p.state == PoleState::Leaning))
                hold *= params.wires.carriedFactor;
            const Real deadEnd = params.wires.deadEndFactor * params.wires.type[static_cast<std::size_t>(kind)].breaking;
            // The fitting is built for its static load (a dead end uses a strain
            // clamp, a corner a guyed fitting): it fails on what changes.
            // Sideways or along the line the pin and the tie hold together;
            // lifted off the top of the insulator (along the pole) only the tie does.
            const Vec3 extra = f - p.restForce[slot];
            const Vec3 axis = p.body >= 0 ? rotate(bodies_[static_cast<std::size_t>(p.body)].q, Vec3{0, 0, 1}) : rotate(p.tilt, Vec3{0, 0, 1});
            const Real lift = std::max(0.0, dot(extra, axis));
            const Real side = length(extra - axis * dot(extra, axis));
            const Real load = length(extra);
            const bool goingOver = p.body >= 0 && (p.state == PoleState::Snapped || p.state == PoleState::Fallen);
            const bool fails = side / hold + lift / ((goingOver ? 1.0 : params.wires.liftFactor) * hold) > 1.0;
            if (!fails || length(f) <= length(p.restForce[slot])) continue;  // a slack wire breaks nothing
            // A conductor that ends here is in a strain clamp, not a tie
            // (counted only when the tie would fail: this runs every step).
            int through = 0;
            for (int k = 0; k < p.spanCount; ++k) {
                const Span& s2 = spans_[static_cast<std::size_t>(p.span[k])];
                for (int w = 0; w < s2.wires; ++w)
                    through += ((s2.a == static_cast<int>(pi) && s2.slotA[static_cast<std::size_t>(w)] == slot) ||
                                (s2.b == static_cast<int>(pi) && s2.slotB[static_cast<std::size_t>(w)] == slot)) ? 1 : 0;
            }
            if (through <= 1 && load <= deadEnd) continue;
            p.clampBroken[static_cast<std::size_t>(slot)] = 1;
            emit(EventType::ClampFailed, static_cast<int>(pi), -1, slot, attachmentPoint(static_cast<int>(pi), slot), load, kind);
            powerDirty_ = true;
            // Free the ends at this pole; join the two spans of a through conductor.
            int ropeEnd[2], nodeEnd[2], found = 0;
            for (int k = 0; k < p.spanCount; ++k) {
                Span& s = spans_[static_cast<std::size_t>(p.span[k])];
                if (s.rope < 0) continue;
                Rope& r = ropes_[static_cast<std::size_t>(s.rope)];
                for (int w = 0; w < s.wires; ++w) {
                    Wire& wi = r.wire[static_cast<std::size_t>(w)];
                    int node = -1;
                    if (s.a == static_cast<int>(pi) && s.slotA[static_cast<std::size_t>(w)] == slot && wi.pinnedA) {
                        wi.pinnedA = 0;
                        node = 0;
                    } else if (s.b == static_cast<int>(pi) && s.slotB[static_cast<std::size_t>(w)] == slot && wi.pinnedB) {
                        wi.pinnedB = 0;
                        node = wi.n - 1;
                    }
                    if (node < 0) continue;
                    const auto idx = static_cast<std::size_t>(s.rope * kNodesPerSlot + w * kNodesPerWire + node);
                    const auto nb = static_cast<std::size_t>(s.rope * kNodesPerSlot + w * kNodesPerWire + (node == 0 ? 1 : node - 1));
                    invMass_[idx] = invMass_[nb] > 0 ? invMass_[nb] * 2.0 : 1.0;
                    if (found < 2) {
                        ropeEnd[found] = s.rope;
                        nodeEnd[found] = w * kNodesPerWire + node;
                        ++found;
                    }
                }
            }
            if (found == 2)
                for (Join& j : joins_)
                    if (!j.active) {
                        j = Join{ropeEnd[0], nodeEnd[0], ropeEnd[1], nodeEnd[1], static_cast<int>(pi), slot, true};
                        break;
                    }
        }
    }
}

void World::checkRopeSleep(Real dt) {
    const Real h = dt;
    for (int k = 0; k < awakeCount_;) {
        const int si = awake_[static_cast<std::size_t>(k)];
        Span& s = spans_[static_cast<std::size_t>(si)];
        const Rope& r = ropes_[static_cast<std::size_t>(s.rope)];
        Real vmax = 0;
        for (int w = 0; w < s.wires; ++w) {
            const auto base = static_cast<std::size_t>(s.rope * kNodesPerSlot + w * kNodesPerWire);
            for (int i = 0; i < r.wire[static_cast<std::size_t>(w)].n; ++i)
                vmax = std::max(vmax, length(x_[base + static_cast<std::size_t>(i)] - prev_[base + static_cast<std::size_t>(i)]) / h);
        }
        bool polesQuiet = true;
        for (int pi : {s.a, s.b}) {
            if (pi < 0) continue;
            const Pole& p = poles_[static_cast<std::size_t>(pi)];
            if (p.body >= 0 && !bodies_[static_cast<std::size_t>(p.body)].asleep) polesQuiet = false;
        }
        const bool quietLine = time() - lastStructural_ > params.sim.quietTime;
        if ((vmax < 3.0 * params.sim.sleepSpeed || (quietLine && vmax < 0.5)) && polesQuiet)
            s.still += h;
        else
            s.still = 0;
        if (s.still < params.sim.sleepTime) {
            ++k;
            continue;
        }
        // Settled. Back to the catenary if nothing changed, else freeze.
        bool pristine = true;
        for (int w = 0; w < s.wires; ++w) {
            const Wire& wi = r.wire[static_cast<std::size_t>(w)];
            pristine = pristine && wi.broken == 0 && wi.pinnedA && wi.pinnedB;
            // A span still carrying a neighbour's pull keeps it (frozen): going
            // back to the rest catenary would hand its poles a load step.
            const Real rest = wireType(s, w).tension;
            pristine = pristine && std::abs(wi.tension - rest) < 0.2 * rest;
        }
        for (int pi : {s.a, s.b}) {
            if (pi < 0) continue;
            const Pole& p = poles_[static_cast<std::size_t>(pi)];
            pristine = pristine && p.body < 0 && p.breakHeight == 0 && p.state != PoleState::Fallen;
        }
        sleepSpan(si, pristine);
        if (!pristine) checkLanes(-1, si);
        // awake_[k] now holds another span: do not advance.
    }
}

void World::hazards(Real dt) {
    // Lanes: anything below the clearance over a lane blocks it (reported once per lane).
    if (!lanes_.empty()) {
        for (int k = 0; k < awakeCount_; ++k) checkLanes(-1, awake_[static_cast<std::size_t>(k)]);
        for (const Body& b : bodies_)
            if (b.active && !b.asleep && b.kind == 0) checkLanes(b.pole, -1);
    }
    // Live conductors on the ground: one event when they land, arcs while live.
    for (std::size_t ri = 0; ri < ropes_.size(); ++ri) {
        Rope& r = ropes_[ri];
        if (!r.used) continue;
        const auto si = static_cast<std::size_t>(r.span);
        Span& s = spans_[si];
        for (int w = 0; w < s.wires; ++w) {
            Wire& wi = r.wire[static_cast<std::size_t>(w)];
            if (wi.kind != 0 && wi.kind != 1 && wi.kind != 3) continue;
            const WireType& t = wireType(s, w);
            bool live = wi.kind == 0;
            if (wi.kind != 0 && s.a >= 0) live = poles_[static_cast<std::size_t>(s.a)].powered;
            if (wi.kind == 0 && params.sim.mvTripTime > 0 && wi.groundTime > params.sim.mvTripTime) live = false;
            const auto base = static_cast<std::size_t>(s.rope * kNodesPerSlot + w * kNodesPerWire);
            int ground = -1, count = 0;
            for (int i = 0; i < wi.n; ++i)
                if (x_[base + static_cast<std::size_t>(i)].z < t.radius + 0.03) {
                    if (ground < 0 || (static_cast<int>(rng_ >> 33) % (count + 1)) == 0) ground = i;
                    ++count;
                }
            if (ground < 0 || !live) continue;
            const Vec3 at = x_[base + static_cast<std::size_t>(ground)];
            wi.groundTime += dt;
            if (!wi.onGroundReported) {
                wi.onGroundReported = true;
                emit(EventType::WireOnGroundLive, s.a, static_cast<int>(si), w, at, t.voltage,
                     static_cast<std::int64_t>(params.sim.hazardRadius * 1000.0));
                wi.arcTimer = 0.05;
            }
            if (wi.kind != 0) continue;  // low voltage on the ground: a hazard, no arcing
            wi.arcTimer -= dt;
            if (wi.arcTimer <= 0) {
                rng_ = rng_ * 6364136223846793005ull + 1442695040888963407ull;
                const Real u = static_cast<Real>(rng_ >> 11) * (1.0 / 9007199254740992.0);
                emit(EventType::Arc, s.a, static_cast<int>(si), w, at, t.voltage, 0);
                wi.arcTimer = params.sim.arcInterval * (0.3 + 1.4 * u);
            }
        }
    }
}

}  // namespace pd
