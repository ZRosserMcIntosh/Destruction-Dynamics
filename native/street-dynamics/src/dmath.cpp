// Street Dynamics — destructible street furniture. Vendored from Ballistic Dynamics (same author, MIT).
// SPDX-License-Identifier: MIT
//
// Deterministic elementary functions (see bd/dmath.hpp). Coefficients from
// fdlibm 5.3 (Sun Microsystems, 1993: "Permission to use, copy, modify, and
// distribute this software is freely granted, provided that this notice is
// preserved"), as carried by FreeBSD msun.
#include "sd/dmath.hpp"

#include <cstring>
#include <limits>

namespace sd {

std::uint64_t bitsOf(Real v) {
    std::uint64_t u;
    std::memcpy(&u, &v, sizeof u);
    return u;
}

namespace dm {

namespace {

constexpr Real kNaN = std::numeric_limits<Real>::quiet_NaN();
constexpr Real kInf = std::numeric_limits<Real>::infinity();

// Round to the nearest integer, halves away from zero, as a double. Exact for
// |v| < 2^52.
Real roundHalfAway(Real v) {
    const Real a = v < 0 ? -v : v;
    Real r = static_cast<Real>(static_cast<std::int64_t>(a + 0.5));
    return v < 0 ? -r : r;
}

// ---- exp (e_exp.c) ----
constexpr Real kLn2Hi = 6.93147180369123816490e-01;
constexpr Real kLn2Lo = 1.90821492927058770002e-10;
constexpr Real kInvLn2 = 1.44269504088896338700e+00;
constexpr Real kP1 = 1.66666666666666019037e-01;
constexpr Real kP2 = -2.77777777770155933842e-03;
constexpr Real kP3 = 6.61375632143793436117e-05;
constexpr Real kP4 = -1.65339022054652515390e-06;
constexpr Real kP5 = 4.13813679705723846039e-08;

// ---- log (e_log.c) ----
constexpr Real kLg1 = 6.666666666666735130e-01;
constexpr Real kLg2 = 3.999999999940941908e-01;
constexpr Real kLg3 = 2.857142874366239149e-01;
constexpr Real kLg4 = 2.222219843214978396e-01;
constexpr Real kLg5 = 1.818357216161805012e-01;
constexpr Real kLg6 = 1.531383769920937332e-01;
constexpr Real kLg7 = 1.479819860511658591e-01;
constexpr Real kSqrtHalf = 0.70710678118654752440;

// ---- sin / cos kernels (k_sin.c, k_cos.c) on [-pi/4, pi/4] ----
constexpr Real kS1 = -1.66666666666666324348e-01;
constexpr Real kS2 = 8.33333333332248946124e-03;
constexpr Real kS3 = -1.98412698298579493134e-04;
constexpr Real kS4 = 2.75573137070700676789e-06;
constexpr Real kS5 = -2.50507602534068634195e-08;
constexpr Real kS6 = 1.58969099521155010221e-10;
constexpr Real kC1 = 4.16666666666666019037e-02;
constexpr Real kC2 = -1.38888888888741095749e-03;
constexpr Real kC3 = 2.48015872894767294178e-05;
constexpr Real kC4 = -2.75573143513906633035e-07;
constexpr Real kC5 = 2.08757232129817482790e-09;
constexpr Real kC6 = -1.13596475577881948265e-11;
// pi/2 in three 33-bit pieces (e_rem_pio2.c).
constexpr Real kInvPio2 = 6.36619772367581382433e-01;
constexpr Real kPio2_1 = 1.57079632673412561417e+00;
constexpr Real kPio2_2 = 6.07710050630396597660e-11;
constexpr Real kPio2_2t = 2.02226624879595063154e-21;

Real kernelSin(Real x) {
    const Real z = x * x;
    const Real v = z * x;
    const Real r = kS2 + z * (kS3 + z * (kS4 + z * (kS5 + z * kS6)));
    return x + v * (kS1 + z * r);
}

Real kernelCos(Real x) {
    const Real z = x * x;
    const Real w = z * z;
    const Real r = z * (kC1 + z * (kC2 + z * kC3)) + w * w * (kC4 + z * (kC5 + z * kC6));
    const Real hz = 0.5 * z;
    const Real one = 1.0 - hz;
    return one + (((1.0 - one) - hz) + z * r);
}

// x = n pi/2 + r, |r| <= ~pi/4; returns n mod 4.
int reduce(Real x, Real& r) {
    const Real n = roundHalfAway(x * kInvPio2);
    r = ((x - n * kPio2_1) - n * kPio2_2) - n * kPio2_2t;
    const std::int64_t q = static_cast<std::int64_t>(n);
    return static_cast<int>(q & 3);
}

// ---- atan (s_atan.c) ----
constexpr Real kAtanHi[4] = {4.63647609000806093515e-01, 7.85398163397448278999e-01, 9.82793723247329054082e-01,
                             1.57079632679489655800e+00};
constexpr Real kAtanLo[4] = {2.26987774529616870924e-17, 3.06161699786838301793e-17, 1.39033110312309984516e-17,
                             6.12323399573676603587e-17};
constexpr Real kAT[11] = {3.33333333333329318027e-01,  -1.99999999998764832476e-01, 1.42857142725034663711e-01,
                          -1.11111104054623557880e-01, 9.09088713343650656196e-02,  -7.69187620504482999495e-02,
                          6.66107313738753120669e-02,  -5.83357013379057348645e-02, 4.97687799461593236017e-02,
                          -3.65315727442169155270e-02, 1.62858201153657823623e-02};
constexpr Real kPiLo = 1.2246467991473531772e-16;  // pi - double(pi)

}  // namespace

Real exp(Real x) {
    if (x != x) return x;
    if (x > 7.09782712893383973096e+02) return kInf;
    if (x < -7.45133219101941108420e+02) return 0.0;
    Real hi = 0, lo = 0;
    int k = 0;
    const Real ax = x < 0 ? -x : x;
    if (ax > 0.5 * 0.6931471805599453) {
        k = static_cast<int>(kInvLn2 * x + (x < 0 ? -0.5 : 0.5));
        hi = x - k * kLn2Hi;
        lo = k * kLn2Lo;
        x = hi - lo;
    } else if (ax < 3.725290298461914e-09) {  // 2^-28
        return 1.0 + x;
    }
    const Real t = x * x;
    const Real c = x - t * (kP1 + t * (kP2 + t * (kP3 + t * (kP4 + t * kP5))));
    if (k == 0) return 1.0 - ((x * c) / (c - 2.0) - x);
    const Real y = 1.0 - ((lo - (x * c) / (2.0 - c)) - hi);
    return std::ldexp(y, k);  // exact scaling
}

Real log(Real x) {
    if (x != x) return x;
    if (x < 0) return kNaN;
    if (x == 0) return -kInf;
    if (x == kInf) return x;
    int e = 0;
    Real m = std::frexp(x, &e);  // exact: x = m 2^e, m in [0.5, 1)
    if (m < kSqrtHalf) {
        m *= 2.0;
        e -= 1;
    }
    const Real f = m - 1.0;  // exact (Sterbenz)
    const Real s = f / (2.0 + f);
    const Real z = s * s;
    const Real w = z * z;
    const Real t1 = w * (kLg2 + w * (kLg4 + w * kLg6));
    const Real t2 = z * (kLg1 + w * (kLg3 + w * (kLg5 + w * kLg7)));
    const Real R = t2 + t1;
    const Real hfsq = 0.5 * f * f;
    const Real dk = e;
    return dk * kLn2Hi - ((hfsq - (s * (hfsq + R) + dk * kLn2Lo)) - f);
}

void sincos(Real x, Real& s, Real& c) {
    if (!(x - x == 0)) {  // inf or nan
        s = c = kNaN;
        return;
    }
    const Real ax = x < 0 ? -x : x;
    if (ax <= 0.7853981633974483) {
        s = kernelSin(x);
        c = kernelCos(x);
        return;
    }
    Real r;
    const int q = reduce(x, r);
    const Real ks = kernelSin(r), kc = kernelCos(r);
    switch (q) {
        case 0: s = ks; c = kc; break;
        case 1: s = kc; c = -ks; break;
        case 2: s = -ks; c = -kc; break;
        default: s = -kc; c = ks; break;
    }
}

Real sin(Real x) {
    Real s, c;
    sincos(x, s, c);
    return s;
}

Real cos(Real x) {
    Real s, c;
    sincos(x, s, c);
    return c;
}

Real tan(Real x) {
    Real s, c;
    sincos(x, s, c);
    return s / c;
}

Real atan(Real x) {
    if (x != x) return x;
    const bool neg = x < 0;
    Real ax = neg ? -x : x;
    if (ax >= 7.3786976294838206464e19) {  // 2^66
        const Real r = kAtanHi[3] + kAtanLo[3];
        return neg ? -r : r;
    }
    int id;
    if (ax < 0.4375) {
        if (ax < 7.450580596923828e-09) return x;  // 2^-27
        id = -1;
    } else if (ax < 0.6875) {
        id = 0;
        ax = (2.0 * ax - 1.0) / (2.0 + ax);
    } else if (ax < 1.1875) {
        id = 1;
        ax = (ax - 1.0) / (ax + 1.0);
    } else if (ax < 2.4375) {
        id = 2;
        ax = (ax - 1.5) / (1.0 + 1.5 * ax);
    } else {
        id = 3;
        ax = -1.0 / ax;
    }
    const Real z = ax * ax;
    const Real w = z * z;
    const Real s1 = z * (kAT[0] + w * (kAT[2] + w * (kAT[4] + w * (kAT[6] + w * (kAT[8] + w * kAT[10])))));
    const Real s2 = w * (kAT[1] + w * (kAT[3] + w * (kAT[5] + w * (kAT[7] + w * kAT[9]))));
    Real r;
    if (id < 0) {
        r = ax - ax * (s1 + s2);
    } else {
        r = kAtanHi[id] - ((ax * (s1 + s2) - kAtanLo[id]) - ax);
    }
    return neg ? -r : r;
}

Real atan2(Real y, Real x) {
    if (x != x || y != y) return x + y;
    if (y == 0) {
        if (std::signbit(x)) return std::signbit(y) ? -kPi : kPi;
        return y;  // +-0
    }
    if (x == 0) return y > 0 ? 0.5 * kPi : -0.5 * kPi;
    const Real ay = y < 0 ? -y : y, ax = x < 0 ? -x : x;
    if (ax == kInf || ay == kInf) {
        if (ax == kInf && ay == kInf) {
            const Real a = x > 0 ? 0.25 * kPi : 0.75 * kPi;
            return y > 0 ? a : -a;
        }
        if (ax == kInf) return x > 0 ? (y > 0 ? 0.0 : -0.0) : (y > 0 ? kPi : -kPi);
        return y > 0 ? 0.5 * kPi : -0.5 * kPi;
    }
    const Real a = atan(ay / ax);
    Real r;
    if (x > 0) {
        r = a;
    } else {
        r = (kPi - (a - kPiLo));
    }
    return y < 0 ? -r : r;
}

Real asin(Real x) {
    if (x > 1 || x < -1) return kNaN;
    return atan2(x, std::sqrt((1.0 - x) * (1.0 + x)));
}

Real acos(Real x) {
    if (x > 1 || x < -1) return kNaN;
    return atan2(std::sqrt((1.0 - x) * (1.0 + x)), x);
}

Real pow(Real x, Real y) {
    if (y == 0) return 1.0;
    if (x == 0) return y > 0 ? 0.0 : kInf;
    if (x < 0) return kNaN;
    if (x == 1) return 1.0;
    return exp(y * log(x));
}

Real cbrt(Real x) {
    if (x == 0 || x != x) return x;
    const bool neg = x < 0;
    const Real a = neg ? -x : x;
    Real y = pow(a, 1.0 / 3.0);
    y = y - (y * y * y - a) / (3.0 * y * y);  // one Newton step: ~0.5 ulp
    return neg ? -y : y;
}

Real tanh(Real x) {
    const Real ax = x < 0 ? -x : x;
    if (ax > 22) return x < 0 ? -1.0 : 1.0;
    if (ax < 0.02) {
        const Real z = x * x;
        return x + x * z * (-1.0 / 3 + z * (2.0 / 15 + z * (-17.0 / 315 + z * (62.0 / 2835))));
    }
    const Real e = exp(2.0 * ax);
    const Real r = (e - 1.0) / (e + 1.0);
    return x < 0 ? -r : r;
}

}  // namespace dm
}  // namespace sd
