// Pole Dynamics — utility poles and overhead wires. Vendored from Ballistic Dynamics (same author, MIT).
// SPDX-License-Identifier: MIT
//
// Deterministic elementary functions. The C library's exp, log, sin, cos,
// atan and pow differ between libms (glibc, Apple, MSVC, wasi-libc) in the
// last bit, which is enough to make a 1,000 m trajectory differ bit for bit
// between machines. These are built from +, -, *, / and exact scaling only,
// so they return the same bits everywhere that IEEE 754 doubles round to
// nearest and the compiler does not fuse multiply-adds (ADR 0004: the build
// sets -ffp-contract=off).
//
// The algorithms and minimax coefficients are those of FreeBSD's msun /
// Sun's fdlibm (e_exp.c, e_log.c, k_sin.c, k_cos.c, s_atan.c): accurate to
// about 1 ulp over the ranges the engine uses. The trigonometric reduction is
// Cody-Waite in three parts, exact for |x| < 2^20 * pi/2; beyond that the
// result is still deterministic but loses accuracy (no ballistic angle gets
// there).
#pragma once

#include "pd/math.hpp"

namespace pd::dm {

Real exp(Real x);
Real log(Real x);
Real sin(Real x);
Real cos(Real x);
void sincos(Real x, Real& s, Real& c);
Real tan(Real x);
Real atan(Real x);
Real atan2(Real y, Real x);
Real asin(Real x);
Real acos(Real x);
// x^y for x > 0 (exp(y log x)); x == 0 gives 0 for y > 0.
Real pow(Real x, Real y);
Real cbrt(Real x);
Real tanh(Real x);

}  // namespace pd::dm
