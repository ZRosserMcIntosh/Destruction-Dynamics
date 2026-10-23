// Authored large-scale impact waves. The near-field solver supplies physically
// inspired local behaviour; for a landmark impact we also want a guaranteed,
// art-directable surge. Each surge is a wave packet expanding from a line
// segment (the long axis of the impact footprint), so a long body produces a
// stadium-shaped front that is strongest broadside and in the direction of fall.

export interface AuthoredSurge {
  /** Segment the front expands from (scene x/z). */
  ax: number;
  az: number;
  bx: number;
  bz: number;
  startTime: number;
  /** Front speed, m/s. */
  speed: number;
  /** Peak crest height at the source, m. */
  amplitude: number;
  /** Packet half-width, m. */
  width: number;
  /** Distance over which amplitude halves (geometric spreading). */
  spreadDistance: number;
  /** Seconds for amplitude to fall by e. */
  lifetime: number;
  /** Direction of fall (unit x/z) and how strongly it biases the front (0..1). */
  dirX: number;
  dirZ: number;
  directionality: number;
}

/** Distance from (px, pz) to the segment, and the unit direction away from it. */
function segmentDistance(s: AuthoredSurge, px: number, pz: number) {
  const ex = s.bx - s.ax;
  const ez = s.bz - s.az;
  const len2 = ex * ex + ez * ez;
  const t = len2 > 0 ? Math.max(0, Math.min(1, ((px - s.ax) * ex + (pz - s.az) * ez) / len2)) : 0;
  const dx = px - (s.ax + ex * t);
  const dz = pz - (s.az + ez * t);
  const d = Math.sqrt(dx * dx + dz * dz);
  return { d, nx: d > 1e-6 ? dx / d : 0, nz: d > 1e-6 ? dz / d : 0 };
}

export function surgeHeight(s: AuthoredSurge, x: number, z: number, t: number): number {
  const age = t - s.startTime;
  if (age <= 0) return 0;
  const { d, nx, nz } = segmentDistance(s, x, z);
  const front = s.speed * age;
  const q = (d - front) / s.width;
  if (q < -4 || q > 3) return 0;
  // Ricker-like profile: a crest with a trough behind it (water drawn back).
  const profile = (1 - 1.6 * q * q) * Math.exp(-q * q);
  const lobe =
    1 - s.directionality + s.directionality * Math.max(0.25, 0.5 + 0.5 * (nx * s.dirX + nz * s.dirZ));
  const spread = 1 / Math.sqrt(1 + d / s.spreadDistance);
  const decay = Math.exp(-age / s.lifetime);
  // Ramp in over the first second so the packet grows from the impact.
  const ramp = Math.min(1, age);
  return s.amplitude * profile * lobe * spread * decay * ramp;
}

/** Radial current under a surge (linear-wave estimate u ≈ c η / D). */
export function surgeVelocity(s: AuthoredSurge, x: number, z: number, t: number, depth: number) {
  const h = surgeHeight(s, x, z, t);
  if (h === 0) return { x: 0, z: 0 };
  const { nx, nz } = segmentDistance(s, x, z);
  const v = (s.speed * h) / Math.max(4, depth);
  return { x: nx * v, z: nz * v };
}
