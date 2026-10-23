// GLSL shared by the ocean, sky and underwater passes.

export const NOISE_GLSL = /* glsl */ `
float ddHash(vec2 p) { return fract(sin(dot(p, vec2(127.1, 311.7))) * 43758.5453123); }
float ddNoise(vec2 p) {
  vec2 i = floor(p), f = fract(p);
  vec2 u = f * f * (3.0 - 2.0 * f);
  return mix(mix(ddHash(i), ddHash(i + vec2(1, 0)), u.x), mix(ddHash(i + vec2(0, 1)), ddHash(i + vec2(1, 1)), u.x), u.y);
}
float ddFbm(vec2 p) {
  float s = 0.0, a = 0.5;
  for (int i = 0; i < 4; i++) { s += a * ddNoise(p); p *= 2.03; a *= 0.5; }
  return s;
}
// Cheap animated caustic pattern (two warped cell layers).
float ddCaustic(vec2 p, float t) {
  vec2 q = p * 0.18;
  float a = abs(sin(q.x * 3.1 + sin(q.y * 2.3 + t * 0.9) * 1.7 + t * 0.6));
  float b = abs(sin(q.y * 2.9 + sin(q.x * 2.1 - t * 0.7) * 1.9 - t * 0.5));
  return pow(1.0 - min(a, b), 6.0);
}
`;

export const SKY_GLSL = /* glsl */ `
uniform vec3 uSkyTop;
uniform vec3 uSkyHorizon;
uniform vec3 uSunDir;
uniform vec3 uSunColor;
vec3 ddSky(vec3 dir) {
  float t = clamp(dir.y, 0.0, 1.0);
  vec3 c = mix(uSkyHorizon, uSkyTop, pow(t, 0.55));
  float s = max(dot(normalize(dir), uSunDir), 0.0);
  c += uSunColor * (pow(s, 900.0) * 18.0 + pow(s, 12.0) * 0.25);
  return c;
}
`;

export const FOG_GLSL = /* glsl */ `
uniform vec3 uFogColor;
uniform float uFogNear;
uniform float uFogFar;
uniform float uFogDensity;
float ddFog(float d) {
  return uFogDensity > 0.0 ? 1.0 - exp(-uFogDensity * uFogDensity * d * d) : smoothstep(uFogNear, uFogFar, d);
}
`;
