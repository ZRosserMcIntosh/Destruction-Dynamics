// Procedural placeholder sound design on the Web Audio API. Every layer is
// synthesised from noise and oscillators at runtime: no recorded assets, no
// licensing questions. It is meant to be replaced by licensed recordings (or
// a game's audio middleware) behind the same AudioEngine interface.
//
// Spatialisation: HRTF PannerNodes in the scene frame (metres), inverse
// distance roll-off, and propagation delay at the speed of sound so a distant
// collapse is seen before it is heard. Underwater, the master bus is low-passed.

import { SeededRandom, distance } from '@destruction-dynamics/core';
import type { AudioEngine, ListenerState, SoundCue, SoundType } from './types';

const SPEED_OF_SOUND = 343;

/** Reference distance per layer: big sources stay loud far away. */
const REF_DISTANCE: Record<SoundType, number> = {
  groan: 120,
  metal: 80,
  glass: 40,
  rumble: 200,
  debris: 25,
  'water-impact': 260,
  splash: 20,
  echo: 400,
  bubbles: 15,
  'cable-snap': 120,
  'cable-hum': 60,
};

export class ProceduralAudioEngine implements AudioEngine {
  private ctx: AudioContext | null = null;
  private master: GainNode | null = null;
  private underwaterFilter: BiquadFilterNode | null = null;
  private noise: AudioBuffer | null = null;
  private brown: AudioBuffer | null = null;
  private readonly voices = new Set<AudioScheduledSourceNode>();
  private listenerPos = { x: 0, y: 0, z: 0 };
  private underwater = false;
  private surf: GainNode | null = null;

  constructor(private readonly volume = 0.8) {}

  get enabled(): boolean {
    return this.ctx !== null && this.ctx.state === 'running';
  }

  get activeVoices(): number {
    return this.voices.size;
  }

  async start(): Promise<void> {
    if (!this.ctx) {
      const Ctor = globalThis.AudioContext;
      if (!Ctor) return;
      const ctx = new Ctor();
      this.ctx = ctx;
      this.master = ctx.createGain();
      this.master.gain.value = this.volume;
      this.underwaterFilter = ctx.createBiquadFilter();
      this.underwaterFilter.type = 'lowpass';
      this.underwaterFilter.frequency.value = 20000;
      const comp = ctx.createDynamicsCompressor();
      comp.threshold.value = -14;
      comp.ratio.value = 6;
      this.master.connect(this.underwaterFilter).connect(comp).connect(ctx.destination);
      this.noise = this.makeNoise(false);
      this.brown = this.makeNoise(true);
      this.startSurf();
    }
    if (this.ctx.state !== 'running') await this.ctx.resume();
  }

  private makeNoise(brown: boolean): AudioBuffer {
    const ctx = this.ctx!;
    const len = ctx.sampleRate * 4;
    const buf = ctx.createBuffer(1, len, ctx.sampleRate);
    const d = buf.getChannelData(0);
    const r = new SeededRandom(brown ? 2 : 1);
    let last = 0;
    for (let i = 0; i < len; i++) {
      const w = r.next() * 2 - 1;
      if (brown) {
        last = (last + 0.02 * w) / 1.02;
        d[i] = last * 3.5;
      } else d[i] = w;
    }
    return buf;
  }

  /** Continuous surf along the shoreline, the bed every other layer sits on. */
  private startSurf(): void {
    const ctx = this.ctx!;
    const src = ctx.createBufferSource();
    src.buffer = this.brown;
    src.loop = true;
    const bp = ctx.createBiquadFilter();
    bp.type = 'lowpass';
    bp.frequency.value = 700;
    const g = ctx.createGain();
    g.gain.value = 0.12;
    const lfo = ctx.createOscillator();
    lfo.frequency.value = 0.12;
    const lfoGain = ctx.createGain();
    lfoGain.gain.value = 0.06;
    lfo.connect(lfoGain).connect(g.gain);
    src.connect(bp).connect(g).connect(this.master!);
    src.start();
    lfo.start();
    this.surf = g;
  }

  setListener(l: ListenerState): void {
    const ctx = this.ctx;
    if (!ctx) return;
    this.listenerPos = l.position;
    const L = ctx.listener;
    const t = ctx.currentTime;
    if (L.positionX) {
      L.positionX.setTargetAtTime(l.position.x, t, 0.05);
      L.positionY.setTargetAtTime(l.position.y, t, 0.05);
      L.positionZ.setTargetAtTime(l.position.z, t, 0.05);
      L.forwardX.setTargetAtTime(l.forward.x, t, 0.05);
      L.forwardY.setTargetAtTime(l.forward.y, t, 0.05);
      L.forwardZ.setTargetAtTime(l.forward.z, t, 0.05);
      L.upX.setTargetAtTime(l.up.x, t, 0.05);
      L.upY.setTargetAtTime(l.up.y, t, 0.05);
      L.upZ.setTargetAtTime(l.up.z, t, 0.05);
    }
    if (l.underwater !== this.underwater) {
      this.underwater = l.underwater;
      this.underwaterFilter!.frequency.setTargetAtTime(l.underwater ? 450 : 20000, t, 0.15);
      this.surf?.gain.setTargetAtTime(l.underwater ? 0.03 : 0.12, t, 0.3);
    }
  }

  stopAll(): void {
    for (const v of this.voices) {
      try {
        v.stop();
      } catch {
        // already stopped
      }
    }
    this.voices.clear();
  }

  play(cue: SoundCue): void {
    const ctx = this.ctx;
    if (!ctx || ctx.state !== 'running' || !this.master) return;
    const out = ctx.createGain();
    out.gain.value = cue.gain;
    let dest: AudioNode = out;
    let delay = 0;
    if (cue.position) {
      const p = ctx.createPanner();
      p.panningModel = 'HRTF';
      p.distanceModel = 'inverse';
      p.refDistance = REF_DISTANCE[cue.type];
      p.rolloffFactor = 1;
      p.maxDistance = 20000;
      p.positionX.value = cue.position.x;
      p.positionY.value = cue.position.y;
      p.positionZ.value = cue.position.z;
      out.connect(p);
      dest = p;
      delay = Math.min(4, distance(cue.position, this.listenerPos) / SPEED_OF_SOUND);
    }
    dest.connect(this.master);
    const r = new SeededRandom(cue.seed ?? 1);
    const t0 = ctx.currentTime + delay + 0.01;
    const dur = cue.duration ?? 2;
    switch (cue.type) {
      case 'groan':
        return this.groan(out, t0, dur, r);
      case 'metal':
        return this.metal(out, t0, dur, r);
      case 'glass':
        return this.glass(out, t0, r);
      case 'rumble':
        return this.rumble(out, t0, dur);
      case 'debris':
        return this.thump(out, t0, 0.4, 500);
      case 'water-impact':
        return this.waterImpact(out, t0);
      case 'splash':
        this.noiseBurst(out, t0, 0.5, 'bandpass', 1500, 0.6);
        return;
      case 'echo':
        [0, 0.7, 1.6].forEach((d, i) => this.thump(out, t0 + d, 2.5, 160 - i * 30, 0.5 / (i + 1)));
        return;
      case 'bubbles':
        return this.bubbles(out, t0, dur, r);
      case 'cable-snap':
        return this.cableSnap(out, t0, r);
      case 'cable-hum':
        this.osc(out, 'sine', 55 + r.range(-5, 5), t0, dur, 0.4, 1.5);
        return;
    }
  }

  private track(n: AudioScheduledSourceNode, stopAt: number): void {
    this.voices.add(n);
    n.onended = () => this.voices.delete(n);
    n.stop(stopAt);
  }

  private env(g: GainNode, t0: number, attack: number, hold: number, release: number, peak = 1): void {
    g.gain.setValueAtTime(0.0001, t0);
    g.gain.exponentialRampToValueAtTime(peak, t0 + attack);
    g.gain.setValueAtTime(peak, t0 + attack + hold);
    g.gain.exponentialRampToValueAtTime(0.0001, t0 + attack + hold + release);
  }

  private osc(
    out: AudioNode,
    type: OscillatorType,
    f: number,
    t0: number,
    dur: number,
    peak: number,
    attack = 0.05,
  ) {
    const ctx = this.ctx!;
    const o = ctx.createOscillator();
    o.type = type;
    o.frequency.value = f;
    const g = ctx.createGain();
    this.env(g, t0, attack, Math.max(0, dur - attack - 0.5), 0.5, peak);
    o.connect(g).connect(out);
    o.start(t0);
    this.track(o, t0 + dur + 0.1);
    return o;
  }

  private noiseBurst(
    out: AudioNode,
    t0: number,
    dur: number,
    type: BiquadFilterType,
    freq: number,
    peak: number,
    brown = false,
  ) {
    const ctx = this.ctx!;
    const s = ctx.createBufferSource();
    s.buffer = brown ? this.brown : this.noise;
    s.loop = true;
    const f = ctx.createBiquadFilter();
    f.type = type;
    f.frequency.value = freq;
    const g = ctx.createGain();
    this.env(g, t0, 0.005, 0, dur, peak);
    s.connect(f).connect(g).connect(out);
    s.start(t0, Math.random() * 3);
    this.track(s, t0 + dur + 0.05);
    return f;
  }

  private thump(out: AudioNode, t0: number, dur: number, freq: number, peak = 1): void {
    this.noiseBurst(out, t0, dur, 'lowpass', freq, peak, true);
    const o = this.osc(out, 'sine', freq / 4, t0, dur, peak * 0.8, 0.01);
    o.frequency.exponentialRampToValueAtTime(Math.max(20, freq / 10), t0 + dur);
  }

  private groan(out: AudioNode, t0: number, dur: number, r: SeededRandom): void {
    const ctx = this.ctx!;
    const lp = ctx.createBiquadFilter();
    lp.type = 'lowpass';
    lp.frequency.value = 260;
    lp.Q.value = 4;
    lp.connect(out);
    for (const detune of [0, 7]) {
      const o = this.osc(lp, 'sawtooth', 38 + r.range(0, 10) + detune, t0, dur, 0.5, 0.8);
      for (let k = 1; k <= 4; k++)
        o.frequency.linearRampToValueAtTime(34 + r.range(0, 22) + detune, t0 + (dur * k) / 4);
    }
  }

  private metal(out: AudioNode, t0: number, dur: number, r: SeededRandom): void {
    const ctx = this.ctx!;
    const bp = ctx.createBiquadFilter();
    bp.type = 'bandpass';
    bp.frequency.value = 900;
    bp.Q.value = 9;
    bp.connect(out);
    const o = this.osc(bp, 'square', 140 + r.range(0, 120), t0, dur, 0.7, 0.2);
    for (let k = 1; k <= 6; k++)
      o.frequency.linearRampToValueAtTime(90 + r.range(0, 260), t0 + (dur * k) / 6);
    for (let k = 0; k < 6; k++) bp.frequency.setValueAtTime(500 + r.range(0, 1800), t0 + (dur * k) / 6);
    this.noiseBurst(out, t0 + dur * 0.3, dur * 0.6, 'highpass', 4000, 0.15);
  }

  private glass(out: AudioNode, t0: number, r: SeededRandom): void {
    for (let i = 0; i < 28; i++) {
      const t = t0 + r.range(0, 1.4) * r.next();
      this.noiseBurst(out, t, r.range(0.04, 0.25), 'highpass', r.range(2500, 7000), r.range(0.15, 0.5));
      if (r.chance(0.4)) this.osc(out, 'sine', r.range(2500, 6500), t, r.range(0.08, 0.3), 0.08, 0.002);
    }
  }

  private rumble(out: AudioNode, t0: number, dur: number): void {
    const f = this.noiseBurst(out, t0, dur, 'lowpass', 140, 1, true);
    f.Q.value = 2;
    this.osc(out, 'sine', 31, t0, dur, 0.6, 1.5);
  }

  private waterImpact(out: AudioNode, t0: number): void {
    const f = this.noiseBurst(out, t0, 5, 'lowpass', 2500, 1.2, true);
    f.frequency.exponentialRampToValueAtTime(180, t0 + 4.5);
    this.thump(out, t0, 2.5, 180, 1.2);
    this.noiseBurst(out, t0 + 0.2, 3.5, 'highpass', 1800, 0.35);
  }

  private bubbles(out: AudioNode, t0: number, dur: number, r: SeededRandom): void {
    const n = Math.floor(dur * 14);
    for (let i = 0; i < n; i++) {
      const t = t0 + r.range(0, dur);
      const o = this.osc(out, 'sine', r.range(250, 700), t, 0.06, r.range(0.05, 0.2), 0.004);
      o.frequency.exponentialRampToValueAtTime(r.range(800, 1600), t + 0.05);
    }
  }

  private cableSnap(out: AudioNode, t0: number, r: SeededRandom): void {
    this.noiseBurst(out, t0, 0.15, 'highpass', 3000, 1);
    const o = this.osc(out, 'sawtooth', r.range(300, 500), t0, 2.5, 0.4, 0.002);
    o.frequency.exponentialRampToValueAtTime(r.range(40, 70), t0 + 2.4);
  }
}
