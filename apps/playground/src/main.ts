import './style.css';
import { AudioCueRouter, ProceduralAudioEngine } from '@destruction-dynamics/audio';
import { QUALITY_PRESET_IDS, type QualityPresetId } from '@destruction-dynamics/core';
import { stageWindows } from '@destruction-dynamics/events';
import { CAMERA_MODES, type CameraMode, CameraRig } from './cameras';
import { SceneView } from './sceneView';
import { DeepEndSimulation, type PlaygroundSave } from './simulation';

const $ = <T extends HTMLElement>(id: string) => document.getElementById(id) as T;
const errors: string[] = [];
const banner = $('banner');
const reportError = (msg: string) => {
  errors.push(msg);
  banner.hidden = false;
  banner.textContent = `Error: ${msg}`;
};
window.addEventListener('error', (e) => reportError(e.message));
window.addEventListener('unhandledrejection', (e) => reportError(String(e.reason)));

const params = new URLSearchParams(location.search);
const initialQuality = (params.get('quality') as QualityPresetId) ?? 'balanced';

async function start(): Promise<void> {
  const seed = Number($<HTMLInputElement>('seed').value) || 1234;
  const sim = new DeepEndSimulation(
    QUALITY_PRESET_IDS.includes(initialQuality) ? initialQuality : 'balanced',
    seed,
  );
  await sim.init();
  const view = new SceneView($('viewport'), sim);
  const rig = new CameraRig(view.camera, view.renderer.domElement, sim);
  const audio = new ProceduralAudioEngine();
  let audioEnabled = true;
  const router = new AudioCueRouter(audio, { defaultPosition: () => ({ x: 20, y: 120, z: 0 }) });
  const d = sim.director;

  d.on('cue', (c) => {
    if (c.cue.channel === 'camera') rig.cue(c.cue.type, c.cue.params ?? {});
    if (c.cue.channel === 'audio' && audioEnabled) {
      // Impacts sound from the water, structural layers from the break.
      const pos =
        c.cue.type === 'water-impact' || c.cue.type === 'echo'
          ? { x: 190, y: 0, z: 0 }
          : { x: 0, y: 40 + Math.min(200, sim.stats().tiltDeg * 2), z: 0 };
      router.handle({ ...c, cue: { ...c.cue, params: { ...c.cue.params, position: pos } } });
    }
  });
  d.on('state', (s) => {
    if (s.to === 'INTACT') {
      rig.resetShots();
      audio.stopAll();
    }
  });
  d.on('seek', () => audio.stopAll());

  // -- UI ---------------------------------------------------------------------
  const timeline = $<HTMLInputElement>('timeline');
  const windows = stageWindows(sim.def);
  const tMin = -sim.def.preparation.duration;
  const tMax = sim.timeline.settleEnd + 20;
  timeline.min = String(tMin);
  timeline.max = String(tMax);
  const markers = $('stage-markers');
  for (const w of windows) {
    const s = document.createElement('span');
    s.style.left = `${((w.start - tMin) / (tMax - tMin)) * 100}%`;
    s.title = w.id;
    markers.appendChild(s);
  }
  let scrubbing = false;
  timeline.addEventListener('input', () => {
    scrubbing = true;
    d.seek(Number(timeline.value));
  });
  timeline.addEventListener('change', () => (scrubbing = false));

  const ensureAudio = () => {
    if (audioEnabled) void audio.start().catch((e) => console.warn('audio unavailable', e));
  };
  const safely = (fn: () => void) => () => {
    try {
      fn();
    } catch (e) {
      console.warn(e);
    }
  };
  $('btn-play').addEventListener(
    'click',
    safely(() => {
      ensureAudio();
      if (d.state !== 'INTACT') d.reset();
      d.arm();
      if (d.paused) d.resume();
    }),
  );
  $('btn-trigger').addEventListener(
    'click',
    safely(() => {
      ensureAudio();
      if (d.state === 'INTACT' || d.state === 'PREPARING') d.trigger();
      else d.restart();
      if (d.paused) d.resume();
    }),
  );
  $('btn-cancel').addEventListener(
    'click',
    safely(() => d.cancel()),
  );
  const pauseBtn = $('btn-pause');
  pauseBtn.addEventListener('click', () => {
    if (d.paused) d.resume();
    else d.pause();
  });
  $('btn-reset').addEventListener('click', () => d.reset());
  $('btn-restore').addEventListener(
    'click',
    safely(() => d.restoreNow()),
  );
  $<HTMLSelectElement>('speed').addEventListener('change', (e) =>
    d.setTimeScale(Number((e.target as HTMLSelectElement).value)),
  );
  const qualitySel = $<HTMLSelectElement>('quality');
  qualitySel.value = sim.quality.id;
  qualitySel.addEventListener('change', () => {
    sim.setQuality(qualitySel.value as QualityPresetId);
    view.applyQuality(sim.quality);
  });
  const cameraSel = $<HTMLSelectElement>('camera');
  cameraSel.addEventListener('change', () => rig.setMode(cameraSel.value as CameraMode));
  $<HTMLInputElement>('seed').addEventListener('change', (e) => {
    const v = Number((e.target as HTMLInputElement).value);
    if (Number.isFinite(v)) sim.setSeed(v);
  });
  $<HTMLInputElement>('toggle-physics').addEventListener(
    'change',
    (e) => (view.debug.physics = (e.target as HTMLInputElement).checked),
  );
  $<HTMLInputElement>('toggle-water').addEventListener(
    'change',
    (e) => (view.debug.water = (e.target as HTMLInputElement).checked),
  );
  $<HTMLInputElement>('toggle-collision').addEventListener(
    'change',
    (e) => (view.debug.collision = (e.target as HTMLInputElement).checked),
  );
  $<HTMLInputElement>('toggle-audio').addEventListener('change', (e) => {
    audioEnabled = (e.target as HTMLInputElement).checked;
    if (!audioEnabled) audio.stopAll();
  });
  let memorySave: PlaygroundSave | null = null;
  $('btn-save').addEventListener('click', () => {
    memorySave = sim.save();
    try {
      localStorage.setItem('dd-playground-save', JSON.stringify(memorySave));
    } catch {
      // storage unavailable; the in-memory copy still works
    }
  });
  $('btn-load').addEventListener(
    'click',
    safely(() => {
      let s = memorySave;
      try {
        const raw = localStorage.getItem('dd-playground-save');
        if (raw) s = JSON.parse(raw) as PlaygroundSave;
      } catch {
        // ignore
      }
      if (s) sim.load(s);
    }),
  );
  window.addEventListener('resize', () => view.resize());

  // -- loop -------------------------------------------------------------------
  const stats = $('stats');
  const webgpu = 'gpu' in navigator;
  let last = performance.now();
  let frames = 0;
  let fps = 0;
  let fpsFrames = 0;
  let fpsT = last;
  let hudT = 0;
  let lastFrameMs = 16;

  const loop = (now: number) => {
    const frameMs = now - last;
    last = now;
    const dt = Math.min(0.1, Math.max(0, frameMs / 1000));
    try {
      sim.update(dt, frames > 30 ? frameMs : undefined);
      rig.update(dt);
      const t0 = performance.now();
      view.render();
      sim.profiler.add('render', performance.now() - t0);
      sim.profiler.endFrame(frameMs);
      if (audioEnabled && audio.enabled) {
        const fwd = view.camera.getWorldDirection(view.camera.position.clone());
        audio.setListener({
          position: view.camera.position,
          forward: { x: fwd.x, y: fwd.y, z: fwd.z },
          up: { x: 0, y: 1, z: 0 },
          underwater: view.underwaterNow,
        });
      }
    } catch (e) {
      reportError(e instanceof Error ? `${e.message}\n${e.stack ?? ''}` : String(e));
    }
    frames++;
    fpsFrames++;
    lastFrameMs = frameMs;
    if (now - fpsT > 500) {
      fps = (fpsFrames * 1000) / (now - fpsT);
      fpsFrames = 0;
      fpsT = now;
    }
    if (now - hudT > 250) {
      hudT = now;
      if (!scrubbing) timeline.value = String(d.state === 'INTACT' ? tMin : Math.min(tMax, d.time));
      $('timeline-label').textContent = d.state === 'INTACT' ? 'intact' : `${d.time.toFixed(1)} s`;
      pauseBtn.textContent = d.paused ? 'Resume' : 'Pause';
      const s = sim.stats();
      const i = view.info;
      const mem = (performance as unknown as { memory?: { usedJSHeapSize: number } }).memory;
      const c = s.costs;
      stats.textContent = [
        `${s.state}${s.stage ? ` · ${s.stage}` : ''}${s.paused ? ' · paused' : ''}`,
        `t ${s.time.toFixed(2)} s · ×${s.timeScale} · tilt ${s.tiltDeg.toFixed(1)}°`,
        `fps ${fps.toFixed(0)} · frame ${lastFrameMs.toFixed(1)} ms · degrade L${s.degradation}`,
        `cpu ms: dir ${(c.director ?? 0).toFixed(2)} phys ${(c.physics ?? 0).toFixed(2)} fluid ${(c.fluid ?? 0).toFixed(2)}`,
        `        fx ${(c.particles ?? 0).toFixed(2)} render ${(c.render ?? 0).toFixed(2)}`,
        `bodies dyn ${s.bodies.dynamic} (sleep ${s.bodies.sleeping}) kin ${s.bodies.kinematic} static ${s.bodies.static}`,
        `debris ${s.debris.active}/${s.debris.limit} · refused ${s.debris.refused}`,
        `particles ${s.particles}/${s.particleLimit}`,
        `fluid ${s.fluid.grid}² · substeps ${s.fluid.substeps} · wave ${s.fluid.maxDisturbance.toFixed(1)} m (peak ${s.fluid.peak.toFixed(1)})`,
        `collision ${s.collisionMode} · nav v${s.navigationVersion} · tiles ${s.activeTiles.length}`,
        `structure ${s.layers.count} layers · ${
          s.layers.status === 'authored'
            ? 'authored wreck'
            : s.layers.status === 'computing'
              ? 'solving in worker…'
              : `simulated (${(s.layers.computeMs ?? 0).toFixed(0)} ms, ${s.layers.jointsBroken ?? 0}/${s.layers.joints ?? 0} joints broke)`
        }`,
        `draws ${i.calls} · tris ${(i.triangles / 1000).toFixed(0)}k · px ${i.pixelRatio.toFixed(2)}`,
        `camera ${rig.mode}${rig.mode === 'cinematic' ? ` (${rig.shotName})` : ''}${i.underwater ? ' · underwater' : ''}`,
        mem
          ? `js heap ${(mem.usedJSHeapSize / 1048576).toFixed(0)} MB`
          : 'js heap n/a (browser does not expose it)',
        `renderer WebGL2${webgpu ? ' (WebGPU available; not used yet)' : ''} · physics ${s.physicsBackend}`,
        ...sim.cueLog.slice(-4),
      ].join('\n');
    }
    requestAnimationFrame(loop);
  };
  requestAnimationFrame(loop);

  // Test and automation hook (read-mostly; e2e tests drive the real UI).
  const hook = {
    ready: true,
    errors,
    get frames() {
      return frames;
    },
    state: () => ({
      ...sim.stats(),
      quality: { ...sim.quality },
      render: view.info,
      camera: {
        mode: rig.mode,
        shot: rig.shotName,
        position: view.camera.position.toArray(),
        underwater: view.underwaterNow,
        depth: view.cameraDepth(),
      },
      fps,
      frames,
      webgpuAvailable: webgpu,
      cameraModes: CAMERA_MODES,
    }),
  };
  (window as unknown as { __DD: typeof hook }).__DD = hook;
}

start().catch((e) => reportError(e instanceof Error ? `${e.message}\n${e.stack ?? ''}` : String(e)));
