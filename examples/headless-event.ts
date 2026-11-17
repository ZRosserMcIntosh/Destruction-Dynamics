// Minimal integration example: drive an event, the authored tower, physics
// collision and water from your own game loop, with no rendering.
// Verified by examples/headless-event.test.ts.

import { InMemoryTileHost, tilesForBounds } from '@destruction-dynamics/core';
import {
  DEEP_END_TOWER,
  TowerCollapse,
  TowerCollision,
  towerCollisionModeAt,
} from '@destruction-dynamics/destruction';
import { EventDirector } from '@destruction-dynamics/events';
import { HybridOcean, makeWaves } from '@destruction-dynamics/fluids';
import { RapierPhysicsAdapter } from '@destruction-dynamics/physics';
import {
  deepEndDepth,
  deepEndEvent,
  deepEndGroundHeight,
  deepEndTimeline,
} from '@destruction-dynamics/scenarios';

export async function runHeadlessDeepEnd(seed = 42) {
  // 1. The event and its authored structure.
  const director = new EventDirector(deepEndEvent, { seed });
  const timeline = deepEndTimeline();
  const tower = new TowerCollapse(DEEP_END_TOWER, timeline, {
    groundHeight: deepEndGroundHeight,
    seed: director.seed,
  });

  // 2. Your physics world, through the adapter.
  const physics = new RapierPhysicsAdapter();
  await physics.init();
  const collision = new TowerCollision(physics, tower);

  // 3. Water: swap HybridOcean for your own FluidAdapter if you have one.
  const ocean = new HybridOcean({
    waves: makeWaves('calm', 0, 1),
    depth: deepEndDepth,
    nearField: { size: 96, extent: 840, centerX: 260, centerZ: 0 },
  });

  // 4. Keep the event's world tiles streamed in while it runs.
  const tiles = new InMemoryTileHost();
  const lease = tiles.acquire(tilesForBounds(deepEndEvent.bounds), deepEndEvent.id);

  // 5. React to cues (here: water impacts) and sample continuous state each frame.
  director.on('cue', ({ cue }) => {
    if (cue.channel === 'fluid' && cue.type === 'impact') {
      const f = tower.impactFootprint();
      ocean.applyImpact({
        position: f.position,
        footprint: { kind: 'box', halfLength: f.halfLength, halfWidth: f.halfWidth, yaw: f.yaw },
        intensity: Number(cue.params?.intensity ?? 1),
        duration: Number(cue.params?.duration ?? 1.5),
        direction: f.direction,
        visualProfile: String(cue.params?.visualProfile ?? 'landmark-collapse'),
      });
    }
  });

  director.trigger();
  const dt = 1 / 30;
  let peakWave = 0;
  while (director.state !== 'AFTERMATH') {
    director.update(dt);
    const poses = tower.posesAt(director.time);
    collision.update(towerCollisionModeAt(director.state, director.time, timeline), poses);
    physics.step(dt);
    ocean.step(dt);
    peakWave = Math.max(peakWave, ocean.maxDisturbance());
  }
  lease.release();
  const result = { state: director.state, time: director.time, collision: collision.mode, peakWave };
  physics.dispose();
  return result;
}
