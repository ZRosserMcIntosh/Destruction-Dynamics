import { expect, it } from 'vitest';
import { runHeadlessDeepEnd } from './headless-event';

it('the headless integration example runs the event to aftermath', async () => {
  const r = await runHeadlessDeepEnd();
  expect(r.state).toBe('AFTERMATH');
  expect(r.collision).toBe('wreck');
  expect(r.peakWave).toBeGreaterThan(3);
}, 60_000);
