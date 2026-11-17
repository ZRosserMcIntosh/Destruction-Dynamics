// Runs the floor-layer solver off the main thread and streams keyframes back
// in chunks as they are computed, so playback never waits for the whole solve.

import { LayerSolver, type LayerBodyInit, type LayerSolverOptions } from '@destruction-dynamics/destruction';
import { deepEndGroundHeight } from '@destruction-dynamics/scenarios';

export interface LayerJob {
  id: number;
  layers: LayerBodyInit[];
  options: Omit<LayerSolverOptions, 'groundHeight'>;
}

export type LayerMessage =
  | { id: number; kind: 'frames'; from: number; to: number; total: number; hz: number; slice: Float32Array }
  | {
      id: number;
      kind: 'done';
      jointBreakTimes: Float32Array;
      settledAfter: number;
      computeMs: number;
      substeps: number;
    };

self.onmessage = (e: MessageEvent<LayerJob>) => {
  const { id, layers, options } = e.data;
  const n = layers.length;
  const hz = options.keyframeHz ?? 30;
  const post = (m: LayerMessage, transfer: Transferable[]) =>
    (self as unknown as Worker).postMessage(m, transfer);
  const result = new LayerSolver(layers, { ...options, groundHeight: deepEndGroundHeight }).run({
    chunkFrames: 15,
    onFrames: (from, to, data) => {
      const slice = data.slice(from * n * 7, to * n * 7);
      post({ id, kind: 'frames', from, to, total: data.length / (n * 7), hz, slice }, [slice.buffer]);
    },
  });
  post(
    {
      id,
      kind: 'done',
      jointBreakTimes: result.jointBreakTimes,
      settledAfter: result.settledAfter,
      computeMs: result.computeMs,
      substeps: result.substeps ?? 0,
    },
    [result.jointBreakTimes.buffer],
  );
};
