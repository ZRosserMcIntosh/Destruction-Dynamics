// Runs the floor-layer solver off the main thread. The tower posts its release
// state; the result (keyframes) comes back as transferable typed arrays.

import { LayerSolver, type LayerBodyInit, type LayerSolverOptions } from '@destruction-dynamics/destruction';
import { deepEndGroundHeight } from '@destruction-dynamics/scenarios';

export interface LayerJob {
  id: number;
  layers: LayerBodyInit[];
  options: Omit<LayerSolverOptions, 'groundHeight'>;
}

self.onmessage = (e: MessageEvent<LayerJob>) => {
  const { id, layers, options } = e.data;
  const result = new LayerSolver(layers, { ...options, groundHeight: deepEndGroundHeight }).run();
  (self as unknown as Worker).postMessage({ id, result }, [
    result.data.buffer,
    result.jointBreakTimes.buffer,
  ]);
};
