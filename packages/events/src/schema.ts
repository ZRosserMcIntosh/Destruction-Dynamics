// The versioned, data-driven event definition format ("dd-event/1").
//
// An event is a cancellable preparation window followed by an ordered list of
// committed stages. Event time t is measured in seconds from the commit
// instant: preparation occupies [-preparation.duration, 0), the first stage
// starts at t = 0. Cues are one-shot instructions to subsystems, timed
// relative to the start of their stage.

import type { GroundBounds } from '@destruction-dynamics/core';

export const EVENT_SCHEMA = 'dd-event/1';

export const LIFECYCLE_STATES = [
  'INTACT',
  'PREPARING',
  'TRIGGERED',
  'INITIAL_FAILURE',
  'ACTIVE_DESTRUCTION',
  'WATER_IMPACT',
  'SETTLING',
  'AFTERMATH',
  'RESTORATION',
] as const;
export type LifecycleState = (typeof LIFECYCLE_STATES)[number];

/** States a committed stage may represent, in the order they must appear. */
export const STAGE_STATES = [
  'INITIAL_FAILURE',
  'ACTIVE_DESTRUCTION',
  'WATER_IMPACT',
  'SETTLING',
  'AFTERMATH',
] as const;
export type StageState = (typeof STAGE_STATES)[number];

export const CUE_CHANNELS = [
  'audio',
  'camera',
  'fluid',
  'particles',
  'physics',
  'collision',
  'world',
  'custom',
] as const;
export type CueChannel = (typeof CUE_CHANNELS)[number];

export interface CueDefinition {
  id: string;
  /** Seconds from the start of the owning stage (or preparation window). */
  at: number;
  channel: CueChannel;
  type: string;
  params?: Record<string, unknown>;
}

export interface StageDefinition {
  id: string;
  state: StageState;
  /** Seconds; null only for a final open-ended AFTERMATH stage. */
  duration: number | null;
  cues: CueDefinition[];
}

export type RestorationPolicy =
  | { mode: 'real-world-duration'; hours: number }
  | { mode: 'event-duration'; seconds: number }
  | { mode: 'manual' };

export interface EventDefinition {
  schema: typeof EVENT_SCHEMA;
  id: string;
  /** Content version; saves and network snapshots must match it. */
  version: number;
  title: string;
  environment: string;
  eventType: 'landmark-collapse' | 'structural-failure' | 'custom';
  /** Ground footprint in the ENU world frame, for tile activation. */
  bounds: GroundBounds;
  preparation: { duration: number; cues: CueDefinition[] };
  stages: StageDefinition[];
  restoration: RestorationPolicy & { transitionSeconds: number };
}

export interface ValidationResult {
  ok: boolean;
  errors: string[];
}

const isObj = (v: unknown): v is Record<string, unknown> =>
  typeof v === 'object' && v !== null && !Array.isArray(v);
const isNum = (v: unknown): v is number => typeof v === 'number' && Number.isFinite(v);
const isStr = (v: unknown): v is string => typeof v === 'string' && v.length > 0;

function validateCues(
  cues: unknown,
  path: string,
  duration: number | null,
  errors: string[],
  ids: Set<string>,
) {
  if (!Array.isArray(cues)) {
    errors.push(`${path}.cues must be an array`);
    return;
  }
  cues.forEach((c, i) => {
    const p = `${path}.cues[${i}]`;
    if (!isObj(c)) return void errors.push(`${p} must be an object`);
    if (!isStr(c.id)) errors.push(`${p}.id must be a non-empty string`);
    else if (ids.has(c.id)) errors.push(`${p}.id "${c.id}" is not unique`);
    else ids.add(c.id);
    if (!isNum(c.at) || c.at < 0) errors.push(`${p}.at must be a number >= 0`);
    else if (duration !== null && c.at >= duration)
      errors.push(`${p}.at (${c.at}) must be < stage duration (${duration})`);
    if (!CUE_CHANNELS.includes(c.channel as CueChannel))
      errors.push(`${p}.channel must be one of ${CUE_CHANNELS.join(', ')}`);
    if (!isStr(c.type)) errors.push(`${p}.type must be a non-empty string`);
    if (c.params !== undefined && !isObj(c.params)) errors.push(`${p}.params must be an object`);
  });
}

/** Structural and semantic validation. Never throws. */
export function validateEventDefinition(doc: unknown): ValidationResult {
  const errors: string[] = [];
  if (!isObj(doc)) return { ok: false, errors: ['definition must be an object'] };
  if (doc.schema !== EVENT_SCHEMA) errors.push(`schema must be "${EVENT_SCHEMA}"`);
  if (!isStr(doc.id) || !/^[a-z0-9][a-z0-9-]*$/.test(doc.id)) errors.push('id must be a kebab-case string');
  if (!Number.isInteger(doc.version) || (doc.version as number) < 1)
    errors.push('version must be an integer >= 1');
  if (!isStr(doc.title)) errors.push('title must be a non-empty string');
  if (!isStr(doc.environment)) errors.push('environment must be a non-empty string');
  if (!['landmark-collapse', 'structural-failure', 'custom'].includes(doc.eventType as string))
    errors.push('eventType must be landmark-collapse, structural-failure or custom');

  const b = doc.bounds;
  if (!isObj(b) || !['minEast', 'maxEast', 'minNorth', 'maxNorth'].every((k) => isNum(b[k])))
    errors.push('bounds must have numeric minEast, maxEast, minNorth, maxNorth');
  else if ((b.minEast as number) > (b.maxEast as number) || (b.minNorth as number) > (b.maxNorth as number))
    errors.push('bounds min must not exceed max');

  const ids = new Set<string>();
  const prep = doc.preparation;
  if (!isObj(prep) || !isNum(prep.duration) || prep.duration < 0)
    errors.push('preparation.duration must be a number >= 0');
  else validateCues(prep.cues, 'preparation', prep.duration > 0 ? prep.duration : null, errors, ids);

  const stageIds = new Set<string>();
  if (!Array.isArray(doc.stages) || doc.stages.length === 0) errors.push('stages must be a non-empty array');
  else {
    let lastOrder = -1;
    doc.stages.forEach((s: unknown, i: number) => {
      const p = `stages[${i}]`;
      if (!isObj(s)) return void errors.push(`${p} must be an object`);
      if (!isStr(s.id)) errors.push(`${p}.id must be a non-empty string`);
      else if (stageIds.has(s.id)) errors.push(`${p}.id "${s.id}" is not unique`);
      else stageIds.add(s.id);
      const order = STAGE_STATES.indexOf(s.state as StageState);
      if (order < 0) errors.push(`${p}.state must be one of ${STAGE_STATES.join(', ')}`);
      else if (order < lastOrder) errors.push(`${p}.state ${String(s.state)} is out of lifecycle order`);
      else lastOrder = order;
      const last = i === (doc.stages as unknown[]).length - 1;
      if (s.duration === null) {
        if (!last || s.state !== 'AFTERMATH')
          errors.push(`${p}.duration may be null only for a final AFTERMATH stage`);
      } else if (!isNum(s.duration) || s.duration <= 0) errors.push(`${p}.duration must be a number > 0`);
      validateCues(s.cues, p, isNum(s.duration) ? s.duration : null, errors, ids);
    });
    const lastStage = doc.stages[doc.stages.length - 1] as Record<string, unknown> | undefined;
    if (lastStage && lastStage.state !== 'AFTERMATH') errors.push('the final stage must be AFTERMATH');
  }

  const r = doc.restoration;
  if (!isObj(r)) errors.push('restoration must be an object');
  else {
    if (r.mode === 'real-world-duration') {
      if (!isNum(r.hours) || r.hours < 0) errors.push('restoration.hours must be a number >= 0');
    } else if (r.mode === 'event-duration') {
      if (!isNum(r.seconds) || r.seconds < 0) errors.push('restoration.seconds must be a number >= 0');
    } else if (r.mode !== 'manual')
      errors.push('restoration.mode must be real-world-duration, event-duration or manual');
    if (!isNum(r.transitionSeconds) || r.transitionSeconds < 0)
      errors.push('restoration.transitionSeconds must be a number >= 0');
  }
  return { ok: errors.length === 0, errors };
}

export class EventDefinitionError extends Error {
  constructor(public readonly errors: string[]) {
    super(`invalid event definition:\n  ${errors.join('\n  ')}`);
  }
}

/** Validates and returns a typed definition, or throws EventDefinitionError. */
export function parseEventDefinition(doc: unknown): EventDefinition {
  const r = validateEventDefinition(doc);
  if (!r.ok) throw new EventDefinitionError(r.errors);
  return doc as EventDefinition;
}

export interface StageWindow {
  id: string;
  state: StageState;
  start: number;
  /** Infinity for an open-ended stage. */
  end: number;
}

/** Absolute event-time windows of each committed stage. */
export function stageWindows(def: EventDefinition): StageWindow[] {
  let t = 0;
  return def.stages.map((s) => {
    const start = t;
    const end = s.duration === null ? Infinity : t + s.duration;
    t = end;
    return { id: s.id, state: s.state, start, end };
  });
}

/** Window of the first stage in `state`, or undefined. */
export function windowFor(def: EventDefinition, state: StageState): StageWindow | undefined {
  return stageWindows(def).find((w) => w.state === state);
}
