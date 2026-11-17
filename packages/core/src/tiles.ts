// World tiles: the Brazil open world is organised in 500 m × 500 m tiles,
// centred on multiples of the tile size in the ENU world frame (as in
// Aviation Dynamics: T_i_j with i = round(east / 500), j = round(north / 500)).
//
// A destruction event declares its ground bounds; the helpers here find every
// tile it touches, hold activation leases so those tiles stay loaded while the
// event runs, and keep saveable world-state overrides (e.g. "tower is a wreck
// until 2026-10-24T12:00Z") per tile.

export const DEFAULT_TILE_SIZE = 500;

export interface TileCoord {
  i: number;
  j: number;
}

/** Axis-aligned ground bounds in the ENU world frame, metres. */
export interface GroundBounds {
  minEast: number;
  maxEast: number;
  minNorth: number;
  maxNorth: number;
}

export const tileKey = (t: TileCoord): string => `T_${t.i}_${t.j}`;

export function parseTileKey(key: string): TileCoord {
  const m = /^T_(-?\d+)_(-?\d+)$/.exec(key);
  if (!m) throw new Error(`invalid tile key: ${key}`);
  return { i: Number(m[1]), j: Number(m[2]) };
}

export function tileAt(east: number, north: number, size = DEFAULT_TILE_SIZE): TileCoord {
  // `|| 0` folds -0 into 0 so keys and equality stay canonical.
  return { i: Math.round(east / size) || 0, j: Math.round(north / size) || 0 };
}

export function tileBounds(t: TileCoord, size = DEFAULT_TILE_SIZE): GroundBounds {
  const h = size / 2;
  return {
    minEast: t.i * size - h,
    maxEast: t.i * size + h,
    minNorth: t.j * size - h,
    maxNorth: t.j * size + h,
  };
}

/** Every tile overlapped by `b` (inclusive of touching edges). */
export function tilesForBounds(b: GroundBounds, size = DEFAULT_TILE_SIZE): TileCoord[] {
  const lo = tileAt(b.minEast, b.minNorth, size);
  const hi = tileAt(b.maxEast, b.maxNorth, size);
  const out: TileCoord[] = [];
  for (let i = lo.i; i <= hi.i; i++) for (let j = lo.j; j <= hi.j; j++) out.push({ i, j });
  return out;
}

/** Position relative to a tile's centre (keeps float32 precision near the player). */
export function tileLocal(east: number, north: number, t: TileCoord, size = DEFAULT_TILE_SIZE) {
  return { east: east - t.i * size, north: north - t.j * size };
}

// ---------------------------------------------------------------------------

export interface TileLease {
  readonly tiles: readonly string[];
  readonly reason: string;
  release(): void;
}

/** What the host world streamer must provide. */
export interface TileActivationHost {
  acquire(tiles: readonly TileCoord[], reason: string): TileLease;
  isActive(tile: TileCoord): boolean;
}

/** Reference-counted in-memory implementation, for tests and the playground. */
export class InMemoryTileHost implements TileActivationHost {
  private readonly refs = new Map<string, number>();

  acquire(tiles: readonly TileCoord[], reason: string): TileLease {
    const keys = tiles.map(tileKey);
    for (const k of keys) this.refs.set(k, (this.refs.get(k) ?? 0) + 1);
    let released = false;
    return {
      tiles: keys,
      reason,
      release: () => {
        if (released) return;
        released = true;
        for (const k of keys) {
          const n = (this.refs.get(k) ?? 0) - 1;
          if (n <= 0) this.refs.delete(k);
          else this.refs.set(k, n);
        }
      },
    };
  }

  isActive(tile: TileCoord): boolean {
    return (this.refs.get(tileKey(tile)) ?? 0) > 0;
  }

  activeTiles(): string[] {
    return [...this.refs.keys()].sort();
  }
}

// ---------------------------------------------------------------------------

export interface TileOverride {
  eventId: string;
  /** Opaque world state for the tile, e.g. "wreck". */
  state: string;
  /** Wall-clock ms at which the override lapses; null = until cleared. */
  restoreAtMs: number | null;
}

export interface WorldStateSnapshot {
  schema: 'dd-world-state/1';
  overrides: Record<string, TileOverride[]>;
}

/** Saveable per-tile overrides written by destruction events. */
export class WorldStateStore {
  private readonly overrides = new Map<string, TileOverride[]>();

  set(eventId: string, tiles: readonly TileCoord[], state: string, restoreAtMs: number | null): void {
    for (const t of tiles) {
      const k = tileKey(t);
      const list = (this.overrides.get(k) ?? []).filter((o) => o.eventId !== eventId);
      list.push({ eventId, state, restoreAtMs });
      this.overrides.set(k, list);
    }
  }

  clear(eventId: string): void {
    for (const [k, list] of this.overrides) {
      const rest = list.filter((o) => o.eventId !== eventId);
      if (rest.length) this.overrides.set(k, rest);
      else this.overrides.delete(k);
    }
  }

  forTile(tile: TileCoord): readonly TileOverride[] {
    return this.overrides.get(tileKey(tile)) ?? [];
  }

  /** Drops overrides whose restore time has passed; returns affected event ids. */
  expire(nowMs: number): string[] {
    const expired = new Set<string>();
    for (const [k, list] of this.overrides) {
      const rest = list.filter((o) => {
        const lapsed = o.restoreAtMs !== null && o.restoreAtMs <= nowMs;
        if (lapsed) expired.add(o.eventId);
        return !lapsed;
      });
      if (rest.length) this.overrides.set(k, rest);
      else this.overrides.delete(k);
    }
    return [...expired];
  }

  toJSON(): WorldStateSnapshot {
    return { schema: 'dd-world-state/1', overrides: Object.fromEntries(this.overrides) };
  }

  static fromJSON(s: WorldStateSnapshot): WorldStateStore {
    if (s.schema !== 'dd-world-state/1') throw new Error(`unsupported world state schema: ${s.schema}`);
    const store = new WorldStateStore();
    for (const [k, list] of Object.entries(s.overrides)) {
      parseTileKey(k);
      store.overrides.set(
        k,
        list.map((o) => ({ ...o })),
      );
    }
    return store;
  }
}
