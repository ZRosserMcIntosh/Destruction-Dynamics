// Pole Dynamics — utility poles and overhead wires.
// SPDX-License-Identifier: MIT
//
// Thin Unity wrapper over the C ABI (include/pd/c_api.h). P/Invoke into
// libPoleDynamics (PoleDynamics.dll / libPoleDynamics.so / .dylib / .bundle).
//
// Frames: the native side is the sim frame (ENU: x east, y north, z up,
// right-handed). Unity is x east, y up, z north, left-handed. Points and
// vectors swap y and z; a rotation (w, x, y, z) becomes (w, -x, -z, -y).
// Nothing else in the game needs to know.
//
// Usage: one PoleWorld per streamed region, stepped from FixedUpdate at the
// game's 60 Hz fixed step. See PoleDynamicsBridge.cs.
using System;
using System.Runtime.InteropServices;
using System.Text;
using UnityEngine;

namespace PoleDynamics
{
    public enum PoleState { Intact = 0, Cracked = 1, Leaning = 2, Snapped = 3, Fallen = 4 }

    public enum PoleEventType
    {
        PoleCracked = 1, PoleLeaning = 2, PoleSnapped = 3, PoleFell = 4,
        WireSnapped = 5, WireOnGroundLive = 6, Arc = 7, PowerOut = 8, LaneBlocked = 9,
        TransformerDetached = 10, TransformerLanded = 11, ClampFailed = 12, PoleHitBody = 13,
        LightChanged = 14, PoleRepaired = 15, LaneCleared = 16, PowerRestored = 17,
    }

    public enum SpanMode { Analytic = 0, Awake = 1, Frozen = 2 }

    [StructLayout(LayoutKind.Sequential)]
    public struct PdEvent
    {
        public int type, pole, span, wire;
        public double x, y, z;
        public double value;
        public long ids;
    }

    [StructLayout(LayoutKind.Sequential)]
    public struct PdImpact
    {
        public double px, py, pz, nx, ny, nz, vx, vy, vz;
        public double vehicleMass, crushStiffness, crushPlateau;
    }

    [StructLayout(LayoutKind.Sequential)]
    public struct PdImpactResult
    {
        public int pole, state, broke, reserved;
        public double impulse, deltaV, peakForce, duration;
    }

    [StructLayout(LayoutKind.Sequential)]
    public struct PdPiece
    {
        public double px, py, pz, qw, qx, qy, qz, z0, z1;
    }

    [StructLayout(LayoutKind.Sequential)]
    public struct PdPoleState
    {
        public int id, state, pieces, transformer, lit, powered, section, reserved;
        public double tiltDeg;
        public PdPiece piece0, piece1;
        public double tx, ty, tz, tqw, tqx, tqy, tqz;
    }

    [StructLayout(LayoutKind.Sequential)]
    public struct PdCapsule
    {
        public int id, reserved;
        public double ax, ay, az, bx, by, bz, radius, vx, vy, vz;
    }

    [StructLayout(LayoutKind.Sequential)]
    public struct PdStats
    {
        public int poles, spans, awakeSpans, frozenSpans, nodes, bodies;
        public double lastStepMs, maxStepMs;
        public long steps;
    }

    internal static class Native
    {
#if UNITY_IOS && !UNITY_EDITOR
        const string Lib = "__Internal";
#else
        const string Lib = "PoleDynamics";
#endif
        [DllImport(Lib)] public static extern IntPtr pd_create_world(uint seed);
        [DllImport(Lib)] public static extern void pd_destroy_world(IntPtr w);
        [DllImport(Lib)] public static extern IntPtr pd_error(IntPtr w);
        [DllImport(Lib)] public static extern int pd_load_tile(IntPtr w, byte[] json, int length);
        [DllImport(Lib)] public static extern int pd_load_lanes(IntPtr w, byte[] json, int length);
        [DllImport(Lib)] public static extern void pd_unload_tile(IntPtr w, int tx, int ty);
        [DllImport(Lib)] public static extern void pd_set_weather(IntPtr w, double wetness);
        [DllImport(Lib)] public static extern int pd_set_param(IntPtr w, [MarshalAs(UnmanagedType.LPStr)] string name, double value);
        [DllImport(Lib)] public static extern double pd_get_param(IntPtr w, [MarshalAs(UnmanagedType.LPStr)] string name);
        [DllImport(Lib)] public static extern void pd_set_bodies(IntPtr w, PdCapsule[] bodies, int count);
        [DllImport(Lib)] public static extern int pd_step(IntPtr w, double dt);
        [DllImport(Lib)] public static extern int pd_report_impact(IntPtr w, ref PdImpact impact, out PdImpactResult result);
        [DllImport(Lib)] public static extern int pd_query_state(IntPtr w, int id, out PdPoleState state);
        [DllImport(Lib)] public static extern int pd_pole_count(IntPtr w);
        [DllImport(Lib)] public static extern int pd_pole_id(IntPtr w, int index);
        [DllImport(Lib)] public static extern int pd_span_count(IntPtr w);
        [DllImport(Lib)] public static extern int pd_span_mode(IntPtr w, int span);
        [DllImport(Lib)] public static extern void pd_span_info(IntPtr w, int span, out int a, out int b, out double tension);
        [DllImport(Lib)] public static extern int pd_get_wire_nodes(IntPtr w, int span, float[] output, int capacity);
        [DllImport(Lib)] public static extern int pd_poll_events(IntPtr w, [Out] PdEvent[] output, int capacity);
        [DllImport(Lib)] public static extern int pd_save_tile_state(IntPtr w, int tx, int ty, byte[] output, int capacity);
        [DllImport(Lib)] public static extern int pd_load_tile_state(IntPtr w, int tx, int ty, byte[] data, int size);
        [DllImport(Lib)] public static extern int pd_repair_pole(IntPtr w, int id);
        [DllImport(Lib)] public static extern void pd_stats(IntPtr w, out PdStats stats);
        [DllImport(Lib)] public static extern ulong pd_state_hash(IntPtr w);
        [DllImport(Lib)] public static extern void pd_unpack_lane(long id, out int tx, out int ty, out int edge, out int lane);
    }

    /// <summary>Sim (ENU, Z up, right-handed) to Unity (Y up, left-handed) and back.</summary>
    public static class Frame
    {
        public static Vector3 ToUnity(double x, double y, double z) => new Vector3((float)x, (float)z, (float)y);
        public static void FromUnity(Vector3 v, out double x, out double y, out double z) { x = v.x; y = v.z; z = v.y; }
        public static Quaternion ToUnity(double w, double x, double y, double z) =>
            new Quaternion((float)-x, (float)-z, (float)-y, (float)w);
    }

    /// <summary>A pole's world-space pose for rendering, in Unity space.</summary>
    public struct PolePose
    {
        public int id;
        public PoleState state;
        public int pieces;                 // 1: one mesh; 2: swap to the broken stub + top meshes
        public Vector3 basePosition, topPosition;
        public Quaternion baseRotation, topRotation;
        public float breakHeight;          // m along the pole (z1 of the stub)
        public int transformer;            // 0 none, 1 on the pole, 2 detached
        public Vector3 transformerPosition;
        public Quaternion transformerRotation;
        public bool lit, powered;
        public float tiltDeg;
    }

    /// <summary>Lane ID packed by the native side: (tile, edge, lane).</summary>
    public struct LaneId
    {
        public int tileX, tileY, edge, lane;
        public static LaneId Unpack(long id)
        {
            Native.pd_unpack_lane(id, out var tx, out var ty, out var e, out var l);
            return new LaneId { tileX = tx, tileY = ty, edge = e, lane = l };
        }
        public override string ToString() => $"T_{tileX}_{tileY}/e{edge}/l{lane}";
    }

    public sealed class PoleWorld : IDisposable
    {
        IntPtr world;
        readonly PdEvent[] eventBuffer = new PdEvent[256];
        float[] nodeBuffer = new float[8 * (4 + 24 * 3 + 1) + 1];
        PdCapsule[] capsules = new PdCapsule[64];

        public const double FixedStep = 1.0 / 60.0;

        public PoleWorld(uint seed = 1) { world = Native.pd_create_world(seed); }

        public void Dispose()
        {
            if (world != IntPtr.Zero) Native.pd_destroy_world(world);
            world = IntPtr.Zero;
        }

        public string Error => Marshal.PtrToStringAnsi(Native.pd_error(world));

        /// <summary>poles_&lt;tile&gt;.json as shipped with the world data (unchanged format).</summary>
        public int LoadTile(string json)
        {
            var bytes = Encoding.UTF8.GetBytes(json);
            return Native.pd_load_tile(world, bytes, bytes.Length);
        }

        public int LoadLanes(string json)
        {
            var bytes = Encoding.UTF8.GetBytes(json);
            return Native.pd_load_lanes(world, bytes, bytes.Length);
        }

        public void UnloadTile(int tileX, int tileY) => Native.pd_unload_tile(world, tileX, tileY);
        public void SetWetness(float wetness) => Native.pd_set_weather(world, wetness);
        public bool SetParam(string name, double value) => Native.pd_set_param(world, name, value) != 0;
        public double GetParam(string name) => Native.pd_get_param(world, name);

        /// <summary>Cars near poles, as capsules in Unity space (the falling pole and wires collide with them).</summary>
        public void SetBodies(int count, Func<int, (int id, Vector3 a, Vector3 b, float radius, Vector3 velocity)> body)
        {
            if (count > capsules.Length) count = capsules.Length;
            for (int i = 0; i < count; i++)
            {
                var c = body(i);
                ref var k = ref capsules[i];
                k.id = c.id;
                Frame.FromUnity(c.a, out k.ax, out k.ay, out k.az);
                Frame.FromUnity(c.b, out k.bx, out k.by, out k.bz);
                Frame.FromUnity(c.velocity, out k.vx, out k.vy, out k.vz);
                k.radius = c.radius;
            }
            Native.pd_set_bodies(world, capsules, count);
        }

        /// <summary>Call from FixedUpdate with Time.fixedDeltaTime (1/60 s). Returns internal steps that did work.</summary>
        public int Step(double dt = FixedStep) => Native.pd_step(world, dt);

        /// <summary>
        /// The vehicle module reports a hit: contact point, push direction into the pole
        /// and the car's velocity relative to it (Unity space), and its mass.
        /// </summary>
        public PdImpactResult ReportImpact(Vector3 point, Vector3 normal, Vector3 relativeVelocity, float vehicleMass,
                                           float crushStiffness = 0)
        {
            var i = new PdImpact { vehicleMass = vehicleMass, crushStiffness = crushStiffness };
            Frame.FromUnity(point, out i.px, out i.py, out i.pz);
            Frame.FromUnity(normal, out i.nx, out i.ny, out i.nz);
            Frame.FromUnity(relativeVelocity, out i.vx, out i.vy, out i.vz);
            Native.pd_report_impact(world, ref i, out var r);
            return r;
        }

        public bool QueryPole(int id, out PolePose pose)
        {
            pose = default;
            if (Native.pd_query_state(world, id, out var s) == 0) return false;
            pose.id = s.id;
            pose.state = (PoleState)s.state;
            pose.pieces = s.pieces;
            pose.basePosition = Frame.ToUnity(s.piece0.px, s.piece0.py, s.piece0.pz);
            pose.baseRotation = Frame.ToUnity(s.piece0.qw, s.piece0.qx, s.piece0.qy, s.piece0.qz);
            pose.topPosition = Frame.ToUnity(s.piece1.px, s.piece1.py, s.piece1.pz);
            pose.topRotation = Frame.ToUnity(s.piece1.qw, s.piece1.qx, s.piece1.qy, s.piece1.qz);
            pose.breakHeight = (float)s.piece0.z1;
            pose.transformer = s.transformer;
            pose.transformerPosition = Frame.ToUnity(s.tx, s.ty, s.tz);
            pose.transformerRotation = Frame.ToUnity(s.tqw, s.tqx, s.tqy, s.tqz);
            pose.lit = s.lit != 0;
            pose.powered = s.powered != 0;
            pose.tiltDeg = (float)s.tiltDeg;
            return true;
        }

        public int PoleCount => Native.pd_pole_count(world);
        public int PoleId(int index) => Native.pd_pole_id(world, index);
        public int SpanCount => Native.pd_span_count(world);
        public SpanMode GetSpanMode(int span) => (SpanMode)Native.pd_span_mode(world, span);

        public void SpanInfo(int span, out int poleA, out int poleB, out double maxTensionN) =>
            Native.pd_span_info(world, span, out poleA, out poleB, out maxTensionN);

        /// <summary>
        /// Simulated wire polylines of a woken or frozen span, in Unity space.
        /// The callback gets (wire index, kind 0 MV/1 LV/2 telecom/3 drop, points, count,
        /// broken-segment mask). Returns false for sleeping spans: draw those with the
        /// existing catenary renderer.
        /// </summary>
        public bool ForEachWire(int span, Vector3[] scratch, Action<int, int, Vector3[], int, uint> wire)
        {
            int n = Native.pd_get_wire_nodes(world, span, nodeBuffer, nodeBuffer.Length);
            if (n < 0)
            {
                nodeBuffer = new float[-n];
                n = Native.pd_get_wire_nodes(world, span, nodeBuffer, nodeBuffer.Length);
            }
            if (n <= 0) return false;
            int k = 0;
            int wires = (int)nodeBuffer[k++];
            for (int w = 0; w < wires; w++)
            {
                int kind = (int)nodeBuffer[k++];
                k++; // broken count
                int count = (int)nodeBuffer[k++];
                for (int i = 0; i < count && i < scratch.Length; i++, k += 3)
                    scratch[i] = new Vector3(nodeBuffer[k], nodeBuffer[k + 2], nodeBuffer[k + 1]);
                uint mask = (uint)nodeBuffer[k++];
                wire(w, kind, scratch, Math.Min(count, scratch.Length), mask);
            }
            return true;
        }

        /// <summary>Drains the event queue. Positions are converted to Unity space.</summary>
        public void PollEvents(Action<PoleEventType, PdEvent, Vector3> handler)
        {
            int n;
            while ((n = Native.pd_poll_events(world, eventBuffer, eventBuffer.Length)) > 0)
                for (int i = 0; i < n; i++)
                {
                    var e = eventBuffer[i];
                    handler((PoleEventType)e.type, e, Frame.ToUnity(e.x, e.y, e.z));
                }
        }

        /// <summary>Damage diff of a tile, for the save game. Null when nothing is damaged.</summary>
        public byte[] SaveTileState(int tileX, int tileY)
        {
            int need = -Native.pd_save_tile_state(world, tileX, tileY, null, 0);
            if (need <= 16) return null;
            var buf = new byte[need];
            Native.pd_save_tile_state(world, tileX, tileY, buf, buf.Length);
            return buf;
        }

        public bool LoadTileState(int tileX, int tileY, byte[] data) =>
            data != null && Native.pd_load_tile_state(world, tileX, tileY, data, data.Length) >= 0;

        /// <summary>The repair-crew hook: the pole goes back up and its spans are re-strung.</summary>
        public bool RepairPole(int id) => Native.pd_repair_pole(world, id) != 0;

        public PdStats Stats { get { Native.pd_stats(world, out var s); return s; } }
        public ulong StateHash => Native.pd_state_hash(world);
    }
}
