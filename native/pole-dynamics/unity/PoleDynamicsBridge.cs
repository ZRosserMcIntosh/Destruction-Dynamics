// Pole Dynamics — utility poles and overhead wires.
// SPDX-License-Identifier: MIT
//
// Example bridge between the game and PoleWorld. Adapt to the game's own
// systems; everything here goes through the public PoleWorld API.
//
//  - Steps the world in FixedUpdate (60 Hz).
//  - Poles are drawn GPU-instanced: one matrix list for intact/cracked/leaning
//    poles (one mesh), and a broken-mesh swap (stub + top) for snapped ones.
//  - Sleeping spans stay with the existing catenary renderer; woken and frozen
//    spans are drawn from the simulated nodes.
//  - Events are forwarded as C# events (lanes to traffic, power to lights,
//    arcs to VFX/audio, transformer landing to the oil/fire systems).
using System;
using System.Collections.Generic;
using UnityEngine;

namespace PoleDynamics
{
    public class PoleDynamicsBridge : MonoBehaviour
    {
        [Header("Meshes (GPU instanced)")]
        public Mesh poleMesh;
        public Mesh poleStubMesh;      // the base up to the break, pivot at the ground
        public Mesh poleTopMesh;       // the top, pivot at the break
        public Mesh transformerMesh;
        public Material poleMaterial;
        public Material transformerMaterial;

        [Header("Wires")]
        public Material wireMaterial;
        public float[] wireWidth = { 0.010f, 0.010f, 0.030f, 0.008f }; // MV, LV, telecom, drop

        [Header("World")]
        [Range(0, 1)] public float wetness;

        /// <summary>Set by the game: draw a sleeping span with the existing catenary renderer.</summary>
        public Action<int, bool> setCatenaryVisible;

        public event Action<long[]> LaneBlocked;           // packed lane IDs
        public event Action<int, Vector3> PowerOut;        // section (transformer pole ID), where
        public event Action<int, bool> LightChanged;       // pole ID, lit
        public event Action<Vector3, float> Arc;           // where, volts
        public event Action<Vector3, float> LiveWireDown;  // where, hazard radius m
        public event Action<int, Vector3, float, bool, bool> TransformerLanded; // pole, where, speed, oil, fire
        public event Action<PoleEventType, PdEvent, Vector3> AnyEvent;

        public PoleWorld World { get; private set; }

        readonly List<Matrix4x4> whole = new List<Matrix4x4>(1024);
        readonly List<Matrix4x4> stubs = new List<Matrix4x4>(64);
        readonly List<Matrix4x4> tops = new List<Matrix4x4>(64);
        readonly List<Matrix4x4> tanks = new List<Matrix4x4>(64);
        readonly Dictionary<int, LineRenderer> wireLines = new Dictionary<int, LineRenderer>();
        readonly Vector3[] scratch = new Vector3[32];
        readonly long[] laneOne = new long[1];

        void Awake() { World = new PoleWorld(1); }
        void OnDestroy() { World?.Dispose(); }

        public int LoadTile(string polesJson, string lanesJson = null)
        {
            int n = World.LoadTile(polesJson);
            if (n < 0) Debug.LogError($"Pole Dynamics: {World.Error}");
            if (lanesJson != null) World.LoadLanes(lanesJson);
            return n;
        }

        /// <summary>Called by the vehicle module on contact with a pole.</summary>
        public PdImpactResult OnVehicleHit(Vector3 point, Vector3 normal, Vector3 relativeVelocity, float mass) =>
            World.ReportImpact(point, normal, relativeVelocity, mass);

        void FixedUpdate()
        {
            World.SetWetness(wetness);
            World.Step(Time.fixedDeltaTime);
            World.PollEvents(Dispatch);
        }

        void Dispatch(PoleEventType type, PdEvent e, Vector3 at)
        {
            AnyEvent?.Invoke(type, e, at);
            switch (type)
            {
                case PoleEventType.LaneBlocked: laneOne[0] = e.ids; LaneBlocked?.Invoke(laneOne); break;
                case PoleEventType.PowerOut: PowerOut?.Invoke((int)e.ids, at); break;
                case PoleEventType.LightChanged: LightChanged?.Invoke(e.pole, e.value > 0.5); break;
                case PoleEventType.Arc: Arc?.Invoke(at, (float)e.value); break;
                case PoleEventType.WireOnGroundLive: LiveWireDown?.Invoke(at, e.ids / 1000f); break;
                case PoleEventType.TransformerLanded:
                    TransformerLanded?.Invoke(e.pole, at, (float)e.value, (e.ids & 1) != 0, (e.ids & 2) != 0);
                    break;
            }
        }

        void LateUpdate()
        {
            DrawPoles();
            DrawWires();
        }

        void DrawPoles()
        {
            whole.Clear(); stubs.Clear(); tops.Clear(); tanks.Clear();
            int n = World.PoleCount;
            for (int i = 0; i < n; i++)
            {
                if (!World.QueryPole(World.PoleId(i), out var p)) continue;
                if (p.pieces == 1)
                    whole.Add(Matrix4x4.TRS(p.basePosition, p.baseRotation, Vector3.one));
                else
                {
                    stubs.Add(Matrix4x4.TRS(p.basePosition, p.baseRotation, Vector3.one));
                    tops.Add(Matrix4x4.TRS(p.topPosition, p.topRotation, Vector3.one));
                }
                if (p.transformer == 2) tanks.Add(Matrix4x4.TRS(p.transformerPosition, p.transformerRotation, Vector3.one));
            }
            Draw(poleMesh, poleMaterial, whole);
            Draw(poleStubMesh, poleMaterial, stubs);
            Draw(poleTopMesh, poleMaterial, tops);
            Draw(transformerMesh, transformerMaterial, tanks);
        }

        static void Draw(Mesh mesh, Material mat, List<Matrix4x4> list)
        {
            if (mesh == null || mat == null) return;
            for (int k = 0; k < list.Count; k += 1023)
                Graphics.DrawMeshInstanced(mesh, 0, mat, list.GetRange(k, Mathf.Min(1023, list.Count - k)));
        }

        void DrawWires()
        {
            int spans = World.SpanCount;
            for (int s = 0; s < spans; s++)
            {
                bool simulated = World.GetSpanMode(s) != SpanMode.Analytic;
                setCatenaryVisible?.Invoke(s, !simulated);
                for (int w = 0; w < 9; w++)
                    if (wireLines.TryGetValue(s * 16 + w, out var lr)) lr.enabled = false;
                if (!simulated) continue;
                World.ForEachWire(s, scratch, (w, kind, pts, count, broken) =>
                {
                    var lr = Line(s * 16 + w, kind);
                    lr.enabled = true;
                    lr.positionCount = count;
                    lr.SetPositions(pts);
                    // A snapped wire is two pieces; drawn as one strip here for brevity. Split at
                    // the first set bit of `broken` for production.
                });
            }
        }

        LineRenderer Line(int key, int kind)
        {
            if (wireLines.TryGetValue(key, out var lr)) return lr;
            var go = new GameObject($"wire {key}");
            go.transform.SetParent(transform, false);
            lr = go.AddComponent<LineRenderer>();
            lr.sharedMaterial = wireMaterial;
            lr.widthMultiplier = wireWidth[Mathf.Clamp(kind, 0, 3)];
            lr.useWorldSpace = true;
            wireLines[key] = lr;
            return lr;
        }
    }
}
