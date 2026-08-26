using System;
using System.Collections.Generic;

namespace XMflight
{
    public enum EndpointObservationCacheStoreOutcome
    {
        Stored,
        Duplicate,
    }

    public sealed class EndpointObservationCacheProtocolException : InvalidOperationException
    {
        public EndpointObservationCacheProtocolException(string message) : base(message) { }
    }

    public sealed class EndpointObservationCacheStoreResult
    {
        public readonly EndpointObservationCacheStoreOutcome outcome;
        public readonly EndpointObservationSnapshotV4 snapshot;

        internal EndpointObservationCacheStoreResult(
            EndpointObservationCacheStoreOutcome outcome,
            EndpointObservationSnapshotV4 snapshot)
        {
            this.outcome = outcome;
            this.snapshot = snapshot;
        }
    }

    /// <summary>
    /// Unity's authoritative in-memory store for exact terminal endpoint
    /// observations.  It is intentionally a pure cache: no socket, telemetry,
    /// result transport, or transition side effect is owned here.
    /// </summary>
    public sealed class EndpointObservationCache
    {
        private readonly Dictionary<string, EndpointObservationSnapshotV4> snapshots =
            new Dictionary<string, EndpointObservationSnapshotV4>();

        public int Count { get { return snapshots.Count; } }

        /// <summary>
        /// Create and retain one immutable snapshot after frame 24's exact
        /// state and depth bytes are both available.  The supplied identity is
        /// the only lookup key; this cache has no latest/nearest fallback.
        /// </summary>
        public EndpointObservationCacheStoreResult StoreTerminalFrame24(
            string runtimeInstanceId,
            string episodeId,
            string resetId,
            long stateId,
            string depthId,
            ulong simTimeNs,
            byte[] stateBytes,
            byte[] depthBytes)
        {
            if (stateBytes == null || depthBytes == null ||
                stateBytes.Length == 0 || depthBytes.Length == 0)
                throw new ArgumentException("terminal snapshot state_bytes and depth_bytes are required");

            var observationRef = new ObservationRefV4 {
                schema_version = 4,
                runtime_instance_id = runtimeInstanceId,
                episode_id = episodeId,
                reset_id = resetId,
                state_id = stateId,
                depth_id = depthId,
                sim_time_ns = simTimeNs,
            };
            string key = Key(observationRef);
            var snapshot = new EndpointObservationSnapshotV4 {
                observation_ref = CloneRef(observationRef),
                state_bytes = CloneBytes(stateBytes),
                depth_bytes = CloneBytes(depthBytes),
            };
            snapshot.snapshot_hash = XMProtocolV4.SnapshotHash(snapshot);

            EndpointObservationSnapshotV4 existing;
            if (snapshots.TryGetValue(key, out existing)) {
                if (!EqualBytes(existing.snapshot_hash, snapshot.snapshot_hash))
                    throw new EndpointObservationCacheProtocolException(
                        "snapshot_hash conflict for immutable observation_ref");
                return new EndpointObservationCacheStoreResult(
                    EndpointObservationCacheStoreOutcome.Duplicate,
                    CloneSnapshot(existing));
            }

            snapshots.Add(key, CloneSnapshot(snapshot));
            return new EndpointObservationCacheStoreResult(
                EndpointObservationCacheStoreOutcome.Stored,
                CloneSnapshot(snapshot));
        }

        public bool TryGet(ObservationRefV4 observationRef, out EndpointObservationSnapshotV4 snapshot)
        {
            EndpointObservationSnapshotV4 existing;
            if (!snapshots.TryGetValue(Key(observationRef), out existing)) {
                snapshot = null;
                return false;
            }
            snapshot = CloneSnapshot(existing);
            return true;
        }

        /// <summary>
        /// End retention only for an exact immutable identity.  A different
        /// hash is a protocol error and cannot remove the stored snapshot.
        /// </summary>
        public bool Release(ObservationRefV4 observationRef, byte[] snapshotHash)
        {
            RequireHash(snapshotHash);
            string key = Key(observationRef);
            EndpointObservationSnapshotV4 existing;
            if (!snapshots.TryGetValue(key, out existing)) return false;
            if (!EqualBytes(existing.snapshot_hash, snapshotHash))
                throw new EndpointObservationCacheProtocolException(
                    "snapshot_hash conflict while releasing observation_ref");
            snapshots.Remove(key);
            return true;
        }

        private static string Key(ObservationRefV4 observationRef)
        {
            return Convert.ToBase64String(XMProtocolV4.CanonicalObservationRef(observationRef));
        }

        private static EndpointObservationSnapshotV4 CloneSnapshot(
            EndpointObservationSnapshotV4 snapshot)
        {
            if (snapshot == null || snapshot.observation_ref == null ||
                snapshot.state_bytes == null || snapshot.depth_bytes == null ||
                snapshot.snapshot_hash == null)
                throw new EndpointObservationCacheProtocolException("snapshot is incomplete");
            RequireHash(snapshot.snapshot_hash);
            return new EndpointObservationSnapshotV4 {
                observation_ref = CloneRef(snapshot.observation_ref),
                state_bytes = CloneBytes(snapshot.state_bytes),
                depth_bytes = CloneBytes(snapshot.depth_bytes),
                snapshot_hash = CloneBytes(snapshot.snapshot_hash),
            };
        }

        private static ObservationRefV4 CloneRef(ObservationRefV4 value)
        {
            if (value == null) throw new EndpointObservationCacheProtocolException("observation_ref is required");
            return new ObservationRefV4 {
                schema_version = value.schema_version,
                runtime_instance_id = value.runtime_instance_id,
                episode_id = value.episode_id,
                reset_id = value.reset_id,
                state_id = value.state_id,
                depth_id = value.depth_id,
                sim_time_ns = value.sim_time_ns,
            };
        }

        private static byte[] CloneBytes(byte[] value)
        {
            return (byte[])value.Clone();
        }

        private static void RequireHash(byte[] value)
        {
            if (value == null || value.Length != 32)
                throw new EndpointObservationCacheProtocolException("snapshot_hash must be bytes32");
        }

        private static bool EqualBytes(byte[] left, byte[] right)
        {
            if (left == null || right == null || left.Length != right.Length) return false;
            for (int index = 0; index < left.Length; ++index)
                if (left[index] != right[index]) return false;
            return true;
        }
    }
}
