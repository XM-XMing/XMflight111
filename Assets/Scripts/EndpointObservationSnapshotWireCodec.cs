using MessagePack;
using System;
using System.Collections;
using System.Collections.Generic;

namespace XMflight
{
    /// <summary>
    /// Runtime-only MessagePack envelopes for the independent snapshot channel.
    /// Snapshot hashes and request identities remain defined by XMProtocolV4's
    /// canonical payloads; these envelopes add routing message_type only.
    /// </summary>
    public static class EndpointObservationSnapshotWireCodec
    {
        public static byte[] SerializeReady(string runtimeInstanceId)
        {
            return MessagePackSerializer.Serialize(new Dictionary<string, object> {
                { "schema_version", 4U },
                { "message_type", "EndpointObservationSnapshotReady" },
                { "runtime_instance_id", runtimeInstanceId },
            });
        }

        public static byte[] SerializeSnapshot(
            SnapshotRequestV4 request, EndpointObservationSnapshotV4 snapshot)
        {
            ValidateRequest(request);
            if (snapshot == null || snapshot.observation_ref == null ||
                !SameRef(request.observation_ref, snapshot.observation_ref) ||
                !SameBytes(snapshot.snapshot_hash, XMProtocolV4.SnapshotHash(snapshot)))
                throw new ArgumentException("snapshot response identity mismatch");
            return MessagePackSerializer.Serialize(new Dictionary<string, object> {
                { "schema_version", 4U },
                { "message_type", "EndpointObservationSnapshot" },
                { "observation_ref", RefMap(snapshot.observation_ref) },
                { "state_bytes", snapshot.state_bytes },
                { "depth_bytes", snapshot.depth_bytes },
                { "snapshot_hash", snapshot.snapshot_hash },
                { "execution_id", request.execution_id },
                { "result_payload_hash", request.result_payload_hash },
                { "command_sequence_hash", request.command_sequence_hash },
            });
        }

        public static byte[] SerializeMissing(SnapshotRequestV4 request)
        {
            ValidateRequest(request);
            return MessagePackSerializer.Serialize(new Dictionary<string, object> {
                { "schema_version", 4U },
                { "message_type", "SnapshotMissing" },
                { "observation_ref", RefMap(request.observation_ref) },
                { "execution_id", request.execution_id },
                { "result_payload_hash", request.result_payload_hash },
                { "command_sequence_hash", request.command_sequence_hash },
            });
        }

        public static bool TryDeserializeRequest(byte[] raw, out SnapshotRequestV4 request)
        {
            request = null;
            Dictionary<string, object> values;
            if (!TryMap(raw, out values) || !MessageType(values, "SnapshotRequest")) return false;
            try {
                request = new SnapshotRequestV4 {
                    observation_ref = ReadRef(RequiredMap(values, "observation_ref")),
                    execution_id = Convert.ToUInt64(Required(values, "execution_id")),
                    result_payload_hash = Bytes32(Required(values, "result_payload_hash")),
                    command_sequence_hash = Bytes32(Required(values, "command_sequence_hash")),
                };
                XMProtocolV4.CanonicalSnapshotRequest(request);
                return true;
            } catch (Exception) {
                request = null;
                return false;
            }
        }

        public static bool TryDeserializeAck(byte[] raw, out SnapshotAckV4 ack)
        {
            ack = null;
            Dictionary<string, object> values;
            if (!TryMap(raw, out values) || !MessageType(values, "SnapshotAck")) return false;
            try {
                ack = new SnapshotAckV4 {
                    observation_ref = ReadRef(RequiredMap(values, "observation_ref")),
                    snapshot_hash = Bytes32(Required(values, "snapshot_hash")),
                };
                XMProtocolV4.CanonicalSnapshotAck(ack);
                return true;
            } catch (Exception) {
                ack = null;
                return false;
            }
        }

        private static bool TryMap(byte[] raw, out Dictionary<string, object> values)
        {
            values = null;
            try {
                values = MessagePackSerializer.Deserialize<Dictionary<string, object>>(raw);
                return values != null && Convert.ToUInt32(Required(values, "schema_version")) == 4U;
            } catch (Exception) { return false; }
        }

        private static bool MessageType(Dictionary<string, object> values, string expected)
        {
            return Convert.ToString(Required(values, "message_type")) == expected;
        }

        private static Dictionary<string, object> RefMap(ObservationRefV4 value)
        {
            return new Dictionary<string, object> {
                { "schema_version", value.schema_version },
                { "runtime_instance_id", value.runtime_instance_id },
                { "episode_id", value.episode_id },
                { "reset_id", value.reset_id },
                { "state_id", value.state_id },
                { "depth_id", value.depth_id },
                { "sim_time_ns", value.sim_time_ns },
            };
        }

        private static ObservationRefV4 ReadRef(Dictionary<string, object> value)
        {
            return new ObservationRefV4 {
                schema_version = Convert.ToUInt32(Required(value, "schema_version")),
                runtime_instance_id = Convert.ToString(Required(value, "runtime_instance_id")),
                episode_id = Convert.ToString(Required(value, "episode_id")),
                reset_id = Convert.ToString(Required(value, "reset_id")),
                state_id = Convert.ToInt64(Required(value, "state_id")),
                depth_id = Convert.ToString(Required(value, "depth_id")),
                sim_time_ns = Convert.ToUInt64(Required(value, "sim_time_ns")),
            };
        }

        private static Dictionary<string, object> RequiredMap(
            Dictionary<string, object> values, string key)
        {
            object raw = Required(values, key);
            Dictionary<string, object> typed = raw as Dictionary<string, object>;
            if (typed != null) return typed;
            IDictionary map = raw as IDictionary;
            if (map == null) throw new ArgumentException("snapshot wire field is not a map: " + key);
            var converted = new Dictionary<string, object>();
            foreach (DictionaryEntry entry in map) {
                string mapKey = entry.Key as string;
                if (mapKey == null) throw new ArgumentException("snapshot wire map key is not text");
                converted.Add(mapKey, entry.Value);
            }
            return converted;
        }

        private static object Required(Dictionary<string, object> values, string key)
        {
            object value;
            if (values == null || !values.TryGetValue(key, out value) || value == null)
                throw new ArgumentException("missing snapshot wire field: " + key);
            return value;
        }

        private static byte[] Bytes32(object value)
        {
            byte[] bytes = value as byte[];
            if (bytes == null || bytes.Length != 32)
                throw new ArgumentException("snapshot wire hash must be bytes32");
            return bytes;
        }

        private static void ValidateRequest(SnapshotRequestV4 request)
        {
            if (request == null) throw new ArgumentException("snapshot request is required");
            XMProtocolV4.CanonicalSnapshotRequest(request);
        }

        private static bool SameRef(ObservationRefV4 left, ObservationRefV4 right)
        {
            return left != null && right != null &&
                XMProtocolV4.CanonicalObservationRef(left).Length ==
                XMProtocolV4.CanonicalObservationRef(right).Length &&
                SameBytes(XMProtocolV4.CanonicalObservationRef(left), XMProtocolV4.CanonicalObservationRef(right));
        }

        private static bool SameBytes(byte[] left, byte[] right)
        {
            if (left == null || right == null || left.Length != right.Length) return false;
            for (int index = 0; index < left.Length; index++) if (left[index] != right[index]) return false;
            return true;
        }
    }
}
