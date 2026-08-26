// filename: Assets/Scripts/XMProtocol.cs
using MessagePack;
using System;
using System.Collections.Generic;
using System.Security.Cryptography;
using System.Text;

namespace XMflight
{
    public static class XMProtocol {
        public const int SchemaVersion = 3;
        public const int ModeVelocity = 0;
        public const int ModeTeleport = 1;
        public const int ModeStep = 2;
        public const int ModeTrajectory = 3;
        public const int ModePrimitiveExecution = 4;

        public const int ExecutionStatusNone = 0;
        public const int ExecutionStatusFrameApplied = 1;
        public const int ExecutionStatusComplete = 2;
        public const int ExecutionStatusFailed = 3;

        public const int DepthEncoding16UC1 = 1;
        public const int ByteOrderLittleEndian = 0;
        public const int FlagCollision = 1 << 0;
        public const int FlagAltitudeViolation = 1 << 1;

        // Identity fields are appended so existing state/depth consumers keep
        // their original indices and telemetry meaning.
        public const int DynamicsStateFieldCount = 17;
        public const int DepthFrameFieldCount = 19;
        public const int CommandFieldCount = 10;
        public const int CameraFieldCount = 8;
        public const int DepthMetaFieldCount = 5;

        public static class DynamicsStateIndex {
            public const int SchemaVersion = 0;
            public const int StateId = 1;
            public const int SimTimeNs = 2;
            public const int Flags = 3;
            public const int MinClearance = 4;
            public const int CurrPos = 5;
            public const int CurrRot = 6;
            public const int CurrVel = 7;
            public const int CurrAcc = 8;
            public const int FrontClearances = 9;
            public const int AppliedExecutionId = 10;
            public const int AppliedExecutionFrameIndex = 11;
            public const int AppliedCommandId = 12;
            public const int ExecutionStatus = 13;
            public const int EpisodeId = 14;
            public const int ResetId = 15;
            public const int RuntimeInstanceId = 16;
        }

        public static class DepthFrameIndex {
            public const int SchemaVersion = 0;
            public const int CaptureId = 1;
            public const int SimTimeNs = 2;
            public const int Flags = 3;
            public const int CapturePos = 4;
            public const int CaptureRot = 5;
            public const int CaptureVel = 6;
            public const int CaptureAcc = 7;
            public const int CaptureForward = 8;
            public const int Camera = 9;
            public const int DepthMeta = 10;
            public const int MinClearance = 11;
            public const int FrontClearances = 12;
            public const int EndpointStateId = 13;
            public const int EndpointPhysicsTimeNs = 14;
            public const int CaptureTimeNs = 15;
            public const int EpisodeId = 16;
            public const int ResetId = 17;
            public const int RuntimeInstanceId = 18;
        }

        public static class CommandIndex {
            public const int SchemaVersion = 0;
            public const int Mode = 1;
            public const int Action = 2;
            public const int Position = 3;
            public const int ClientTimeNs = 4;
            public const int CommandId = 5;
            public const int ExecutionId = 6;
            public const int ExecutionFrameIndex = 7;
            public const int ExecutionFrameCount = 8;
            public const int ExecutionFrames = 9;
        }
    }

    public sealed class ControlCommandMsg {
        public int schema_version;
        public int mode;
        public float[] action;
        public float[] position;
        public long client_time_ns;
        public long command_id;
        public long execution_id;
        public int execution_frame_index;
        public int execution_frame_count;
        public PrimitiveExecutionFrameMsg[] execution_frames;
    }

    public sealed class PrimitiveExecutionFrameMsg {
        public int frame_index;
        public long command_id;
        public float[] action;
    }

    public sealed class PrimitiveExecutionV4Frame {
        public uint frame_index;
        public long command_id;
        public float[] action;
    }

    public sealed class PrimitiveExecutionV4ObservationRef {
        public uint schema_version = 4;
        public string runtime_instance_id;
        public string episode_id;
        public string reset_id;
        public long state_id;
        public string depth_id;
        public ulong sim_time_ns;
    }

    public sealed class PrimitiveExecutionV4Result {
        public uint schema_version = 4;
        public string message_type = "PrimitiveExecutionResult";
        public string runtime_instance_id;
        public ulong execution_id;
        public string status;
        public uint requested_frame_count = 25;
        public uint applied_frame_count;
        public long? first_applied_state_id;
        public long? endpoint_state_id;
        public int last_applied_frame_index = -1;
        public string reason_code;
        public byte[] command_sequence_hash;
        public ulong? endpoint_sim_time_ns;
        public PrimitiveExecutionV4ObservationRef endpoint_observation_ref;
        public uint result_generation;
    }

    public sealed class PrimitiveExecutionV4Ack {
        public uint schema_version = 4;
        public string message_type = "PrimitiveExecutionResultAck";
        public string runtime_instance_id;
        public ulong execution_id;
        public string ack_status;
        public byte[] result_payload_hash;
        public byte[] command_sequence_hash;
    }

    // Exact endpoint identity for reliable observation snapshot retrieval.
    // This intentionally does not alter the existing three-field result ref.
    public sealed class ObservationRefV4 {
        public uint schema_version = 4;
        public string runtime_instance_id;
        public string episode_id;
        public string reset_id;
        public long state_id;
        public string depth_id;
        public ulong sim_time_ns;
    }

    public sealed class EndpointObservationSnapshotV4 {
        public ObservationRefV4 observation_ref;
        public byte[] state_bytes;
        public byte[] depth_bytes;
        // Transport identity; excluded from CanonicalEndpointObservationSnapshot.
        public byte[] snapshot_hash;
    }

    public sealed class SnapshotRequestV4 {
        public ObservationRefV4 observation_ref;
        public ulong execution_id;
        public byte[] result_payload_hash;
        public byte[] command_sequence_hash;
    }

    public sealed class SnapshotAckV4 {
        public ObservationRefV4 observation_ref;
        public byte[] snapshot_hash;
    }

    public enum ObservationSnapshotV4IdentityOutcome {
        First,
        Duplicate,
    }

    public static class XMProtocolV4 {
        private sealed class Writer {
            private readonly List<byte> bytes = new List<byte>();

            private void U8(byte value) { bytes.Add(value); }

            private void U16(ushort value) {
                U8((byte)(value >> 8));
                U8((byte)value);
            }

            private void U32(uint value) {
                U8((byte)(value >> 24));
                U8((byte)(value >> 16));
                U8((byte)(value >> 8));
                U8((byte)value);
            }

            private void U64(ulong value) {
                U8((byte)(value >> 56));
                U8((byte)(value >> 48));
                U8((byte)(value >> 40));
                U8((byte)(value >> 32));
                U8((byte)(value >> 24));
                U8((byte)(value >> 16));
                U8((byte)(value >> 8));
                U8((byte)value);
            }

            internal void MapHeader(uint count) {
                if (count <= 15) U8((byte)(0x80 | count));
                else if (count <= ushort.MaxValue) { U8(0xde); U16((ushort)count); }
                else { U8(0xdf); U32(count); }
            }

            internal void ArrayHeader(uint count) {
                if (count <= 15) U8((byte)(0x90 | count));
                else if (count <= ushort.MaxValue) { U8(0xdc); U16((ushort)count); }
                else { U8(0xdd); U32(count); }
            }

            internal void Nil() { U8(0xc0); }

            internal void String(string value) {
                if (value == null) throw new ArgumentException("v4 string is required");
                byte[] encoded = Encoding.UTF8.GetBytes(value);
                if (encoded.Length <= byte.MaxValue) { U8(0xd9); U8((byte)encoded.Length); }
                else if (encoded.Length <= ushort.MaxValue) { U8(0xda); U16((ushort)encoded.Length); }
                else { U8(0xdb); U32((uint)encoded.Length); }
                bytes.AddRange(encoded);
            }

            internal void Binary32(byte[] value) {
                if (value == null || value.Length != 32)
                    throw new ArgumentException("v4 binary fields must be bytes32");
                U8(0xc4);
                U8(32);
                bytes.AddRange(value);
            }

            internal void Binary(byte[] value) {
                if (value == null) throw new ArgumentException("v4 binary fields must be bytes");
                if (value.Length <= byte.MaxValue) { U8(0xc4); U8((byte)value.Length); }
                else if (value.Length <= ushort.MaxValue) { U8(0xc5); U16((ushort)value.Length); }
                else { U8(0xc6); U32((uint)value.Length); }
                bytes.AddRange(value);
            }

            internal void UInt32(uint value) { U8(0xce); U32(value); }

            internal void UInt64(ulong value) { U8(0xcf); U64(value); }

            internal void Int32(int value) { U8(0xd2); U32(unchecked((uint)value)); }

            internal void Int64(long value) { U8(0xd3); U64(unchecked((ulong)value)); }

            internal void Float32(float value) {
                if (float.IsNaN(value) || float.IsInfinity(value))
                    throw new ArgumentException("v4 action values reject NaN and infinity");
                byte[] encoded = BitConverter.GetBytes(value);
                U8(0xca);
                if (BitConverter.IsLittleEndian) Array.Reverse(encoded);
                bytes.AddRange(encoded);
            }

            internal byte[] ToArray() { return bytes.ToArray(); }
        }

        private static void Require(bool condition, string message) {
            if (!condition) throw new ArgumentException(message);
        }

        private static void OptionalInt64(Writer writer, long? value) {
            if (value.HasValue) writer.Int64(value.Value); else writer.Nil();
        }

        private static void OptionalUInt64(Writer writer, ulong? value) {
            if (value.HasValue) writer.UInt64(value.Value); else writer.Nil();
        }

        private static void ValidateObservationRef(ObservationRefV4 observationRef) {
            Require(observationRef != null, "observation_ref is required");
            Require(observationRef.schema_version == 4, "observation schema_version must be 4");
            Require(!string.IsNullOrEmpty(observationRef.runtime_instance_id),
                "observation runtime_instance_id is required");
            Require(!string.IsNullOrEmpty(observationRef.episode_id),
                "observation episode_id is required");
            Require(!string.IsNullOrEmpty(observationRef.reset_id),
                "observation reset_id is required");
            Require(observationRef.state_id >= 0, "observation state_id must be non-negative");
            Require(!string.IsNullOrEmpty(observationRef.depth_id),
                "observation depth_id is required");
        }

        private static void AppendObservationRef(Writer writer, ObservationRefV4 observationRef) {
            ValidateObservationRef(observationRef);
            writer.MapHeader(7);
            writer.String("depth_id");
            writer.String(observationRef.depth_id);
            writer.String("episode_id");
            writer.String(observationRef.episode_id);
            writer.String("reset_id");
            writer.String(observationRef.reset_id);
            writer.String("runtime_instance_id");
            writer.String(observationRef.runtime_instance_id);
            writer.String("schema_version");
            writer.UInt32(observationRef.schema_version);
            writer.String("sim_time_ns");
            writer.UInt64(observationRef.sim_time_ns);
            writer.String("state_id");
            writer.Int64(observationRef.state_id);
        }

        private static void ValidateResult(PrimitiveExecutionV4Result result) {
            Require(result != null, "result is required");
            Require(result.schema_version == 4, "schema_version must be 4");
            Require(result.message_type == "PrimitiveExecutionResult", "message_type mismatch");
            Require(!string.IsNullOrEmpty(result.runtime_instance_id), "runtime_instance_id is required");
            Require(result.requested_frame_count == 25, "requested_frame_count mismatch");
            Require(result.command_sequence_hash != null && result.command_sequence_hash.Length == 32,
                "command_sequence_hash must be bytes32");
            Require(result.result_generation == 0, "result_generation must remain zero");
            if (result.status == "COMPLETE") {
                Require(result.applied_frame_count == 25, "COMPLETE applied count mismatch");
                Require(result.last_applied_frame_index == 24, "COMPLETE last frame mismatch");
                Require(result.first_applied_state_id.HasValue && result.endpoint_state_id.HasValue,
                    "COMPLETE state ids are required");
                Require(result.endpoint_state_id.Value == result.first_applied_state_id.Value + 24,
                    "COMPLETE endpoint state mismatch");
                Require(result.reason_code == "NONE", "COMPLETE reason must be NONE");
                Require(result.endpoint_sim_time_ns.HasValue && result.endpoint_observation_ref != null,
                    "COMPLETE endpoint observation is required");
                Require(result.endpoint_observation_ref.schema_version == 4,
                    "COMPLETE observation schema_version mismatch");
                Require(result.endpoint_observation_ref.runtime_instance_id == result.runtime_instance_id,
                    "COMPLETE observation runtime identity mismatch");
                Require(!string.IsNullOrEmpty(result.endpoint_observation_ref.episode_id),
                    "COMPLETE episode id is required");
                Require(!string.IsNullOrEmpty(result.endpoint_observation_ref.reset_id),
                    "COMPLETE reset id is required");
                Require(!string.IsNullOrEmpty(result.endpoint_observation_ref.depth_id),
                    "COMPLETE depth id is required");
                Require(result.endpoint_observation_ref.state_id == result.endpoint_state_id.Value,
                    "COMPLETE observation state mismatch");
                Require(result.endpoint_observation_ref.sim_time_ns == result.endpoint_sim_time_ns.Value,
                    "COMPLETE observation time mismatch");
            } else if (result.status == "REJECTED") {
                Require(result.applied_frame_count == 0, "REJECTED applied count must be zero");
                Require(result.last_applied_frame_index == -1, "REJECTED last frame must be -1");
                Require(!result.first_applied_state_id.HasValue && !result.endpoint_state_id.HasValue,
                    "REJECTED must not claim state ids");
                Require(!result.endpoint_sim_time_ns.HasValue && result.endpoint_observation_ref == null,
                    "REJECTED must not claim endpoint observation");
                Require(!string.IsNullOrEmpty(result.reason_code) && result.reason_code != "NONE",
                    "REJECTED reason is required");
            } else if (result.status == "CANCELLED" || result.status == "FAILED") {
                bool physicsCompleteObservationFailed =
                    result.status == "FAILED" &&
                    result.reason_code == "PHYSICS_COMPLETE_OBSERVATION_FAILED";
                bool environmentTerminalWithObservation =
                    result.status == "FAILED" && result.reason_code == "COLLISION";
                if (physicsCompleteObservationFailed) {
                    Require(result.applied_frame_count == 25,
                        "observation failure must preserve all applied frames");
                    Require(result.last_applied_frame_index == 24,
                        "observation failure last frame mismatch");
                } else {
                    Require(result.applied_frame_count < 25,
                        "interrupted result cannot claim complete prefix");
                }
                Require(result.last_applied_frame_index == (int)result.applied_frame_count - 1,
                    "interrupted last frame mismatch");
                Require(!string.IsNullOrEmpty(result.reason_code) && result.reason_code != "NONE",
                    "interrupted reason is required");
                if (environmentTerminalWithObservation) {
                    Require(result.endpoint_state_id.HasValue && result.endpoint_sim_time_ns.HasValue &&
                        result.endpoint_observation_ref != null,
                        "environment terminal observation is required");
                    Require(result.endpoint_observation_ref.schema_version == 4,
                        "terminal observation schema_version mismatch");
                    Require(result.endpoint_observation_ref.runtime_instance_id == result.runtime_instance_id,
                        "terminal observation runtime identity mismatch");
                    Require(!string.IsNullOrEmpty(result.endpoint_observation_ref.episode_id),
                        "terminal observation episode id is required");
                    Require(!string.IsNullOrEmpty(result.endpoint_observation_ref.reset_id),
                        "terminal observation reset id is required");
                    Require(!string.IsNullOrEmpty(result.endpoint_observation_ref.depth_id),
                        "terminal observation depth id is required");
                    Require(result.endpoint_observation_ref.state_id == result.endpoint_state_id.Value,
                        "terminal observation state mismatch");
                    Require(result.endpoint_observation_ref.sim_time_ns == result.endpoint_sim_time_ns.Value,
                        "terminal observation time mismatch");
                } else {
                    Require(!result.endpoint_state_id.HasValue && !result.endpoint_sim_time_ns.HasValue &&
                        result.endpoint_observation_ref == null, "interrupted endpoint must be absent");
                }
                Require(result.applied_frame_count == 0 || result.first_applied_state_id.HasValue,
                    "applied prefix requires first state id");
            } else {
                throw new ArgumentException("unknown v4 result status");
            }
        }

        public static void ValidateAck(PrimitiveExecutionV4Ack ack) {
            Require(ack != null, "ack is required");
            Require(ack.schema_version == 4, "ack schema_version must be 4");
            Require(ack.message_type == "PrimitiveExecutionResultAck", "ack message_type mismatch");
            Require(!string.IsNullOrEmpty(ack.runtime_instance_id),
                "ack runtime_instance_id is required");
            Require(ack.ack_status == "DURABLE_RECEIVED", "ack status mismatch");
            Require(ack.result_payload_hash != null && ack.result_payload_hash.Length == 32,
                "ack result_payload_hash must be bytes32");
            Require(ack.command_sequence_hash != null && ack.command_sequence_hash.Length == 32,
                "ack command_sequence_hash must be bytes32");
        }

        public static byte[] CanonicalCommandSequence(IList<PrimitiveExecutionV4Frame> frames) {
            Require(frames != null && frames.Count > 0, "command sequence must not be empty");
            Writer writer = new Writer();
            writer.ArrayHeader((uint)frames.Count);
            for (int index = 0; index < frames.Count; ++index) {
                PrimitiveExecutionV4Frame frame = frames[index];
                Require(frame != null && frame.action != null, "command frame is incomplete");
                writer.MapHeader(3);
                writer.String("action");
                writer.ArrayHeader((uint)frame.action.Length);
                foreach (float value in frame.action) writer.Float32(value);
                writer.String("command_id");
                writer.Int64(frame.command_id);
                writer.String("frame_index");
                writer.UInt32(frame.frame_index);
            }
            return writer.ToArray();
        }

        public static byte[] CanonicalResultPayload(PrimitiveExecutionV4Result result) {
            ValidateResult(result);
            Writer writer = new Writer();
            writer.MapHeader(15);
            writer.String("applied_frame_count");
            writer.UInt32(result.applied_frame_count);
            writer.String("command_sequence_hash");
            writer.Binary32(result.command_sequence_hash);
            writer.String("endpoint_observation_ref");
            if (result.endpoint_observation_ref == null) {
                writer.Nil();
            } else {
                writer.MapHeader(7);
                writer.String("depth_id");
                writer.String(result.endpoint_observation_ref.depth_id);
                writer.String("episode_id");
                writer.String(result.endpoint_observation_ref.episode_id);
                writer.String("reset_id");
                writer.String(result.endpoint_observation_ref.reset_id);
                writer.String("runtime_instance_id");
                writer.String(result.endpoint_observation_ref.runtime_instance_id);
                writer.String("schema_version");
                writer.UInt32(result.endpoint_observation_ref.schema_version);
                writer.String("sim_time_ns");
                writer.UInt64(result.endpoint_observation_ref.sim_time_ns);
                writer.String("state_id");
                writer.Int64(result.endpoint_observation_ref.state_id);
            }
            writer.String("endpoint_sim_time_ns");
            OptionalUInt64(writer, result.endpoint_sim_time_ns);
            writer.String("endpoint_state_id");
            OptionalInt64(writer, result.endpoint_state_id);
            writer.String("execution_id");
            writer.UInt64(result.execution_id);
            writer.String("first_applied_state_id");
            OptionalInt64(writer, result.first_applied_state_id);
            writer.String("last_applied_frame_index");
            writer.Int32(result.last_applied_frame_index);
            writer.String("message_type");
            writer.String(result.message_type);
            writer.String("reason_code");
            writer.String(result.reason_code);
            writer.String("requested_frame_count");
            writer.UInt32(result.requested_frame_count);
            writer.String("result_generation");
            writer.UInt32(result.result_generation);
            writer.String("runtime_instance_id");
            writer.String(result.runtime_instance_id);
            writer.String("schema_version");
            writer.UInt32(result.schema_version);
            writer.String("status");
            writer.String(result.status);
            return writer.ToArray();
        }

        public static byte[] CanonicalAckPayload(PrimitiveExecutionV4Ack ack) {
            ValidateAck(ack);
            Writer writer = new Writer();
            writer.MapHeader(7);
            writer.String("ack_status");
            writer.String(ack.ack_status);
            writer.String("command_sequence_hash");
            writer.Binary32(ack.command_sequence_hash);
            writer.String("execution_id");
            writer.UInt64(ack.execution_id);
            writer.String("message_type");
            writer.String(ack.message_type);
            writer.String("result_payload_hash");
            writer.Binary32(ack.result_payload_hash);
            writer.String("runtime_instance_id");
            writer.String(ack.runtime_instance_id);
            writer.String("schema_version");
            writer.UInt32(ack.schema_version);
            return writer.ToArray();
        }

        public static byte[] CanonicalObservationRef(ObservationRefV4 observationRef) {
            Writer writer = new Writer();
            AppendObservationRef(writer, observationRef);
            return writer.ToArray();
        }

        public static byte[] CanonicalEndpointObservationSnapshot(
            EndpointObservationSnapshotV4 snapshot) {
            Require(snapshot != null, "snapshot is required");
            Require(snapshot.state_bytes != null, "snapshot state_bytes are required");
            Require(snapshot.depth_bytes != null, "snapshot depth_bytes are required");
            Writer writer = new Writer();
            writer.MapHeader(3);
            writer.String("depth_bytes");
            writer.Binary(snapshot.depth_bytes);
            writer.String("observation_ref");
            AppendObservationRef(writer, snapshot.observation_ref);
            writer.String("state_bytes");
            writer.Binary(snapshot.state_bytes);
            return writer.ToArray();
        }

        public static byte[] CanonicalSnapshotRequest(SnapshotRequestV4 request) {
            Require(request != null, "snapshot request is required");
            Require(request.result_payload_hash != null && request.result_payload_hash.Length == 32,
                "snapshot request result_payload_hash must be bytes32");
            Require(request.command_sequence_hash != null && request.command_sequence_hash.Length == 32,
                "snapshot request command_sequence_hash must be bytes32");
            Writer writer = new Writer();
            writer.MapHeader(4);
            writer.String("command_sequence_hash");
            writer.Binary32(request.command_sequence_hash);
            writer.String("execution_id");
            writer.UInt64(request.execution_id);
            writer.String("observation_ref");
            AppendObservationRef(writer, request.observation_ref);
            writer.String("result_payload_hash");
            writer.Binary32(request.result_payload_hash);
            return writer.ToArray();
        }

        public static byte[] CanonicalSnapshotAck(SnapshotAckV4 ack) {
            Require(ack != null, "snapshot ack is required");
            Require(ack.snapshot_hash != null && ack.snapshot_hash.Length == 32,
                "snapshot ack snapshot_hash must be bytes32");
            Writer writer = new Writer();
            writer.MapHeader(2);
            writer.String("observation_ref");
            AppendObservationRef(writer, ack.observation_ref);
            writer.String("snapshot_hash");
            writer.Binary32(ack.snapshot_hash);
            return writer.ToArray();
        }

        public static byte[] SnapshotHash(EndpointObservationSnapshotV4 snapshot) {
            return Sha256Bytes(CanonicalEndpointObservationSnapshot(snapshot));
        }

        public static byte[] Sha256Bytes(byte[] bytes) {
            if (bytes == null) throw new ArgumentException("bytes are required");
            using (SHA256 sha = SHA256.Create()) return sha.ComputeHash(bytes);
        }

        public static string Sha256Hex(byte[] bytes) {
            byte[] digest = Sha256Bytes(bytes);
            StringBuilder output = new StringBuilder(digest.Length * 2);
            foreach (byte value in digest) output.Append(value.ToString("x2"));
            return output.ToString();
        }
    }

    public sealed class ObservationSnapshotV4IdentityRegistry {
        private readonly Dictionary<string, byte[]> snapshotHashes = new Dictionary<string, byte[]>();

        public ObservationSnapshotV4IdentityOutcome Observe(EndpointObservationSnapshotV4 snapshot) {
            if (snapshot == null || snapshot.snapshot_hash == null || snapshot.snapshot_hash.Length != 32)
                throw new ArgumentException("snapshot_hash must be bytes32");
            string key = Convert.ToBase64String(XMProtocolV4.CanonicalObservationRef(snapshot.observation_ref));
            byte[] previous;
            if (!snapshotHashes.TryGetValue(key, out previous)) {
                snapshotHashes[key] = (byte[])snapshot.snapshot_hash.Clone();
                return ObservationSnapshotV4IdentityOutcome.First;
            }
            if (!EqualBytes(previous, snapshot.snapshot_hash))
                throw new ArgumentException("snapshot_hash conflict for observation_ref");
            return ObservationSnapshotV4IdentityOutcome.Duplicate;
        }

        private static bool EqualBytes(byte[] left, byte[] right) {
            if (left.Length != right.Length) return false;
            for (int index = 0; index < left.Length; ++index)
                if (left[index] != right[index]) return false;
            return true;
        }
    }
}
