using MessagePack;
using System;
using System.Collections.Generic;

namespace XMflight
{
    // Runtime envelope codec. The lifecycle hash remains the canonical
    // 15-field semantic payload hash; this codec adds the transport identity
    // field result_payload_hash for the 16-field reliable message.
    public static class PrimitiveExecutionResultWireCodec
    {
        public static byte[] SerializeResult(
            PrimitiveExecutionResultTransmission transmission)
        {
            if (transmission == null)
                throw new ArgumentException("result transmission is required");

            var result = new PrimitiveExecutionV4Result {
                schema_version = 4,
                message_type = "PrimitiveExecutionResult",
                runtime_instance_id = transmission.RuntimeInstanceId,
                execution_id = transmission.ExecutionId,
                status = transmission.Status,
                requested_frame_count = transmission.RequestedFrameCount,
                applied_frame_count = transmission.AppliedFrameCount,
                first_applied_state_id = transmission.FirstAppliedStateId,
                endpoint_state_id = transmission.EndpointStateId,
                last_applied_frame_index = transmission.LastAppliedFrameIndex,
                reason_code = transmission.ReasonCode,
                command_sequence_hash = transmission.CommandSequenceHash,
                endpoint_sim_time_ns = transmission.EndpointSimTimeNs,
                endpoint_observation_ref = transmission.EndpointObservationRef,
                result_generation = transmission.ResultGeneration,
            };

            byte[] canonical = XMProtocolV4.CanonicalResultPayload(result);
            byte[] payloadHash = XMProtocolV4.Sha256Bytes(canonical);
            if (!SameBytes(payloadHash, transmission.ResultPayloadHash))
                throw new ArgumentException("result payload hash mismatch");

            var wire = new Dictionary<string, object> {
                { "applied_frame_count", result.applied_frame_count },
                { "command_sequence_hash", result.command_sequence_hash },
                { "endpoint_observation_ref", result.endpoint_observation_ref == null
                    ? null
                    : new Dictionary<string, object> {
                        { "depth_id", result.endpoint_observation_ref.depth_id },
                        { "episode_id", result.endpoint_observation_ref.episode_id },
                        { "reset_id", result.endpoint_observation_ref.reset_id },
                        { "runtime_instance_id", result.endpoint_observation_ref.runtime_instance_id },
                        { "schema_version", result.endpoint_observation_ref.schema_version },
                        { "sim_time_ns", result.endpoint_observation_ref.sim_time_ns },
                        { "state_id", result.endpoint_observation_ref.state_id },
                    } },
                { "endpoint_sim_time_ns", result.endpoint_sim_time_ns },
                { "endpoint_state_id", result.endpoint_state_id },
                { "execution_id", result.execution_id },
                { "first_applied_state_id", result.first_applied_state_id },
                { "last_applied_frame_index", result.last_applied_frame_index },
                { "message_type", result.message_type },
                { "reason_code", result.reason_code },
                { "requested_frame_count", result.requested_frame_count },
                { "result_generation", result.result_generation },
                { "result_payload_hash", payloadHash },
                { "runtime_instance_id", result.runtime_instance_id },
                { "schema_version", result.schema_version },
                { "status", result.status },
            };
            return MessagePackSerializer.Serialize(wire);
        }

        public static PrimitiveExecutionV4Ack DeserializeAck(byte[] raw)
        {
            if (raw == null || raw.Length == 0)
                throw new ArgumentException("ack bytes are required");
            Dictionary<string, object> values =
                MessagePackSerializer.Deserialize<Dictionary<string, object>>(raw);
            return new PrimitiveExecutionV4Ack {
                schema_version = Convert.ToUInt32(RequiredValue(values, "schema_version")),
                message_type = Convert.ToString(RequiredValue(values, "message_type")),
                runtime_instance_id = Convert.ToString(
                    RequiredValue(values, "runtime_instance_id")),
                execution_id = Convert.ToUInt64(RequiredValue(values, "execution_id")),
                ack_status = Convert.ToString(RequiredValue(values, "ack_status")),
                result_payload_hash = RequiredBytes(values, "result_payload_hash"),
                command_sequence_hash = RequiredBytes(values, "command_sequence_hash"),
            };
        }

        private static object RequiredValue(Dictionary<string, object> values, string key)
        {
            object value;
            if (values == null || !values.TryGetValue(key, out value) || value == null)
                throw new ArgumentException("missing ack field: " + key);
            return value;
        }

        private static byte[] RequiredBytes(Dictionary<string, object> values, string key)
        {
            byte[] value = RequiredValue(values, key) as byte[];
            if (value == null)
                throw new ArgumentException("ack field is not binary: " + key);
            return value;
        }

        private static bool SameBytes(byte[] left, byte[] right)
        {
            if (left == null || right == null || left.Length != right.Length) return false;
            for (int index = 0; index < left.Length; ++index)
                if (left[index] != right[index]) return false;
            return true;
        }
    }
}
