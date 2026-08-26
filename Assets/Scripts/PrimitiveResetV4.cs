using System;

namespace XMflight
{
    public sealed class PrimitiveResetV4Request
    {
        public uint schema_version = 4;
        public string message_type = "PrimitiveResetRequest";
        public string runtime_instance_id;
        public string episode_id;
        public string reset_id;
        public float[] start;
        public float[] goal;
    }

    public sealed class PrimitiveResetV4Complete
    {
        public uint schema_version = 4;
        public string message_type = "PrimitiveResetComplete";
        public string runtime_instance_id;
        public string episode_id;
        public string reset_id;
        public ObservationRefV4 observation_ref;
    }

    public sealed class PrimitiveResetV4ReceivedAck
    {
        public uint schema_version = 4;
        public string message_type = "PrimitiveResetReceivedAck";
        public string runtime_instance_id;
        public string episode_id;
        public string reset_id;
    }

    public static class PrimitiveResetV4WireCodec
    {
        public static byte[] SerializeRequest(PrimitiveResetV4Request request)
        {
            if (request == null || request.start == null || request.start.Length != 3 ||
                request.goal == null || request.goal.Length != 3)
                throw new ArgumentException("reset request pose/goal must contain three values");
            return MessagePack.MessagePackSerializer.Serialize(new System.Collections.Generic.Dictionary<string, object> {
                { "episode_id", request.episode_id },
                { "goal", request.goal },
                { "message_type", request.message_type },
                { "reset_id", request.reset_id },
                { "runtime_instance_id", request.runtime_instance_id },
                { "schema_version", request.schema_version },
                { "start", request.start },
            });
        }

        public static PrimitiveResetV4Request DeserializeRequest(byte[] raw)
        {
            var values = MessagePack.MessagePackSerializer.Deserialize<System.Collections.Generic.Dictionary<string, object>>(raw);
            return new PrimitiveResetV4Request {
                schema_version = Convert.ToUInt32(Required(values, "schema_version")),
                message_type = Convert.ToString(Required(values, "message_type")),
                runtime_instance_id = Convert.ToString(Required(values, "runtime_instance_id")),
                episode_id = Convert.ToString(Required(values, "episode_id")),
                reset_id = Convert.ToString(Required(values, "reset_id")),
                start = Array3(values, "start"),
                goal = Array3(values, "goal"),
            };
        }

        public static byte[] SerializeComplete(PrimitiveResetV4Complete complete)
        {
            if (complete == null || complete.observation_ref == null)
                throw new ArgumentException("reset completion observation_ref is required");
            return MessagePack.MessagePackSerializer.Serialize(new System.Collections.Generic.Dictionary<string, object> {
                { "episode_id", complete.episode_id },
                { "message_type", complete.message_type },
                { "observation_ref", new System.Collections.Generic.Dictionary<string, object> {
                    { "depth_id", complete.observation_ref.depth_id },
                    { "episode_id", complete.observation_ref.episode_id },
                    { "reset_id", complete.observation_ref.reset_id },
                    { "runtime_instance_id", complete.observation_ref.runtime_instance_id },
                    { "schema_version", complete.observation_ref.schema_version },
                    { "sim_time_ns", complete.observation_ref.sim_time_ns },
                    { "state_id", complete.observation_ref.state_id },
                }},
                { "reset_id", complete.reset_id },
                { "runtime_instance_id", complete.runtime_instance_id },
                { "schema_version", complete.schema_version },
            });
        }

        public static byte[] SerializeReceivedAck(PrimitiveResetV4ReceivedAck ack)
        {
            return MessagePack.MessagePackSerializer.Serialize(new System.Collections.Generic.Dictionary<string, object> {
                { "episode_id", ack.episode_id },
                { "message_type", ack.message_type },
                { "reset_id", ack.reset_id },
                { "runtime_instance_id", ack.runtime_instance_id },
                { "schema_version", ack.schema_version },
            });
        }

        private static object Required(System.Collections.Generic.IDictionary<string, object> values, string key)
        {
            object value;
            if (!values.TryGetValue(key, out value) || value == null)
                throw new ArgumentException("missing reset field: " + key);
            return value;
        }

        private static float[] Array3(System.Collections.Generic.IDictionary<string, object> values, string key)
        {
            object value = Required(values, key);
            var list = value as System.Collections.IList;
            if (list == null || list.Count != 3)
                throw new ArgumentException("reset field must contain three values: " + key);
            return new[] { Convert.ToSingle(list[0]), Convert.ToSingle(list[1]), Convert.ToSingle(list[2]) };
        }
    }
}
