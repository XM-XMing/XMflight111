using MessagePack;
using System;
using System.Collections;
using System.Collections.Generic;

namespace XMflight
{
    // Runtime envelope codec for the reliable v4 command channel.  The
    // received command object is validated by PrimitiveExecutionCommandAdmission;
    // retransmission always happens at the bridge using the original bytes.
    public static class PrimitiveExecutionCommandWireCodec
    {
        public static byte[] SerializeReady(string runtimeInstanceId)
        {
            return MessagePackSerializer.Serialize(new Dictionary<string, object> {
                { "message_type", "PrimitiveExecutionCommandReady" },
                { "runtime_instance_id", runtimeInstanceId },
                { "schema_version", (uint)4 },
            });
        }

        public static PrimitiveExecutionV4Command DeserializeCommand(byte[] raw)
        {
            if (raw == null || raw.Length == 0)
                throw new ArgumentException("command bytes are required");
            Dictionary<string, object> values =
                MessagePackSerializer.Deserialize<Dictionary<string, object>>(raw);
            var command = new PrimitiveExecutionV4Command {
                schema_version = Convert.ToUInt32(Required(values, "schema_version")),
                message_type = Convert.ToString(Required(values, "message_type")),
                runtime_instance_id = Convert.ToString(
                    Required(values, "runtime_instance_id")),
                execution_id = Convert.ToUInt64(Required(values, "execution_id")),
                command_sequence_hash = RequiredBytes(values, "command_sequence_hash"),
            };
            object[] frames = RequiredArray(values, "frames");
            foreach (object value in frames) {
                if (value == null) throw new ArgumentException("command frame is invalid");
                object[] actionValues = RequiredArray(value, "action");
                var action = new float[actionValues.Length];
                for (int index = 0; index < action.Length; ++index)
                    action[index] = Convert.ToSingle(actionValues[index]);
                command.frames.Add(new PrimitiveExecutionV4Frame {
                    frame_index = Convert.ToUInt32(Required(value, "frame_index")),
                    command_id = Convert.ToInt64(Required(value, "command_id")),
                    action = action,
                });
            }
            return command;
        }

        public static byte[] SerializeReceipt(PrimitiveExecutionV4CommandReceipt receipt)
        {
            if (receipt == null) throw new ArgumentException("command receipt is required");
            return MessagePackSerializer.Serialize(new Dictionary<string, object> {
                { "ack_status", receipt.ack_status },
                { "command_sequence_hash", receipt.command_sequence_hash },
                { "execution_id", receipt.execution_id },
                { "message_type", receipt.message_type },
                { "reason_code", receipt.reason_code },
                { "runtime_instance_id", receipt.runtime_instance_id },
                { "schema_version", receipt.schema_version },
            });
        }

        private static object Required(object values, string key)
        {
            object value = null;
            IDictionary<string, object> stringDictionary =
                values as IDictionary<string, object>;
            if (stringDictionary != null)
                stringDictionary.TryGetValue(key, out value);
            else {
                IDictionary dictionary = values as IDictionary;
                if (dictionary != null && dictionary.Contains(key))
                    value = dictionary[key];
            }
            if (value == null)
                throw new ArgumentException("missing command field: " + key);
            return value;
        }

        private static byte[] RequiredBytes(object values, string key)
        {
            byte[] value = Required(values, key) as byte[];
            if (value == null || value.Length != 32)
                throw new ArgumentException("command field must be bytes32: " + key);
            return value;
        }

        private static object[] RequiredArray(object values, string key)
        {
            object value = Required(values, key);
            object[] array = value as object[];
            if (array != null) return array;
            IList list = value as IList;
            if (list == null)
                throw new ArgumentException("command field must be array: " + key);
            array = new object[list.Count];
            list.CopyTo(array, 0);
            return array;
        }
    }
}
