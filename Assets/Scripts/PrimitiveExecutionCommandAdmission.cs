using System;
using System.Collections.Generic;

namespace XMflight
{
    public enum PrimitiveExecutionCommandAdmissionOutcome
    {
        ACCEPTED,
        DUPLICATE,
        PROTOCOL_ERROR,
        REJECTED,
    }

    public sealed class PrimitiveExecutionV4Command
    {
        public uint schema_version = 4;
        public string message_type = "PrimitiveExecutionCommand";
        public string runtime_instance_id;
        public ulong execution_id;
        public byte[] command_sequence_hash;
        public List<PrimitiveExecutionV4Frame> frames =
            new List<PrimitiveExecutionV4Frame>();
    }

    public sealed class PrimitiveExecutionV4CommandReceipt
    {
        public uint schema_version = 4;
        public string message_type = "PrimitiveExecutionCommandReceiptAck";
        public string runtime_instance_id;
        public ulong execution_id;
        public string ack_status;
        public string reason_code;
        public byte[] command_sequence_hash;
    }

    public sealed class PrimitiveExecutionCommandAdmissionResult
    {
        internal PrimitiveExecutionCommandAdmissionResult(
            PrimitiveExecutionCommandAdmissionOutcome outcome,
            PrimitiveExecutionV4CommandReceipt receipt)
        {
            Outcome = outcome;
            Receipt = receipt;
        }

        public PrimitiveExecutionCommandAdmissionOutcome Outcome { get; private set; }
        public PrimitiveExecutionV4CommandReceipt Receipt { get; private set; }
    }

    public sealed class PrimitiveExecutionCommandAdmission
    {
        private readonly string expectedRuntimeInstanceId;
        private readonly Dictionary<string, byte[]> admittedHashes =
            new Dictionary<string, byte[]>();

        public PrimitiveExecutionCommandAdmission(string runtimeInstanceId)
        {
            if (String.IsNullOrEmpty(runtimeInstanceId))
                throw new ArgumentException("runtime instance id is required");
            expectedRuntimeInstanceId = runtimeInstanceId;
        }

        public int PhysicalExecutionCount { get; private set; }

        public PrimitiveExecutionCommandAdmissionResult Register(
            PrimitiveExecutionV4Command command)
        {
            if (command == null || command.runtime_instance_id != expectedRuntimeInstanceId)
                return Result(PrimitiveExecutionCommandAdmissionOutcome.REJECTED, command,
                    "RUNTIME_INSTANCE_MISMATCH");
            Validate(command);

            string key = command.runtime_instance_id + "\n" + command.execution_id;
            byte[] previous;
            if (admittedHashes.TryGetValue(key, out previous))
            {
                if (!EqualBytes(previous, command.command_sequence_hash))
                    return Result(PrimitiveExecutionCommandAdmissionOutcome.PROTOCOL_ERROR,
                        command, "COMMAND_HASH_CONFLICT");
                return Result(PrimitiveExecutionCommandAdmissionOutcome.DUPLICATE,
                    command, "DUPLICATE_COMMAND");
            }

            admittedHashes[key] = (byte[])command.command_sequence_hash.Clone();
            PhysicalExecutionCount += 1;
            return Result(PrimitiveExecutionCommandAdmissionOutcome.ACCEPTED,
                command, "NONE");
        }

        private static PrimitiveExecutionCommandAdmissionResult Result(
            PrimitiveExecutionCommandAdmissionOutcome outcome,
            PrimitiveExecutionV4Command command,
            string reason)
        {
            PrimitiveExecutionV4CommandReceipt receipt = null;
            if (command != null)
            {
                receipt = new PrimitiveExecutionV4CommandReceipt {
                    runtime_instance_id = command.runtime_instance_id,
                    execution_id = command.execution_id,
                    ack_status = outcome == PrimitiveExecutionCommandAdmissionOutcome.ACCEPTED
                        ? "ACCEPTED"
                        : outcome == PrimitiveExecutionCommandAdmissionOutcome.DUPLICATE
                            ? "DUPLICATE"
                            : outcome.ToString(),
                    reason_code = reason,
                    command_sequence_hash = command.command_sequence_hash == null
                        ? null : (byte[])command.command_sequence_hash.Clone(),
                };
            }
            return new PrimitiveExecutionCommandAdmissionResult(outcome, receipt);
        }

        private static void Validate(PrimitiveExecutionV4Command command)
        {
            if (command.schema_version != 4 ||
                command.message_type != "PrimitiveExecutionCommand")
                throw new ArgumentException("command schema or message type mismatch");
            if (command.frames == null || command.frames.Count != 25)
                throw new ArgumentException("command frame count mismatch");
            if (command.command_sequence_hash == null ||
                command.command_sequence_hash.Length != 32)
                throw new ArgumentException("command hash must be bytes32");
            for (uint index = 0; index < command.frames.Count; ++index)
                if (command.frames[(int)index].frame_index != index)
                    throw new ArgumentException("command frame indices must be contiguous");
            byte[] actual = XMProtocolV4.Sha256Bytes(
                XMProtocolV4.CanonicalCommandSequence(command.frames));
            if (!EqualBytes(actual, command.command_sequence_hash))
                throw new ArgumentException("command sequence hash mismatch");
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
