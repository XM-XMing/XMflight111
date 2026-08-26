using System;
using System.Collections.Generic;

namespace XMflight
{
    public enum ReliableResultReceiveOutcome
    {
        ACCEPTED,
        DUPLICATE,
        PROTOCOL_ERROR,
    }

    public interface IPrimitiveExecutionResultAckSink
    {
        void SendAck(PrimitiveExecutionV4Ack ack);
    }

    public sealed class ReliableResultReceiver
    {
        private sealed class Receipt
        {
            public string runtimeInstanceId;
            public ulong executionId;
            public byte[] resultPayloadHash;
            public byte[] commandSequenceHash;
            public byte[] serializedPayload;
        }

        private readonly IPrimitiveExecutionResultAckSink ackSink;
        // This receipt table is deliberately process-local for L2d. Restart
        // recovery requires durable broker storage and is deferred.
        private readonly Dictionary<string, Receipt> receipts =
            new Dictionary<string, Receipt>();

        public ReliableResultReceiver(IPrimitiveExecutionResultAckSink ackSink)
        {
            if (ackSink == null) throw new ArgumentException("ACK sink is required");
            this.ackSink = ackSink;
        }

        public int NewResultCount { get; private set; }
        public int DuplicateResultCount { get; private set; }
        public int ProtocolErrorCount { get; private set; }

        public ReliableResultReceiveOutcome Receive(
            PrimitiveExecutionResultTransmission result)
        {
            if (!IsValidTransmission(result)) {
                ProtocolErrorCount += 1;
                return ReliableResultReceiveOutcome.PROTOCOL_ERROR;
            }

            string key = ReceiptKey(result.RuntimeInstanceId, result.ExecutionId);
            Receipt existing;
            if (receipts.TryGetValue(key, out existing)) {
                if (!SameReceipt(existing, result)) {
                    ProtocolErrorCount += 1;
                    return ReliableResultReceiveOutcome.PROTOCOL_ERROR;
                }
                DuplicateResultCount += 1;
                ackSink.SendAck(BuildAck(result));
                return ReliableResultReceiveOutcome.DUPLICATE;
            }

            receipts.Add(key, new Receipt {
                runtimeInstanceId = result.RuntimeInstanceId,
                executionId = result.ExecutionId,
                resultPayloadHash = result.ResultPayloadHash,
                commandSequenceHash = result.CommandSequenceHash,
                serializedPayload = result.SerializedCanonicalResultBytes,
            });
            NewResultCount += 1;
            ackSink.SendAck(BuildAck(result));
            return ReliableResultReceiveOutcome.ACCEPTED;
        }

        private static string ReceiptKey(string runtimeInstanceId, ulong executionId)
        {
            return runtimeInstanceId + "\n" + executionId.ToString();
        }

        private static PrimitiveExecutionV4Ack BuildAck(
            PrimitiveExecutionResultTransmission result)
        {
            return new PrimitiveExecutionV4Ack {
                schema_version = 4,
                runtime_instance_id = result.RuntimeInstanceId,
                execution_id = result.ExecutionId,
                ack_status = "DURABLE_RECEIVED",
                result_payload_hash = result.ResultPayloadHash,
                command_sequence_hash = result.CommandSequenceHash,
            };
        }

        private static bool IsValidTransmission(
            PrimitiveExecutionResultTransmission result)
        {
            if (result == null || string.IsNullOrEmpty(result.RuntimeInstanceId) ||
                result.CommandSequenceHash == null || result.CommandSequenceHash.Length != 32 ||
                result.ResultPayloadHash == null || result.ResultPayloadHash.Length != 32 ||
                result.SerializedCanonicalResultBytes == null)
                return false;
            return SameBytes(
                XMProtocolV4.Sha256Bytes(result.SerializedCanonicalResultBytes),
                result.ResultPayloadHash);
        }

        private static bool SameReceipt(
            Receipt existing,
            PrimitiveExecutionResultTransmission result)
        {
            return SameBytes(existing.resultPayloadHash, result.ResultPayloadHash) &&
                SameBytes(existing.commandSequenceHash, result.CommandSequenceHash) &&
                SameBytes(existing.serializedPayload,
                    result.SerializedCanonicalResultBytes);
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
