using System;

namespace XMflight
{
    public interface IPrimitiveExecutionResultTransport
    {
        bool IsConnected { get; }
        void Send(PrimitiveExecutionResultTransmission result);
    }

    public sealed class PrimitiveExecutionResultTransportAdapter
    {
        private readonly PrimitiveExecutionResultLifecycle lifecycle;
        private readonly IPrimitiveExecutionResultTransport transport;
        private PrimitiveExecutionResultTransmission currentResult;
        private bool disconnected;

        public PrimitiveExecutionResultTransportAdapter(
            PrimitiveExecutionResultLifecycle lifecycle,
            IPrimitiveExecutionResultTransport transport)
        {
            if (lifecycle == null) throw new ArgumentException("lifecycle is required");
            if (transport == null) throw new ArgumentException("result transport is required");
            this.lifecycle = lifecycle;
            this.transport = transport;
        }

        public int PendingResultCount { get { return lifecycle.HasPendingResult ? 1 : 0; } }
        public int SendAttemptCount { get; private set; }

        public void Record(PrimitiveExecutionResultTransmission result)
        {
            ValidateTransmission(result);
            PrimitiveExecutionResultTransmission pending = lifecycle.PendingResult;
            if (pending == null || !SameTransmission(pending, result))
                throw new PrimitiveExecutionLifecycleException(
                    "transport result is not the lifecycle pending result");

            if (currentResult != null &&
                SameExecution(currentResult, result) &&
                !SameTransmission(currentResult, result) &&
                lifecycle.State == PrimitiveExecutionLifecycleState.FINAL_RESULT_PENDING_ACK)
                throw new PrimitiveExecutionLifecycleException(
                    "pending result payload is immutable");

            bool alreadyRecorded = currentResult != null && SameTransmission(currentResult, result);
            currentResult = result;
            if (!alreadyRecorded) SendIfConnected(result);
        }

        public PrimitiveExecutionAckOutcome ReceiveAck(PrimitiveExecutionV4Ack ack)
        {
            return lifecycle.Acknowledge(ack);
        }

        public bool Poll(ulong nowMs)
        {
            if (!CanSend()) return false;
            PrimitiveExecutionResultTransmission retry = lifecycle.Poll(nowMs);
            if (retry == null) return false;
            EnsureCurrentResult(retry);
            SendIfConnected(retry);
            return true;
        }

        public bool ResendPending()
        {
            if (!CanSend() || !lifecycle.HasPendingResult) return false;
            PrimitiveExecutionResultTransmission pending = lifecycle.PendingResult;
            EnsureCurrentResult(pending);
            SendIfConnected(pending);
            return true;
        }

        public void OnDisconnected()
        {
            disconnected = true;
        }

        public bool OnReconnected()
        {
            disconnected = false;
            if (!CanSend() || !lifecycle.HasPendingResult) return false;
            PrimitiveExecutionResultTransmission pending = lifecycle.PendingResult;
            EnsureCurrentResult(pending);
            SendIfConnected(pending);
            return true;
        }

        private bool CanSend()
        {
            return !disconnected && transport.IsConnected;
        }

        private void SendIfConnected(PrimitiveExecutionResultTransmission result)
        {
            if (!CanSend()) return;
            transport.Send(result);
            SendAttemptCount += 1;
        }

        private void EnsureCurrentResult(PrimitiveExecutionResultTransmission result)
        {
            if (currentResult == null) {
                currentResult = result;
                return;
            }
            if (SameExecution(currentResult, result) &&
                !SameTransmission(currentResult, result))
                throw new PrimitiveExecutionLifecycleException(
                    "retry payload is not immutable");
            if (!SameExecution(currentResult, result)) currentResult = result;
        }

        private static void ValidateTransmission(PrimitiveExecutionResultTransmission result)
        {
            if (result == null)
                throw new PrimitiveExecutionLifecycleException("result is required");
            if (string.IsNullOrEmpty(result.RuntimeInstanceId) ||
                result.CommandSequenceHash == null || result.CommandSequenceHash.Length != 32 ||
                result.ResultPayloadHash == null || result.ResultPayloadHash.Length != 32 ||
                result.SerializedCanonicalResultBytes == null)
                throw new PrimitiveExecutionLifecycleException("result identity is incomplete");
            if (!SameBytes(
                    XMProtocolV4.Sha256Bytes(result.SerializedCanonicalResultBytes),
                    result.ResultPayloadHash))
                throw new PrimitiveExecutionLifecycleException(
                    "result payload hash does not match serialized payload");
        }

        private static bool SameTransmission(
            PrimitiveExecutionResultTransmission left,
            PrimitiveExecutionResultTransmission right)
        {
            return left.RuntimeInstanceId == right.RuntimeInstanceId &&
                left.ExecutionId == right.ExecutionId &&
                SameBytes(left.CommandSequenceHash, right.CommandSequenceHash) &&
                SameBytes(left.ResultPayloadHash, right.ResultPayloadHash) &&
                SameBytes(left.SerializedCanonicalResultBytes,
                    right.SerializedCanonicalResultBytes);
        }

        private static bool SameExecution(
            PrimitiveExecutionResultTransmission left,
            PrimitiveExecutionResultTransmission right)
        {
            return left.RuntimeInstanceId == right.RuntimeInstanceId &&
                left.ExecutionId == right.ExecutionId;
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
