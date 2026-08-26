using System;

namespace XMflight
{
    public enum PrimitiveExecutionLifecycleState
    {
        IDLE,
        EXECUTING,
        FINAL_RESULT_PENDING_ACK,
        ACKED,
    }

    public enum PrimitiveExecutionAckOutcome
    {
        REJECTED,
        ACCEPTED,
        DUPLICATE,
    }

    public sealed class PrimitiveExecutionLifecycleException : InvalidOperationException
    {
        public PrimitiveExecutionLifecycleException(string message) : base(message) { }
    }

    public sealed class PrimitiveExecutionResultTransmission
    {
        private readonly string runtimeInstanceId;
        private readonly ulong executionId;
        private readonly string status;
        private readonly uint resultGeneration;
        private readonly uint requestedFrameCount;
        private readonly uint appliedFrameCount;
        private readonly long? firstAppliedStateId;
        private readonly long? endpointStateId;
        private readonly int lastAppliedFrameIndex;
        private readonly string reasonCode;
        private readonly ulong? endpointSimTimeNs;
        private readonly PrimitiveExecutionV4ObservationRef endpointObservationRef;
        private readonly byte[] commandSequenceHash;
        private readonly byte[] resultPayloadHash;
        private readonly byte[] serializedCanonicalResultBytes;

        internal PrimitiveExecutionResultTransmission(
            string runtimeInstanceId,
            ulong executionId,
            string status,
            uint resultGeneration,
            uint requestedFrameCount,
            uint appliedFrameCount,
            long? firstAppliedStateId,
            long? endpointStateId,
            int lastAppliedFrameIndex,
            string reasonCode,
            ulong? endpointSimTimeNs,
            PrimitiveExecutionV4ObservationRef endpointObservationRef,
            byte[] commandSequenceHash,
            byte[] resultPayloadHash,
            byte[] serializedCanonicalResultBytes)
        {
            this.runtimeInstanceId = runtimeInstanceId;
            this.executionId = executionId;
            this.status = status;
            this.resultGeneration = resultGeneration;
            this.requestedFrameCount = requestedFrameCount;
            this.appliedFrameCount = appliedFrameCount;
            this.firstAppliedStateId = firstAppliedStateId;
            this.endpointStateId = endpointStateId;
            this.lastAppliedFrameIndex = lastAppliedFrameIndex;
            this.reasonCode = reasonCode;
            this.endpointSimTimeNs = endpointSimTimeNs;
            this.endpointObservationRef = CloneObservationRef(endpointObservationRef);
            this.commandSequenceHash = (byte[])commandSequenceHash.Clone();
            this.resultPayloadHash = (byte[])resultPayloadHash.Clone();
            this.serializedCanonicalResultBytes = (byte[])serializedCanonicalResultBytes.Clone();
        }

        public string RuntimeInstanceId { get { return runtimeInstanceId; } }
        public ulong ExecutionId { get { return executionId; } }
        public string Status { get { return status; } }
        public uint ResultGeneration { get { return resultGeneration; } }
        public uint RequestedFrameCount { get { return requestedFrameCount; } }
        public uint AppliedFrameCount { get { return appliedFrameCount; } }
        public long? FirstAppliedStateId { get { return firstAppliedStateId; } }
        public long? EndpointStateId { get { return endpointStateId; } }
        public int LastAppliedFrameIndex { get { return lastAppliedFrameIndex; } }
        public string ReasonCode { get { return reasonCode; } }
        public ulong? EndpointSimTimeNs { get { return endpointSimTimeNs; } }
        public PrimitiveExecutionV4ObservationRef EndpointObservationRef {
            get { return CloneObservationRef(endpointObservationRef); }
        }

        public byte[] CommandSequenceHash { get { return (byte[])commandSequenceHash.Clone(); } }
        public byte[] ResultPayloadHash { get { return (byte[])resultPayloadHash.Clone(); } }
        public byte[] SerializedCanonicalResultBytes {
            get { return (byte[])serializedCanonicalResultBytes.Clone(); }
        }

        private static PrimitiveExecutionV4ObservationRef CloneObservationRef(
            PrimitiveExecutionV4ObservationRef source)
        {
            if (source == null) return null;
            return new PrimitiveExecutionV4ObservationRef {
                schema_version = source.schema_version,
                runtime_instance_id = source.runtime_instance_id,
                episode_id = source.episode_id,
                reset_id = source.reset_id,
                state_id = source.state_id,
                depth_id = source.depth_id,
                sim_time_ns = source.sim_time_ns,
            };
        }
    }

    public sealed class PrimitiveExecutionResultLifecycle
    {
        private readonly ulong retryIntervalMs;
        private string runtimeInstanceId;
        private ulong executionId;
        private byte[] commandSequenceHash;
        private PrimitiveExecutionResultTransmission pendingResult;
        private PrimitiveExecutionResultTransmission acknowledgedResult;
        private ulong nextRetryAtMs;

        public PrimitiveExecutionResultLifecycle(ulong retryIntervalMs)
        {
            if (retryIntervalMs == 0UL)
                throw new ArgumentException("retry interval must be positive");
            this.retryIntervalMs = retryIntervalMs;
            State = PrimitiveExecutionLifecycleState.IDLE;
        }

        public PrimitiveExecutionLifecycleState State { get; private set; }
        public int PhysicalExecutionCount { get; private set; }
        public uint AppliedFrameCount { get; private set; }
        public int ResultGeneratedCount { get; private set; }
        public int SendAttemptCount { get; private set; }
        public bool HasPendingResult { get { return pendingResult != null; } }
        public PrimitiveExecutionResultTransmission PendingResult { get { return pendingResult; } }

        public void BeginExecution(string runtimeInstanceId, ulong executionId, byte[] sequenceHash)
        {
            if (State != PrimitiveExecutionLifecycleState.IDLE &&
                State != PrimitiveExecutionLifecycleState.ACKED)
                throw new PrimitiveExecutionLifecycleException(
                    "only one active or pending execution is supported");
            RequireIdentity(runtimeInstanceId, executionId, sequenceHash);
            acknowledgedResult = null;
            this.runtimeInstanceId = runtimeInstanceId;
            this.executionId = executionId;
            commandSequenceHash = (byte[])sequenceHash.Clone();
            PhysicalExecutionCount += 1;
            State = PrimitiveExecutionLifecycleState.EXECUTING;
        }

        public PrimitiveExecutionResultTransmission FinalizeExecution(
            PrimitiveExecutionV4Result result, ulong nowMs)
        {
            if (State != PrimitiveExecutionLifecycleState.EXECUTING)
                throw new PrimitiveExecutionLifecycleException(
                    "execution is not in EXECUTING state");
            byte[] serialized = ValidateBoundResult(result);
            return GenerateFinalResult(result, nowMs, serialized);
        }

        public PrimitiveExecutionResultTransmission RejectBeforeExecution(
            PrimitiveExecutionV4Result result, ulong nowMs)
        {
            if (State != PrimitiveExecutionLifecycleState.IDLE &&
                State != PrimitiveExecutionLifecycleState.ACKED)
                throw new PrimitiveExecutionLifecycleException(
                    "pre-execution rejection requires IDLE state");
            if (result == null || result.status != "REJECTED")
                throw new PrimitiveExecutionLifecycleException(
                    "pre-execution result must be REJECTED");
            RequireIdentity(result.runtime_instance_id, result.execution_id,
                result.command_sequence_hash);
            byte[] serialized = XMProtocolV4.CanonicalResultPayload(result);
            runtimeInstanceId = result.runtime_instance_id;
            executionId = result.execution_id;
            commandSequenceHash = (byte[])result.command_sequence_hash.Clone();
            acknowledgedResult = null;
            return GenerateFinalResult(result, nowMs, serialized);
        }

        public PrimitiveExecutionResultTransmission Poll(ulong nowMs)
        {
            if (pendingResult == null || nowMs < nextRetryAtMs) return null;
            SendAttemptCount += 1;
            nextRetryAtMs = AddInterval(nowMs, retryIntervalMs);
            return pendingResult;
        }

        public PrimitiveExecutionAckOutcome Acknowledge(PrimitiveExecutionV4Ack ack)
        {
            try
            {
                XMProtocolV4.ValidateAck(ack);
            }
            catch (ArgumentException)
            {
                return PrimitiveExecutionAckOutcome.REJECTED;
            }

            PrimitiveExecutionResultTransmission expected = pendingResult ?? acknowledgedResult;
            if (expected == null || !Matches(ack, expected))
                return PrimitiveExecutionAckOutcome.REJECTED;
            if (pendingResult == null)
                return PrimitiveExecutionAckOutcome.DUPLICATE;

            acknowledgedResult = pendingResult;
            pendingResult = null;
            State = PrimitiveExecutionLifecycleState.ACKED;
            return PrimitiveExecutionAckOutcome.ACCEPTED;
        }

        private PrimitiveExecutionResultTransmission GenerateFinalResult(
            PrimitiveExecutionV4Result result, ulong nowMs, byte[] serialized)
        {
            if (pendingResult != null || acknowledgedResult != null)
                throw new PrimitiveExecutionLifecycleException(
                    "final result is already immutable");
            byte[] resultHash = XMProtocolV4.Sha256Bytes(serialized);
            pendingResult = new PrimitiveExecutionResultTransmission(
                runtimeInstanceId, executionId, result.status, result.result_generation,
                result.requested_frame_count, result.applied_frame_count,
                result.first_applied_state_id, result.endpoint_state_id,
                result.last_applied_frame_index, result.reason_code,
                result.endpoint_sim_time_ns,
                result.endpoint_observation_ref,
                commandSequenceHash, resultHash, serialized);
            AppliedFrameCount = result.applied_frame_count;
            ResultGeneratedCount += 1;
            SendAttemptCount += 1;
            nextRetryAtMs = AddInterval(nowMs, retryIntervalMs);
            State = PrimitiveExecutionLifecycleState.FINAL_RESULT_PENDING_ACK;
            return pendingResult;
        }

        private byte[] ValidateBoundResult(PrimitiveExecutionV4Result result)
        {
            if (result == null || result.runtime_instance_id != runtimeInstanceId ||
                result.execution_id != executionId ||
                !SameBytes(result.command_sequence_hash, commandSequenceHash))
                throw new PrimitiveExecutionLifecycleException(
                    "final result identity does not match execution");
            if (result.status == "REJECTED")
                throw new PrimitiveExecutionLifecycleException(
                    "REJECTED is only valid before execution starts");
            // This is also the authoritative validation of COMPLETE accounting. It
            // must run before GenerateFinalResult, so an invalid COMPLETE cannot
            // create a pending payload.
            return XMProtocolV4.CanonicalResultPayload(result);
        }

        private static void RequireIdentity(string runtime, ulong id, byte[] hash)
        {
            if (string.IsNullOrEmpty(runtime) || hash == null || hash.Length != 32)
                throw new PrimitiveExecutionLifecycleException("execution identity is incomplete");
        }

        private static bool Matches(PrimitiveExecutionV4Ack ack,
            PrimitiveExecutionResultTransmission expected)
        {
            return ack.runtime_instance_id == expected.RuntimeInstanceId &&
                ack.execution_id == expected.ExecutionId &&
                SameBytes(ack.result_payload_hash, expected.ResultPayloadHash) &&
                SameBytes(ack.command_sequence_hash, expected.CommandSequenceHash);
        }

        private static bool SameBytes(byte[] left, byte[] right)
        {
            if (left == null || right == null || left.Length != right.Length) return false;
            for (int index = 0; index < left.Length; ++index)
                if (left[index] != right[index]) return false;
            return true;
        }

        private static ulong AddInterval(ulong nowMs, ulong intervalMs)
        {
            return ulong.MaxValue - nowMs < intervalMs
                ? ulong.MaxValue
                : nowMs + intervalMs;
        }
    }
}
