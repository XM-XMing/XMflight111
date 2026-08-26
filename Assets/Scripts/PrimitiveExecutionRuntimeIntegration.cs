using System;
using System.Collections.Generic;

namespace XMflight
{
    public interface IPrimitiveExecutionResultSink
    {
        void Record(PrimitiveExecutionResultTransmission result);
    }

    public interface IPrimitiveExecutionFrameTelemetrySink
    {
        void PublishFrameApplied(uint frameIndex, long stateId, ulong simTimeNs);
    }

    public sealed class PrimitiveExecutionRuntimeIntegration
    {
        private sealed class ResultRecord
        {
            public readonly PrimitiveExecutionResultLifecycle Lifecycle;
            public readonly ulong ExecutionId;
            public readonly PrimitiveExecutionResultTransmission Transmission;
            public PrimitiveExecutionResultTransportAdapter TransportAdapter;

            public ResultRecord(
                PrimitiveExecutionResultLifecycle lifecycle,
                ulong executionId,
                PrimitiveExecutionResultTransmission transmission)
            {
                Lifecycle = lifecycle;
                ExecutionId = executionId;
                Transmission = transmission;
            }
        }

        private readonly string runtimeInstanceId;
        private readonly IPrimitiveExecutionResultSink resultSink;
        private readonly ulong retryIntervalMs;
        private readonly List<PrimitiveExecutionV4Frame> frames =
            new List<PrimitiveExecutionV4Frame>();
        private readonly List<uint> appliedFrameIndices = new List<uint>();
        private readonly Dictionary<ulong, ResultRecord> pendingResults =
            new Dictionary<ulong, ResultRecord>();

        private bool active;
        private IPrimitiveExecutionFrameTelemetrySink frameTelemetrySink;
        private PrimitiveExecutionResultLifecycle activeLifecycle;
        private ResultRecord acknowledgedResult;
        private IPrimitiveExecutionResultTransport resultTransport;
        private ulong executionId;
        private byte[] commandSequenceHash;
        private uint appliedFrameCount;
        private long firstAppliedStateId = -1;
        private long endpointStateId = -1;
        private ulong endpointSimTimeNs;
        private int physicalExecutionCount;
        private int resultGeneratedCount;

        // Diagnostic-only hooks.  They do not participate in lifecycle
        // decisions and are intentionally optional so the pure contract tests
        // remain independent of Unity logging.
        public Action<PrimitiveExecutionResultTransmission> OnTerminalResultGenerated;
        public Action<PrimitiveExecutionResultTransmission> OnTerminalResultFirstSend;
        public Action<PrimitiveExecutionResultTransmission, Exception> OnTerminalResultSendFailed;

        public PrimitiveExecutionRuntimeIntegration(
            string runtimeInstanceId,
            IPrimitiveExecutionResultSink resultSink,
            ulong retryIntervalMs)
        {
            if (string.IsNullOrEmpty(runtimeInstanceId))
                throw new ArgumentException("runtime instance id is required");
            if (resultSink == null) throw new ArgumentException("result sink is required");
            this.runtimeInstanceId = runtimeInstanceId;
            this.resultSink = resultSink;
            this.retryIntervalMs = retryIntervalMs;
        }

        public PrimitiveExecutionLifecycleState State {
            get {
                if (activeLifecycle != null) return activeLifecycle.State;
                if (pendingResults.Count > 0)
                    return PrimitiveExecutionLifecycleState.FINAL_RESULT_PENDING_ACK;
                if (acknowledgedResult != null)
                    return PrimitiveExecutionLifecycleState.ACKED;
                return PrimitiveExecutionLifecycleState.IDLE;
            }
        }
        public int PhysicalExecutionCount { get { return physicalExecutionCount; } }
        public int ResultGeneratedCount { get { return resultGeneratedCount; } }
        public uint AppliedFrameCount { get { return appliedFrameCount; } }
        public long FirstAppliedStateId { get { return firstAppliedStateId; } }
        public long EndpointStateId { get { return endpointStateId; } }
        public ulong EndpointSimTimeNs { get { return endpointSimTimeNs; } }
        public bool IsActive { get { return active; } }
        public int PendingResultCount { get { return pendingResults.Count; } }
        public int SendAttemptCount {
            get {
                int attempts = 0;
                if (activeLifecycle != null) attempts += activeLifecycle.SendAttemptCount;
                foreach (ResultRecord record in pendingResults.Values)
                    attempts += record.Lifecycle.SendAttemptCount;
                if (acknowledgedResult != null)
                    attempts += acknowledgedResult.Lifecycle.SendAttemptCount;
                return attempts;
            }
        }
        public IList<uint> AppliedFrameIndices {
            get { return new List<uint>(appliedFrameIndices); }
        }

        public void AttachResultTransport(IPrimitiveExecutionResultTransport transport)
        {
            if (transport == null) throw new ArgumentException("result transport is required");
            if (resultTransport != null)
                throw new PrimitiveExecutionLifecycleException(
                    "result transport is already attached");
            resultTransport = transport;
        }

        public PrimitiveExecutionAckOutcome ReceiveResultAck(PrimitiveExecutionV4Ack ack)
        {
            if (ack == null) return PrimitiveExecutionAckOutcome.REJECTED;

            ResultRecord record;
            if (pendingResults.TryGetValue(ack.execution_id, out record)) {
                PrimitiveExecutionAckOutcome outcome = record.TransportAdapter == null
                    ? record.Lifecycle.Acknowledge(ack)
                    : record.TransportAdapter.ReceiveAck(ack);
                if (outcome == PrimitiveExecutionAckOutcome.ACCEPTED) {
                    pendingResults.Remove(ack.execution_id);
                    acknowledgedResult = record;
                }
                return outcome;
            }

            if (acknowledgedResult != null)
                return acknowledgedResult.TransportAdapter == null
                    ? acknowledgedResult.Lifecycle.Acknowledge(ack)
                    : acknowledgedResult.TransportAdapter.ReceiveAck(ack);
            return PrimitiveExecutionAckOutcome.REJECTED;
        }

        public bool PollResultTransport(ulong nowMs)
        {
            bool sent = false;
            foreach (ResultRecord record in new List<ResultRecord>(pendingResults.Values)) {
                if (record.TransportAdapter != null && record.TransportAdapter.Poll(nowMs))
                    sent = true;
            }
            return sent;
        }

        public void ResultTransportDisconnected()
        {
            foreach (ResultRecord record in new List<ResultRecord>(pendingResults.Values))
                if (record.TransportAdapter != null) record.TransportAdapter.OnDisconnected();
        }

        public bool ResultTransportReconnected()
        {
            bool resent = false;
            foreach (ResultRecord record in new List<ResultRecord>(pendingResults.Values)) {
                if (record.TransportAdapter != null && record.TransportAdapter.OnReconnected())
                    resent = true;
            }
            return resent;
        }

        private void RecordTerminalResult(
            PrimitiveExecutionResultLifecycle lifecycle,
            PrimitiveExecutionResultTransmission transmission)
        {
            if (transmission == null)
                throw new PrimitiveExecutionLifecycleException("terminal result is required");
            if (lifecycle == null)
                throw new PrimitiveExecutionLifecycleException("terminal lifecycle is required");
            if (pendingResults.ContainsKey(transmission.ExecutionId))
                throw new PrimitiveExecutionLifecycleException(
                    "terminal result identity already has a pending payload");

            var record = new ResultRecord(
                lifecycle, transmission.ExecutionId, transmission);
            pendingResults.Add(transmission.ExecutionId, record);
            resultSink.Record(transmission);
            resultGeneratedCount += 1;
            if (OnTerminalResultGenerated != null)
                OnTerminalResultGenerated(transmission);
            if (resultTransport != null) {
                record.TransportAdapter = new PrimitiveExecutionResultTransportAdapter(
                    record.Lifecycle, resultTransport);
                try {
                    if (OnTerminalResultFirstSend != null)
                        OnTerminalResultFirstSend(transmission);
                    record.TransportAdapter.Record(transmission);
                }
                catch (Exception e) {
                    if (OnTerminalResultSendFailed != null)
                        OnTerminalResultSendFailed(transmission, e);
                }
            }
        }

        public void SetFrameTelemetrySink(IPrimitiveExecutionFrameTelemetrySink sink)
        {
            frameTelemetrySink = sink;
        }

        public void BeginExecution(
            ulong executionId,
            IList<PrimitiveExecutionV4Frame> submittedFrames,
            ulong nowMs)
        {
            if (submittedFrames == null || submittedFrames.Count != 25)
                throw new PrimitiveExecutionLifecycleException(
                    "v4 runtime integration requires exactly 25 frames");
            if (active)
                throw new PrimitiveExecutionLifecycleException(
                    "runtime integration already has an active execution");

            byte[] commandBytes = XMProtocolV4.CanonicalCommandSequence(submittedFrames);
            byte[] commandHash = XMProtocolV4.Sha256Bytes(commandBytes);
            if (pendingResults.ContainsKey(executionId) ||
                (acknowledgedResult != null && acknowledgedResult.ExecutionId == executionId))
                throw new PrimitiveExecutionLifecycleException(
                    "execution identity already has a terminal result");

            activeLifecycle = new PrimitiveExecutionResultLifecycle(retryIntervalMs);
            activeLifecycle.BeginExecution(runtimeInstanceId, executionId, commandHash);
            physicalExecutionCount += 1;

            this.executionId = executionId;
            commandSequenceHash = (byte[])commandHash.Clone();
            frames.Clear();
            frames.AddRange(submittedFrames);
            appliedFrameIndices.Clear();
            appliedFrameCount = 0U;
            firstAppliedStateId = -1;
            endpointStateId = -1;
            endpointSimTimeNs = 0UL;
            active = true;
        }

        public bool RecordAppliedFrame(
            uint frameIndex,
            long stateId,
            ulong simTimeNs,
            ulong nowMs)
        {
            if (!active) return false;
            uint expectedIndex = (uint)appliedFrameIndices.Count;
            if (frameIndex != expectedIndex || frameIndex >= frames.Count ||
                frames[(int)frameIndex].frame_index != frameIndex)
                throw new PrimitiveExecutionLifecycleException(
                    "applied frame index is not the next submitted frame");

            if (appliedFrameIndices.Count == 0) firstAppliedStateId = stateId;
            appliedFrameIndices.Add(frameIndex);
            appliedFrameCount = (uint)appliedFrameIndices.Count;
            if (frameTelemetrySink != null)
                frameTelemetrySink.PublishFrameApplied(frameIndex, stateId, simTimeNs);

            if (frameIndex != 24U) return true;

            endpointStateId = stateId;
            endpointSimTimeNs = simTimeNs;
            // COMPLETE is generated only after the explicitly requested depth
            // capture returns.  The physics endpoint is recorded here, but a
            // render timestamp is never inferred from this state timestamp.
            return true;
        }

        public bool RecordEndpointObservation(
            string captureId,
            ulong captureTimeNs,
            string episodeId,
            string resetId,
            string observationRuntimeInstanceId,
            ulong nowMs)
        {
            if (!active || appliedFrameCount != 25U || endpointStateId < 0)
                return false;
            if (string.IsNullOrEmpty(captureId) || captureTimeNs == 0UL ||
                string.IsNullOrEmpty(episodeId) || string.IsNullOrEmpty(resetId) ||
                observationRuntimeInstanceId != runtimeInstanceId)
                throw new PrimitiveExecutionLifecycleException(
                    "endpoint observation identity is incomplete or mismatched");

            PrimitiveExecutionV4Result result = new PrimitiveExecutionV4Result {
                schema_version = 4,
                runtime_instance_id = runtimeInstanceId,
                execution_id = executionId,
                status = "COMPLETE",
                requested_frame_count = 25,
                applied_frame_count = 25,
                first_applied_state_id = firstAppliedStateId,
                endpoint_state_id = endpointStateId,
                last_applied_frame_index = 24,
                reason_code = "NONE",
                command_sequence_hash = (byte[])commandSequenceHash.Clone(),
                endpoint_sim_time_ns = endpointSimTimeNs,
                endpoint_observation_ref = new PrimitiveExecutionV4ObservationRef {
                    schema_version = 4,
                    runtime_instance_id = observationRuntimeInstanceId,
                    episode_id = episodeId,
                    reset_id = resetId,
                    state_id = endpointStateId,
                    depth_id = captureId,
                    sim_time_ns = endpointSimTimeNs,
                },
                result_generation = 0,
            };
            PrimitiveExecutionResultTransmission transmission = activeLifecycle.FinalizeExecution(
                result, nowMs);
            RecordTerminalResult(activeLifecycle, transmission);
            activeLifecycle = null;
            active = false;
            return true;
        }

        public bool RecordTerminalFailureObservation(
            string reason,
            long terminalStateId,
            ulong terminalSimTimeNs,
            string captureId,
            ulong captureTimeNs,
            string episodeId,
            string resetId,
            string observationRuntimeInstanceId,
            ulong nowMs)
        {
            if (!active || appliedFrameCount >= 25U || terminalStateId < 0)
                return false;
            if (string.IsNullOrEmpty(reason) || reason != "COLLISION")
                throw new PrimitiveExecutionLifecycleException(
                    "unsupported terminal environment failure reason");
            if (string.IsNullOrEmpty(captureId) || captureTimeNs == 0UL ||
                string.IsNullOrEmpty(episodeId) || string.IsNullOrEmpty(resetId) ||
                observationRuntimeInstanceId != runtimeInstanceId)
                throw new PrimitiveExecutionLifecycleException(
                    "terminal observation identity is incomplete or mismatched");

            endpointStateId = terminalStateId;
            endpointSimTimeNs = terminalSimTimeNs;
            PrimitiveExecutionV4Result result = new PrimitiveExecutionV4Result {
                schema_version = 4,
                runtime_instance_id = runtimeInstanceId,
                execution_id = executionId,
                status = "FAILED",
                requested_frame_count = 25,
                applied_frame_count = appliedFrameCount,
                first_applied_state_id = appliedFrameCount == 0U
                    ? (long?)null
                    : firstAppliedStateId,
                endpoint_state_id = terminalStateId,
                last_applied_frame_index = (int)appliedFrameCount - 1,
                reason_code = reason,
                command_sequence_hash = (byte[])commandSequenceHash.Clone(),
                endpoint_sim_time_ns = terminalSimTimeNs,
                endpoint_observation_ref = new PrimitiveExecutionV4ObservationRef {
                    schema_version = 4,
                    runtime_instance_id = observationRuntimeInstanceId,
                    episode_id = episodeId,
                    reset_id = resetId,
                    state_id = terminalStateId,
                    depth_id = captureId,
                    sim_time_ns = terminalSimTimeNs,
                },
                result_generation = 0,
            };
            PrimitiveExecutionResultTransmission transmission = activeLifecycle.FinalizeExecution(
                result, nowMs);
            RecordTerminalResult(activeLifecycle, transmission);
            activeLifecycle = null;
            active = false;
            return true;
        }

        public bool RejectOverlappingExecution(
            ulong rejectedExecutionId,
            IList<PrimitiveExecutionV4Frame> rejectedFrames,
            ulong nowMs)
        {
            if (!active) return false;
            if (rejectedFrames == null || rejectedFrames.Count != 25)
                throw new PrimitiveExecutionLifecycleException(
                    "overlapping v4 execution must contain exactly 25 frames");

            byte[] commandBytes = XMProtocolV4.CanonicalCommandSequence(rejectedFrames);
            byte[] commandHash = XMProtocolV4.Sha256Bytes(commandBytes);
            return RejectBeforeExecution(
                rejectedExecutionId, commandHash, "OVERLAPPING_EXECUTION", nowMs);
        }

        // This is the v4 admission boundary.  Endpoint capture remains part of
        // the active execution after physics frame 24, even when the manager has
        // already released its BufferedPrimitiveExecution object.
        public bool RejectExecutionIfBusy(
            ulong rejectedExecutionId,
            IList<PrimitiveExecutionV4Frame> rejectedFrames,
            ulong nowMs)
        {
            if (!active) return false;
            return RejectOverlappingExecution(rejectedExecutionId, rejectedFrames, nowMs);
        }

        public bool RejectBeforeExecution(
            ulong rejectedExecutionId,
            byte[] commandHash,
            string reason,
            ulong nowMs)
        {
            if (commandHash == null || commandHash.Length != 32)
                throw new PrimitiveExecutionLifecycleException(
                    "rejected execution command hash must be bytes32");
            if (string.IsNullOrEmpty(reason))
                throw new PrimitiveExecutionLifecycleException(
                    "rejection reason is required");

            ResultRecord existing;
            if (pendingResults.TryGetValue(rejectedExecutionId, out existing)) {
                if (existing.Transmission == null ||
                    existing.Transmission.Status != "REJECTED" ||
                    !SameBytes(existing.Transmission.CommandSequenceHash, commandHash))
                    throw new PrimitiveExecutionLifecycleException(
                        "conflicting terminal result identity");
                if (existing.TransportAdapter != null)
                    existing.TransportAdapter.ResendPending();
                return false;
            }
            if (acknowledgedResult != null &&
                acknowledgedResult.ExecutionId == rejectedExecutionId) {
                if (acknowledgedResult.Transmission == null ||
                    acknowledgedResult.Transmission.Status != "REJECTED" ||
                    !SameBytes(acknowledgedResult.Transmission.CommandSequenceHash, commandHash))
                    throw new PrimitiveExecutionLifecycleException(
                        "conflicting terminal result identity");
                return false;
            }

            var rejectionLifecycle = new PrimitiveExecutionResultLifecycle(retryIntervalMs);
            PrimitiveExecutionV4Result result = new PrimitiveExecutionV4Result {
                schema_version = 4,
                runtime_instance_id = runtimeInstanceId,
                execution_id = rejectedExecutionId,
                status = "REJECTED",
                requested_frame_count = 25,
                applied_frame_count = 0,
                last_applied_frame_index = -1,
                reason_code = reason,
                command_sequence_hash = commandHash,
                result_generation = 0,
            };
            PrimitiveExecutionResultTransmission transmission = rejectionLifecycle
                .RejectBeforeExecution(result, nowMs);
            RecordTerminalResult(rejectionLifecycle, transmission);
            return true;
        }

        private static bool SameBytes(byte[] left, byte[] right)
        {
            if (left == null || right == null || left.Length != right.Length) return false;
            for (int index = 0; index < left.Length; ++index)
                if (left[index] != right[index]) return false;
            return true;
        }

        public bool CancelActiveExecution(string reason, ulong nowMs)
        {
            if (!active) return false;
            if (string.IsNullOrEmpty(reason))
                throw new PrimitiveExecutionLifecycleException(
                    "cancellation reason is required");

            PrimitiveExecutionV4Result result = new PrimitiveExecutionV4Result {
                schema_version = 4,
                runtime_instance_id = runtimeInstanceId,
                execution_id = executionId,
                status = "CANCELLED",
                requested_frame_count = 25,
                applied_frame_count = appliedFrameCount,
                first_applied_state_id = appliedFrameCount == 0U
                    ? (long?)null
                    : firstAppliedStateId,
                endpoint_state_id = null,
                last_applied_frame_index = (int)appliedFrameCount - 1,
                reason_code = reason,
                command_sequence_hash = (byte[])commandSequenceHash.Clone(),
                endpoint_sim_time_ns = null,
                endpoint_observation_ref = null,
                result_generation = 0,
            };
            PrimitiveExecutionResultTransmission transmission = activeLifecycle.FinalizeExecution(
                result, nowMs);
            RecordTerminalResult(activeLifecycle, transmission);
            activeLifecycle = null;
            active = false;
            return true;
        }

        public bool FailActiveExecution(string reason, ulong nowMs)
        {
            if (!active) return false;
            if (string.IsNullOrEmpty(reason))
                throw new PrimitiveExecutionLifecycleException(
                    "failure reason is required");

            PrimitiveExecutionV4Result result = new PrimitiveExecutionV4Result {
                schema_version = 4,
                runtime_instance_id = runtimeInstanceId,
                execution_id = executionId,
                status = "FAILED",
                requested_frame_count = 25,
                applied_frame_count = appliedFrameCount,
                first_applied_state_id = appliedFrameCount == 0U
                    ? (long?)null
                    : firstAppliedStateId,
                endpoint_state_id = null,
                last_applied_frame_index = (int)appliedFrameCount - 1,
                reason_code = reason,
                command_sequence_hash = (byte[])commandSequenceHash.Clone(),
                endpoint_sim_time_ns = null,
                endpoint_observation_ref = null,
                result_generation = 0,
            };
            PrimitiveExecutionResultTransmission transmission = activeLifecycle.FinalizeExecution(
                result, nowMs);
            RecordTerminalResult(activeLifecycle, transmission);
            activeLifecycle = null;
            active = false;
            return true;
        }
    }
}
