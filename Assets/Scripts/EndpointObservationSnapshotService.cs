using System;

namespace XMflight
{
    /// <summary>
    /// Pure authoritative service over <see cref="EndpointObservationCache"/>.
    /// It accepts only an exact observation reference and never performs a
    /// latest, nearest-time, or telemetry lookup.
    /// </summary>
    public interface IEndpointObservationSnapshotSink
    {
        void SendSnapshot(SnapshotRequestV4 request, EndpointObservationSnapshotV4 snapshot);
        void SendMissing(SnapshotRequestV4 request);
    }

    public sealed class EndpointObservationSnapshotService
    {
        private readonly EndpointObservationCache cache;
        private readonly IEndpointObservationSnapshotSink sink;
        private readonly string runtimeInstanceId;

        public EndpointObservationSnapshotService(
            EndpointObservationCache cache,
            IEndpointObservationSnapshotSink sink,
            string runtimeInstanceId)
        {
            if (cache == null) throw new ArgumentException("endpoint cache is required");
            if (sink == null) throw new ArgumentException("snapshot sink is required");
            if (string.IsNullOrEmpty(runtimeInstanceId))
                throw new ArgumentException("runtime_instance_id is required");
            this.cache = cache;
            this.sink = sink;
            this.runtimeInstanceId = runtimeInstanceId;
        }

        public void HandleRequest(SnapshotRequestV4 request)
        {
            ValidateRequest(request);
            EndpointObservationSnapshotV4 snapshot;
            if (!cache.TryGet(request.observation_ref, out snapshot)) {
                sink.SendMissing(request);
                return;
            }
            sink.SendSnapshot(request, snapshot);
        }

        public void HandleAck(SnapshotAckV4 ack)
        {
            if (ack == null || ack.observation_ref == null ||
                ack.observation_ref.runtime_instance_id != runtimeInstanceId)
                throw new ArgumentException("snapshot ACK runtime identity mismatch");
            cache.Release(ack.observation_ref, ack.snapshot_hash);
        }

        private void ValidateRequest(SnapshotRequestV4 request)
        {
            if (request == null || request.observation_ref == null ||
                request.observation_ref.runtime_instance_id != runtimeInstanceId)
                throw new ArgumentException("snapshot request runtime identity mismatch");
            XMProtocolV4.CanonicalSnapshotRequest(request);
        }
    }
}
