using System;

namespace XMflight
{
    public enum EndpointObservationCaptureAcceptOutcome
    {
        ACCEPTED,
        REJECTED,
        STALE,
    }

    /// <summary>
    /// Owns one exact endpoint capture callback.  The owner is consumed by
    /// the first exact callback, so a late or duplicate GPU callback cannot
    /// finalize another execution.
    /// </summary>
    public sealed class EndpointObservationCaptureOwnership
    {
        private EndpointObservationCaptureIdentity owner;

        public bool HasOwner { get { return owner != null; } }

        public void Begin(EndpointObservationCaptureIdentity identity)
        {
            Validate(identity);
            if (owner != null)
                throw new InvalidOperationException("endpoint capture ownership is already active");
            owner = Clone(identity);
        }

        public void Invalidate()
        {
            owner = null;
        }

        public EndpointObservationCaptureAcceptOutcome TryAccept(
            EndpointObservationCaptureIdentity callback)
        {
            if (owner == null) return EndpointObservationCaptureAcceptOutcome.STALE;
            if (!IsValid(callback))
                return EndpointObservationCaptureAcceptOutcome.REJECTED;
            if (!Matches(owner, callback))
                return EndpointObservationCaptureAcceptOutcome.REJECTED;
            owner = null;
            return EndpointObservationCaptureAcceptOutcome.ACCEPTED;
        }

        private static void Validate(EndpointObservationCaptureIdentity identity)
        {
            if (!IsValid(identity))
                throw new ArgumentException("endpoint capture identity is incomplete");
        }

        private static bool IsValid(EndpointObservationCaptureIdentity identity)
        {
            return identity != null &&
                !string.IsNullOrEmpty(identity.runtime_instance_id) &&
                identity.execution_id >= 0 &&
                !string.IsNullOrEmpty(identity.episode_id) &&
                !string.IsNullOrEmpty(identity.reset_id) &&
                identity.endpoint_state_id >= 0 &&
                (identity.source_frame_index == 24 ||
                    identity.source_frame_index == -1) &&
                identity.capture_id >= 0;
        }

        private static EndpointObservationCaptureIdentity Clone(
            EndpointObservationCaptureIdentity source)
        {
            return new EndpointObservationCaptureIdentity {
                runtime_instance_id = source.runtime_instance_id,
                execution_id = source.execution_id,
                episode_id = source.episode_id,
                reset_id = source.reset_id,
                endpoint_state_id = source.endpoint_state_id,
                source_frame_index = source.source_frame_index,
                capture_id = source.capture_id,
            };
        }

        private static bool Matches(
            EndpointObservationCaptureIdentity left,
            EndpointObservationCaptureIdentity right)
        {
            return left.runtime_instance_id == right.runtime_instance_id &&
                left.execution_id == right.execution_id &&
                left.episode_id == right.episode_id &&
                left.reset_id == right.reset_id &&
                left.endpoint_state_id == right.endpoint_state_id &&
                left.source_frame_index == right.source_frame_index &&
                left.capture_id == right.capture_id;
        }
    }

    public sealed class EndpointObservationCaptureIdentity
    {
        public string runtime_instance_id;
        public long execution_id;
        public string episode_id;
        public string reset_id;
        public long endpoint_state_id;
        public int source_frame_index;
        public long capture_id;
    }
}
