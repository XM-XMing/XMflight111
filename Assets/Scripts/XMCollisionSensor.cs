// filename: Assets/Scripts/XMCollisionSensor.cs
using UnityEngine;
using Unity.Jobs;
using Unity.Collections;

namespace XMflight
{
    public sealed class XMCollisionSensor : MonoBehaviour {
        [Header("Collision Geometry")]
        [SerializeField] private float _collisionRadius = 0.2f;
        [SerializeField] private float _maxRange = 5.0f;
        [SerializeField] private LayerMask _obstacleMask = 1 << 3;

        [Header("Adaptive Ray Sampling")]
        [SerializeField] private int _maxRayCount = 1536;
        [SerializeField] private int _mediumRayCount = 768;
        [SerializeField] private int _lowRayCount = 384;
        [SerializeField, Range(0f, 1f)] private float _frontRayRatio = 0.65f;
        [SerializeField] private float _frontConeHalfAngleDeg = 40.0f;

        [Header("Risk Thresholds")]
        [SerializeField] private float _highRiskClearance = 0.8f;
        [SerializeField] private float _mediumRiskClearance = 2.0f;
        [SerializeField] private float _highRiskSpeed = 2.2f;
        [SerializeField] private float _mediumRiskSpeed = 1.2f;

        [Header("Detection Intervals in FixedUpdate frames")]
        [SerializeField] private int _highRiskInterval = 1;
        [SerializeField] private int _mediumRiskInterval = 2;
        [SerializeField] private int _lowRiskInterval = 5;

        public float MinClearance { get; private set; }
        public float[] FrontClearances { get; private set; } = new float[3];
        public bool HasCollided { get; private set; }
        public int ActiveRayCount { get; private set; }
        public int ActiveInterval { get; private set; }

        private NativeArray<Vector3> _rayDirs;
        private NativeArray<RaycastHit> _results;
        private NativeArray<RaycastCommand> _commands;
        private readonly Collider[] _overlapBuf = new Collider[XMConstants.OverlapBufferSize];

        private int _frameCounter;
        private float _speedHint;
        private bool _initialized;

        public void ApplyConfig(XMConfig cfg) {
            if (cfg == null) return;

            _collisionRadius = cfg.droneRadius;
            _maxRange = cfg.collisionRayMaxRange;
            _obstacleMask = cfg.obstacleMask;

            _maxRayCount = cfg.maxRayCount;
            _mediumRayCount = cfg.mediumRayCount;
            _lowRayCount = cfg.lowRayCount;
            _frontRayRatio = cfg.frontRayRatio;
            _frontConeHalfAngleDeg = cfg.frontConeHalfAngleDeg;

            _highRiskClearance = cfg.highRiskClearance;
            _mediumRiskClearance = cfg.mediumRiskClearance;
            _highRiskSpeed = cfg.highRiskSpeed;
            _mediumRiskSpeed = cfg.mediumRiskSpeed;

            _highRiskInterval = cfg.highRiskInterval;
            _mediumRiskInterval = cfg.mediumRiskInterval;
            _lowRiskInterval = cfg.lowRiskInterval;

            if (_initialized) {
                RebuildRayBuffers();
            }
        }
        
        public void SetSpeedHint(float speedMetersPerSecond) {
            _speedHint = Mathf.Max(0f, speedMetersPerSecond);
        }

        private void Start() {
            RebuildRayBuffers();
        }

        private void RebuildRayBuffers() {
            DisposeBuffers();

            _maxRayCount = Mathf.Max(XMConstants.MinRayCount, _maxRayCount);
            _mediumRayCount = Mathf.Clamp(_mediumRayCount, XMConstants.MinRayCount, _maxRayCount);
            _lowRayCount = Mathf.Clamp(_lowRayCount, XMConstants.MinRayCount, _mediumRayCount);

            _rayDirs = new NativeArray<Vector3>(_maxRayCount, Allocator.Persistent);
            _results = new NativeArray<RaycastHit>(_maxRayCount, Allocator.Persistent);
            _commands = new NativeArray<RaycastCommand>(_maxRayCount, Allocator.Persistent);

            BuildBiasedSphere();
            
            for (int k = 0; k < 3; k++){
                FrontClearances[k] = _maxRange;
            }

            MinClearance = _maxRange;
            ActiveRayCount = _maxRayCount;
            ActiveInterval = _highRiskInterval;
            _initialized = true;
        }

        private void DisposeBuffers() {
            if (_rayDirs.IsCreated) _rayDirs.Dispose();
            if (_results.IsCreated) _results.Dispose();
            if (_commands.IsCreated) _commands.Dispose();
        }
        
        private void BuildBiasedSphere() {
            int frontCount = Mathf.RoundToInt(_maxRayCount * _frontRayRatio);
            int globalCount = _maxRayCount - frontCount;

            int idx = 0;

            for (int i = 0; i < globalCount; i++, idx++) {
                _rayDirs[idx] = FibonacciSphereDir(i, globalCount);
            }

            for (int i = 0; i < frontCount; i++, idx++) {
                _rayDirs[idx] = FibonacciConeDir(i, frontCount, _frontConeHalfAngleDeg);
            }
        }

        private static Vector3 FibonacciSphereDir(int i, int n) {
            float goldenRatio = (1f + Mathf.Sqrt(5f)) * 0.5f;
            float t = (i + 0.5f) / n;
            float phi = Mathf.Acos(1f - 2f * t);
            float theta = 2f * Mathf.PI * goldenRatio * i;

            return new Vector3( Mathf.Sin(phi) * Mathf.Cos(theta), Mathf.Cos(phi), Mathf.Sin(phi) * Mathf.Sin(theta) ).normalized;
        }

        private static Vector3 FibonacciConeDir(int i, int n, float halfAngleDeg) {
            float goldenRatio = (1f + Mathf.Sqrt(5f)) * 0.5f;
            float t = (i + 0.5f) / n;

            float cosMax = Mathf.Cos(halfAngleDeg * Mathf.Deg2Rad);
            float cosTheta = Mathf.Lerp(1f, cosMax, t);
            float sinTheta = Mathf.Sqrt(1f - cosTheta * cosTheta);
            float phi = 2f * Mathf.PI * goldenRatio * i;

            return new Vector3( sinTheta * Mathf.Cos(phi), sinTheta * Mathf.Sin(phi), cosTheta ).normalized;
        }

        private void FixedUpdate() {
            if (!_initialized) return;
            _frameCounter++;

            UpdateOverlapCollision();

            ActiveRayCount = SelectRayCount(_speedHint, MinClearance);
            ActiveInterval = SelectInterval(_speedHint, MinClearance);

            if (_frameCounter % ActiveInterval != 0)
                return;

            UpdateRayClearance(ActiveRayCount);
        }

        private void UpdateOverlapCollision() {
            int count = Physics.OverlapSphereNonAlloc(
                transform.position,
                _collisionRadius,
                _overlapBuf,
                _obstacleMask,
                QueryTriggerInteraction.Ignore
            );

            HasCollided = false;

            for (int i = 0; i < count; i++) {
                Collider c = _overlapBuf[i];
                if (c == null) continue;

                if (c.transform.root != transform.root) {
                    HasCollided = true;
                    break;
                }
            }
        }

        private void UpdateRayClearance(int rayCount) {
            Vector3 origin = transform.position;
            Quaternion rot = transform.rotation;
            QueryParameters queryParams = new QueryParameters(
                _obstacleMask,
                false,
                QueryTriggerInteraction.Ignore,
                false
            );

            for (int i = 0; i < rayCount; i++) {
                _commands[i] = new RaycastCommand(origin, rot * _rayDirs[i], queryParams, _maxRange);
            }

            NativeArray<RaycastCommand> cmdSub = _commands.GetSubArray(0, rayCount);
            NativeArray<RaycastHit> resSub = _results.GetSubArray(0, rayCount);

            JobHandle handle = RaycastCommand.ScheduleBatch(cmdSub, resSub, XMConstants.RaycastBatchSize);
            handle.Complete();

            float minDist = _maxRange;

            for (int k = 0; k < 3; k++)
                FrontClearances[k] = _maxRange;

            for (int i = 0; i < rayCount; i++) {
                RaycastHit hit = _results[i];
                if (hit.collider == null) continue;

                if (hit.collider.transform.root == transform.root)
                    continue;

                float dist = Mathf.Max(0f, hit.distance - _collisionRadius);

                if (dist < minDist)
                    minDist = dist;

                Vector3 localDir = _rayDirs[i];

                if (localDir.z < 0.1f)
                    continue;

                if (localDir.x < -0.2f)
                    FrontClearances[0] = Mathf.Min(FrontClearances[0], dist);
                else if (localDir.x > 0.2f)
                    FrontClearances[2] = Mathf.Min(FrontClearances[2], dist);
                else
                    FrontClearances[1] = Mathf.Min(FrontClearances[1], dist);
            }

            MinClearance = Mathf.Max(0f, minDist);
        }

        private int SelectRayCount(float speed, float clearance) {
            if (clearance <= _highRiskClearance || speed >= _highRiskSpeed)
                return _maxRayCount;

            if (clearance <= _mediumRiskClearance || speed >= _mediumRiskSpeed)
                return _mediumRayCount;

            return _lowRayCount;
        }

        private int SelectInterval(float speed, float clearance) {
            if (clearance <= _highRiskClearance || speed >= _highRiskSpeed)
                return Mathf.Max(1, _highRiskInterval);

            if (clearance <= _mediumRiskClearance || speed >= _mediumRiskSpeed)
                return Mathf.Max(1, _mediumRiskInterval);

            return Mathf.Max(1, _lowRiskInterval);
        }

        private void OnDestroy() {
            DisposeBuffers();
        }
    }
}
