// filename: Assets/Scripts/XMConfig.cs
using UnityEngine;

namespace XMflight
{
    [CreateAssetMenu(fileName = "XMFlightConfig", menuName = "XMflight/Config")]
    public sealed class XMConfig : ScriptableObject {
        [Header("ZMQ")]
        public int commandSubPort = 10253;
        public int statePubPort = 10254;
        public int depthPubPort = 11254;
        public int recvHighWatermark = 2;
        public int sendHighWatermark = 2;

        [Header("Simulation")]
        [Range(1f, 10f)] public float simulationSpeed = 1f;
        public float fixedDeltaTime = 0.02f;

        [Header("RL Training")]
        public bool isRlTrainingMode = false;

        [Header("Dynamics")]
        public float timeConstantXY = 0.06f;
        public float timeConstantZ = 0.06f;
        public float obsNoiseStdDev = 0.00f;
        public float posNoiseStdDev = 0.00f;
        public int ctrlLatencyFrames = 0;

        public float maxSpeed = 5.0f;
        public float maxAngularRateDeg = 120.0f;
        public float accFilterTime = 0.15f;
        public float maxTiltAngleDeg = 15.0f;
        public float attitudeFilterTime = 0.30f;
        public float maxTiltRateDeg = 70.0f;
        [Range(0f, 1f)] public float visualTiltScale = 0.65f;

        [Header("External Trajectory Tracking")]
        public float stepPosKp = 3.0f;
        public float stepPosErrClamp = 1.0f;
        public bool zeroCommandOnTimeout = true;
        public float commandTimeoutSec = 0.25f;

        [Header("Flight Envelope")]
        public bool enableAltitudeViolationFlag = true;
        public float minFlightHeight = 1.0f;
        public float maxFlightHeight = 3.0f;
        public float altitudeViolationMargin = 0.0f;

        [Header("Collision Geometry")]
        public bool stopOnCollision = true;
        public float droneRadius = 0.2f;
        public float collisionRayMaxRange = 5.0f;
        public LayerMask obstacleMask = 1 << 3;

        [Header("Collision Ray Sampling")]
        public int maxRayCount = 1536;
        public int mediumRayCount = 768;
        public int lowRayCount = 384;
        [Range(0f, 1f)] public float frontRayRatio = 0.65f;
        public float frontConeHalfAngleDeg = 40.0f;

        [Header("Collision Risk Thresholds")]
        public float highRiskClearance = 0.8f;
        public float mediumRiskClearance = 2.0f;
        public float highRiskSpeed = 2.2f;
        public float mediumRiskSpeed = 1.2f;

        [Header("Collision Detection Intervals")]
        public int highRiskInterval = 1;
        public int mediumRiskInterval = 2;
        public int lowRiskInterval = 5;

        [Header("D435i Depth")]
        public float targetHFovDeg = 87.0f;
        public float targetVFovDeg = 58.0f;
        public float minDepthRange = 0.3f;
        public float maxDepthRange = 3.0f;
        public int outputWidth = 848;
        public int outputHeight = 480;

        public Vector3 cameraOffset = Vector3.zero;
        public Vector3 cameraEuler = Vector3.zero;

        [Header("Depth Visualization")]
        public bool showDepthInGameView = true;

        [Header("Depth Noise")]
        public bool enableDepthNoise = false;
        public float baseDepthNoiseStd = 0.002f;
        public float depthNoiseQuadraticK = 0.0015f;
        [Range(0f, 1f)] public float randomDropoutProb = 0f;
    }
}