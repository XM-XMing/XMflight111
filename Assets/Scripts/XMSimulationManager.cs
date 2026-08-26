// filename: Assets/Scripts/XMSimulationManager.cs

using UnityEngine;
using NetMQ;
using NetMQ.Sockets;
using MessagePack;
using System;
using System.Collections;
using System.Collections.Generic;

namespace XMflight
{
    public sealed class XMSimulationManager : MonoBehaviour {
        [Header("Optional Unified Config")]
        [SerializeField] private XMConfig _config;

        [Header("ZMQ")]
        [SerializeField] private int _commandSubPort = 10253;
        [SerializeField] private int _statePubPort = 10254;
        [SerializeField] private int _depthPubPort = 11254;
        [SerializeField] private int _executionResultPort = 11255;
        [SerializeField] private int _observationSnapshotPort = 11256;
        [SerializeField] private int _reliableCommandPort = -1;
        [SerializeField] private int _recvHwm = 2;
        [SerializeField] private int _sendHwm = 2;
        [SerializeField] private bool _enableDiagnostics = false;

        [Header("Dynamics")]
        [SerializeField] private float _timeConstantXY = 0.06f;
        [SerializeField] private float _timeConstantZ = 0.06f;
        [SerializeField] private float _obsNoiseStdDev = 0.00f;
        [SerializeField] private float _posNoiseStdDev = 0.00f;
        [SerializeField] private int _ctrlLatencyFrames = 0;

        [Header("Physical Limits")]
        [SerializeField] private float _maxSpeed = 5.0f;
        [SerializeField] private float _maxAngularRate = 120.0f;
        [SerializeField] private float _accFilterTime = 0.15f;
        [SerializeField] private float _maxTiltAngle = 15.0f;
        [SerializeField] private float _attitudeFilterTime = 0.30f;
        [SerializeField] private float _maxTiltRate = 70.0f;
        [SerializeField, Range(0f, 1f)] private float _visualTiltScale = 0.65f;

        [Header("External Trajectory Tracking")]
        [SerializeField] private float _stepPosKp = 3.0f;
        [SerializeField] private float _stepPosErrClamp = 1.0f;
        [SerializeField] private bool _zeroCommandOnTimeout = true;
        [SerializeField] private float _commandTimeoutSec = 0.25f;

        [Header("Flight Envelope")]
        [SerializeField] private bool _enableAltitudeViolationFlag = true;
        [SerializeField] private float _minFlightHeight = 1.0f;
        [SerializeField] private float _maxFlightHeight = 3.0f;
        [SerializeField] private float _altitudeViolationMargin = 0.0f;

        [Header("Simulation")]
        [SerializeField, Range(1f, 10f)] private float _simulationSpeed = 1.0f;
        [SerializeField] private float _fixedDeltaTime = 0.02f;

        [Header("Collision")]
        [SerializeField] private bool _stopOnCollision = true;

        [Header("P0-L2c v4 Result Integration")]
        [SerializeField] private bool _enableV4PrimitiveResultLifecycle = false;
        [SerializeField] private string _runtimeInstanceId = "unity-runtime-unknown";
        [SerializeField] private ulong _resultRetryIntervalMs = 100UL;

        [SerializeField] private int _publishStride = 1;
        private int _publishCounter = 0;

        private struct DelayedCmd {
            public Vector3 linearVelWorld;
            public float yawRate;
            public bool hasStepTarget;
            public Vector3 stepTargetPosWorld;
        }

        private sealed class BufferedPrimitiveExecution {
            public long executionId;
            public PrimitiveExecutionFrameMsg[] frames;
            public int nextFrameIndex;
        }

        private sealed class InMemoryPrimitiveExecutionResultSink :
            IPrimitiveExecutionResultSink {
            public PrimitiveExecutionResultTransmission LastResult;
            public int ResultCount;

            public void Record(PrimitiveExecutionResultTransmission result) {
                if (result == null)
                    throw new InvalidOperationException("v4 terminal result is required");
                LastResult = result;
                ResultCount++;
            }
        }

        private sealed class ZmqPrimitiveExecutionResultTransport :
            IPrimitiveExecutionResultTransport {
            private readonly DealerSocket socket;
            private string cachedIdentity;
            private byte[] cachedPayload;

            public ZmqPrimitiveExecutionResultTransport(int port) {
                socket = new DealerSocket();
                socket.Options.Linger = TimeSpan.Zero;
                socket.Connect($"tcp://127.0.0.1:{port}");
            }

            public bool IsConnected { get { return true; } }

            public void Send(PrimitiveExecutionResultTransmission result) {
                string identity = result.RuntimeInstanceId + ":" +
                    result.ExecutionId + ":" +
                    Convert.ToBase64String(result.ResultPayloadHash);
                if (cachedIdentity != identity) {
                    cachedIdentity = identity;
                    cachedPayload = PrimitiveExecutionResultWireCodec.SerializeResult(result);
                }
                byte[] payload = cachedPayload;
                if (!socket.TrySendFrame(payload))
                    throw new InvalidOperationException(
                        "execution result socket did not accept payload");
            }

            public bool TryReceive(out byte[] payload) {
                return socket.TryReceiveFrameBytes(out payload);
            }

            public void Close() {
                socket.Close();
                socket.Dispose();
            }
        }

        private sealed class ZmqPrimitiveExecutionCommandTransport {
            private readonly DealerSocket socket;

            public ZmqPrimitiveExecutionCommandTransport(
                int port, string runtimeInstanceId) {
                if (port <= 0)
                    throw new ArgumentException("reliable command port is required");
                socket = new DealerSocket();
                socket.Options.Linger = TimeSpan.Zero;
                socket.Connect($"tcp://127.0.0.1:{port}");
                if (!socket.TrySendFrame(
                        PrimitiveExecutionCommandWireCodec.SerializeReady(runtimeInstanceId)))
                    throw new InvalidOperationException(
                        "reliable command socket did not accept READY");
            }

            public bool TryReceive(out byte[] payload) {
                return socket.TryReceiveFrameBytes(out payload);
            }

            public bool SendReceipt(PrimitiveExecutionV4CommandReceipt receipt) {
                return socket.TrySendFrame(
                    PrimitiveExecutionCommandWireCodec.SerializeReceipt(receipt));
            }

            public bool SendResetComplete(PrimitiveResetV4Complete complete) {
                return socket.TrySendFrame(
                    PrimitiveResetV4WireCodec.SerializeComplete(complete));
            }

            public void Close() {
                socket.Close();
                socket.Dispose();
            }
        }

        private sealed class ZmqEndpointObservationSnapshotTransport :
            IEndpointObservationSnapshotSink {
            private readonly DealerSocket socket;

            public ZmqEndpointObservationSnapshotTransport(
                int port, string runtimeInstanceId) {
                socket = new DealerSocket();
                socket.Options.Linger = TimeSpan.Zero;
                socket.Connect($"tcp://127.0.0.1:{port}");
                if (!socket.TrySendFrame(
                        EndpointObservationSnapshotWireCodec.SerializeReady(runtimeInstanceId)))
                    throw new InvalidOperationException("snapshot socket did not accept READY");
            }

            public void SendSnapshot(
                SnapshotRequestV4 request, EndpointObservationSnapshotV4 snapshot) {
                if (!socket.TrySendFrame(
                        EndpointObservationSnapshotWireCodec.SerializeSnapshot(request, snapshot)))
                    throw new InvalidOperationException("snapshot socket did not accept response");
            }

            public void SendMissing(SnapshotRequestV4 request) {
                if (!socket.TrySendFrame(EndpointObservationSnapshotWireCodec.SerializeMissing(request)))
                    throw new InvalidOperationException("snapshot socket did not accept missing response");
            }

            public bool TryReceive(out byte[] payload) {
                return socket.TryReceiveFrameBytes(out payload);
            }

            public void Close() {
                socket.Close();
                socket.Dispose();
            }
        }

        private bool _hasPendingStepTarget;
        private Vector3 _pendingStepTargetWorld;

        private Vector3 _targetVelWorld;
        private float _targetYawRate;

        private Vector3 _realVelocity;
        private Vector3 _prevVelocity;
        private Vector3 _filteredAcc;

        private Vector3 _prevFixedPos;
        private float _maxFixedDelta;
        private float _avgFixedDelta;
        private int _fixedDeltaCount;

        private long _modeVelocityCount;
        private long _modeStepCount;
        private long _modeTrajectoryCount;
        private long _modeTeleportCount;
        private float _lastCommandRealtime;
        private bool _hasReceivedMotionCommand;
        private float _yawDeg;
        private float _smoothPitchDeg;
        private float _smoothRollDeg;
        private bool _frozen;
        private bool _deterministicExecutionMode;
        private bool _clearExecutionAfterPublish;
        private bool _endpointObservationCaptureQueued;
        private float _endpointObservationCaptureDeadlineRealtime = -1f;
        private const float EndpointObservationCaptureTimeoutSec = 1.0f;
        private EndpointObservationCaptureOwnership _endpointCaptureOwnership;
        private bool _pendingV4TerminalFailure;
        private long _pendingV4TerminalFailureExecutionId = -1L;
        private string _pendingV4TerminalFailureReason = "";
        private long _pendingV4TerminalFailureStateId = -1L;
        private BufferedPrimitiveExecution _activeExecution;
        private InMemoryPrimitiveExecutionResultSink _v4ResultSink;
        private PrimitiveExecutionRuntimeIntegration _v4RuntimeIntegration;
        private ZmqPrimitiveExecutionCommandTransport _v4CommandTransport;
        private PrimitiveExecutionCommandAdmission _v4CommandAdmission;
        private PrimitiveResetV4Request _pendingReset;
        private long _pendingResetStateId = -1L;
        private byte[] _pendingResetStateBytes;
        private bool _pendingResetCaptureQueued;
        private long _pendingResetCaptureId = -1L;
        private PrimitiveResetV4Complete _pendingResetComplete;
        private long _appliedExecutionId = -1;
        private int _appliedExecutionFrameIndex = -1;
        private long _appliedCommandId = -1;
        private int _executionStatus = XMProtocol.ExecutionStatusNone;

        private const string ExecutionTransportAuditContractId =
            "[DEBUG-EXEC-TRANSPORT-81660]unity_execution_transport_audit";
        // Diagnostic-only capacity for bounded long-run transport audits.
        // This does not change primitive execution or acknowledgement semantics.
        private const int ExecutionTransportAuditCapacity = 262144;
        private string _executionTransportAuditPath = "";
        private int _executionTransportAuditEpisodeId = -1;
        private string _executionTransportAuditRunId = "";
        private string _executionTransportAuditRuntimeIdentity = "";
        private string _endpointEpisodeId = "episode-0";
        private string _endpointResetId = "reset-0";
        private bool _executionTransportAuditWritten;
        private bool _executionTransportAuditOverflow;
        private System.Collections.Generic.List<ExecutionTransportAuditRecord>
            _executionTransportAudits;

        private struct ExecutionTransportAuditRecord {
            public long executionId;
            public int frameIndex;
            public int frameCount;
            public long commandId;
            public long stateId;
            public long simTimeNs;
            public int executionStatus;
            public bool frameApplied;
            public bool serializationAttempted;
            public bool serializationSuccess;
            public bool statePublishAttempted;
            public int trySendReturn;
            public int configuredSendHwm;
            public int configuredRecvHwm;
        }

        private readonly System.Collections.Generic.Queue<DelayedCmd> _cmdQueue = new System.Collections.Generic.Queue<DelayedCmd>(16);

        private DelayedCmd _lastExecutedCmd;

        private float _collisionLatchTimer;
        private const float LatchDuration = XMConstants.CollisionLatchDuration;

        private readonly NetMQMessage _depthMsg = new NetMQMessage();

        private Transform _drone;
        private XMCollisionSensor _col;
        private XMImageSynthesis _cam;

        private SubscriberSocket _sub;
        private PublisherSocket _statePub;
        private PublisherSocket _depthPub;
        private ZmqPrimitiveExecutionResultTransport _v4ResultTransport;
        private ZmqEndpointObservationSnapshotTransport _v4SnapshotTransport;
        private EndpointObservationCache _endpointObservationCache;
        private EndpointObservationSnapshotService _endpointObservationSnapshotService;
        private readonly Dictionary<long, byte[]> _endpointStateBytes =
            new Dictionary<long, byte[]>();

        private bool _running;

        private long _stateFramesSent;
        private long _depthFramesSent;
        // Diagnostic-only publication order. These counters are not serialized
        // and do not affect PUB/SUB, execution, or endpoint binding.
        private ulong _statePublishSequence;
        private ulong _depthPublishSequence;
        private long _stateId;
        private long _cmdReceived;
        private long _cmdDecodeErrors;
        private float _diagTimer;

        private readonly object[] _stateMsg = new object[XMProtocol.DynamicsStateFieldCount];
        private readonly float[] _statePosRos = new float[3];
        private readonly float[] _stateRotRos = new float[4];
        private readonly float[] _stateVelRos = new float[3];
        private readonly float[] _stateAccRos = new float[3];
        private readonly float[] _stateFrontClearances = new float[3];

        private readonly object[] _depthMetaMsg = new object[XMProtocol.DepthFrameFieldCount];
        private readonly float[] _depthPosRos = new float[3];
        private readonly float[] _depthRotRos = new float[4];
        private readonly float[] _depthVelRos = new float[3];
        private readonly float[] _depthAccRos = new float[3];
        private readonly float[] _depthForwardRos = new float[3];
        private readonly float[] _depthCameraFields = new float[XMProtocol.CameraFieldCount];
        private readonly int[] _depthMetaFields = new int[XMProtocol.DepthMetaFieldCount];
        private readonly float[] _depthFrontClearances = new float[3];

        private void Awake() {
            ApplyConfig();
            ParseCommandLineArgs();
            ApplyTimingSettings();
            QualitySettings.vSyncCount = 0;
        }

        private void ApplyTimingSettings() {
            _fixedDeltaTime = Mathf.Clamp(_fixedDeltaTime, 0.001f, 0.1f);
            Time.fixedDeltaTime = _fixedDeltaTime;
            Time.maximumDeltaTime = Mathf.Max(0.1f, 5f * _fixedDeltaTime);
            Application.targetFrameRate = Mathf.Max(1, Mathf.RoundToInt(1f / _fixedDeltaTime));
        }

        private void ApplyConfig() {
            if (_config == null) return;

            _commandSubPort = _config.commandSubPort;
            _statePubPort = _config.statePubPort;
            _depthPubPort = _config.depthPubPort;
            _recvHwm = _config.recvHighWatermark;
            _sendHwm = _config.sendHighWatermark;

            _fixedDeltaTime = _config.fixedDeltaTime;
            _simulationSpeed = _config.simulationSpeed;

            _timeConstantXY = _config.timeConstantXY;
            _timeConstantZ = _config.timeConstantZ;
            _obsNoiseStdDev = _config.obsNoiseStdDev;
            _posNoiseStdDev = _config.posNoiseStdDev;
            _ctrlLatencyFrames = _config.ctrlLatencyFrames;

            _maxSpeed = _config.maxSpeed;
            _maxAngularRate = _config.maxAngularRateDeg;
            _accFilterTime = _config.accFilterTime;
            _maxTiltAngle = _config.maxTiltAngleDeg;
            _attitudeFilterTime = _config.attitudeFilterTime;
            _maxTiltRate = _config.maxTiltRateDeg;
            _visualTiltScale = Mathf.Clamp01(_config.visualTiltScale);

            _stopOnCollision = _config.stopOnCollision;

            _stepPosKp = _config.stepPosKp;
            _stepPosErrClamp = _config.stepPosErrClamp;
            _zeroCommandOnTimeout = _config.zeroCommandOnTimeout;
            _commandTimeoutSec = _config.commandTimeoutSec;

            _enableAltitudeViolationFlag = _config.enableAltitudeViolationFlag;
            _minFlightHeight = _config.minFlightHeight;
            _maxFlightHeight = _config.maxFlightHeight;
            _altitudeViolationMargin = _config.altitudeViolationMargin;
        }

        private void Start() {
            Time.timeScale = _simulationSpeed;

            if (_enableV4PrimitiveResultLifecycle) {
                _v4ResultSink = new InMemoryPrimitiveExecutionResultSink();
                _v4RuntimeIntegration = new PrimitiveExecutionRuntimeIntegration(
                    _runtimeInstanceId, _v4ResultSink, _resultRetryIntervalMs);
                _v4RuntimeIntegration.OnTerminalResultGenerated =
                    OnV4TerminalResultGenerated;
                _v4RuntimeIntegration.OnTerminalResultFirstSend =
                    OnV4TerminalResultFirstSend;
                _v4RuntimeIntegration.OnTerminalResultSendFailed =
                    OnV4TerminalResultSendFailed;
                _endpointObservationCache = new EndpointObservationCache();
                _endpointCaptureOwnership = new EndpointObservationCaptureOwnership();
            }

            GameObject droneObj = GameObject.Find("Drone");
            if (droneObj == null) {
                Debug.LogError("[XMflight] Cannot find GameObject named Drone.");
                enabled = false;
                return;
            }

            _drone = droneObj.transform;
            _col = _drone.GetComponent<XMCollisionSensor>();
            _cam = _drone.GetComponentInChildren<XMImageSynthesis>();

            _yawDeg = _drone.eulerAngles.y;
            _prevFixedPos = _drone.position;

            if (_config != null) {
                if (_col != null) _col.ApplyConfig(_config);
                if (_cam != null) _cam.ApplyConfig(_config);
            }

            if (_cam != null) {
                _cam.OnRequestDynamics = () => (
                    _realVelocity,
                    _filteredAcc,
                    (_col != null && _col.HasCollided) || _collisionLatchTimer > 0f,
                    _col != null ? _col.MinClearance : 2f,
                    _col != null ? _col.FrontClearances : new float[3] { 2f, 2f, 2f }
                );

                _cam.OnCaptureLifecycleDiagnostic += OnCaptureLifecycleDiagnostic;
                _cam.OnFrameReady += OnFrameReady;
            }

            InitZMQ();
            _running = true;

            Debug.Log(
                $"[XMflight] Started. cmd_sub=tcp://*:{_commandSubPort}, " +
                $"state_pub=tcp://*:{_statePubPort}, depth_pub=tcp://*:{_depthPubPort}, " +
                $"execution_result=tcp://127.0.0.1:{_executionResultPort}, " +
                $"observation_snapshot=tcp://127.0.0.1:{_observationSnapshotPort}, " +
                $"timeScale={_simulationSpeed}"
            );
        }

        private void ParseCommandLineArgs() {
            string[] args = Environment.GetCommandLineArgs();

            for (int i = 0; i < args.Length; i++) {
                if (args[i] == "-cmdSubPort" && i + 1 < args.Length &&
                    int.TryParse(args[i + 1], out int cmdPort)) {
                    _commandSubPort = cmdPort;
                }

                if (args[i] == "-statePubPort" && i + 1 < args.Length &&
                    int.TryParse(args[i + 1], out int statePort)) {
                    _statePubPort = statePort;
                }

                if (args[i] == "-depthPubPort" && i + 1 < args.Length &&
                    int.TryParse(args[i + 1], out int depthPort)) {
                    _depthPubPort = depthPort;
                }

                if (args[i] == "-executionResultPort" && i + 1 < args.Length &&
                    int.TryParse(args[i + 1], out int resultPort)) {
                    _executionResultPort = resultPort;
                }

                if (args[i] == "-observationSnapshotPort" && i + 1 < args.Length &&
                    int.TryParse(args[i + 1], out int snapshotPort)) {
                    _observationSnapshotPort = snapshotPort;
                }

                if (args[i] == "-reliableCommandPort" && i + 1 < args.Length &&
                    int.TryParse(args[i + 1], out int reliableCommandPort)) {
                    _reliableCommandPort = reliableCommandPort;
                }

                if (args[i] == "-executionTransportAuditPath" && i + 1 < args.Length) {
                    _executionTransportAuditPath = args[i + 1];
                }

                if (args[i] == "-executionTransportAuditEpisodeId" && i + 1 < args.Length &&
                    int.TryParse(args[i + 1], out int auditEpisodeId)) {
                    _executionTransportAuditEpisodeId = auditEpisodeId;
                }

                if (args[i] == "-executionTransportAuditRunId" && i + 1 < args.Length) {
                    _executionTransportAuditRunId = args[i + 1];
                }

                if (args[i] == "-executionTransportAuditRuntimeIdentity" && i + 1 < args.Length) {
                    _executionTransportAuditRuntimeIdentity = args[i + 1];
                }

                if (args[i] == "-endpointEpisodeId" && i + 1 < args.Length) {
                    _endpointEpisodeId = args[i + 1];
                }

                if (args[i] == "-endpointResetId" && i + 1 < args.Length) {
                    _endpointResetId = args[i + 1];
                }

                if (args[i] == "-subPort" && i + 1 < args.Length &&
                    int.TryParse(args[i + 1], out int oldSub)) {
                    _commandSubPort = oldSub;
                }

                if (args[i] == "-pubPort" && i + 1 < args.Length &&
                    int.TryParse(args[i + 1], out int oldPub)) {
                    _statePubPort = oldPub;
                }

                if (args[i] == "-timeScale" && i + 1 < args.Length &&
                    float.TryParse(args[i + 1], out float scale)) {
                    _simulationSpeed = scale;
                }

                if ((args[i] == "-fixedDeltaTime" || args[i] == "-fixedDt") && i + 1 < args.Length &&
                    float.TryParse(args[i + 1], out float fixedDt)) {
                    _fixedDeltaTime = fixedDt;
                }

                if (args[i] == "-commandTimeoutSec" && i + 1 < args.Length &&
                    float.TryParse(args[i + 1], out float timeoutSec)) {
                    _commandTimeoutSec = timeoutSec;
                }

                if (args[i] == "-minFlightHeight" && i + 1 < args.Length &&
                    float.TryParse(args[i + 1], out float minHeight)) {
                    _minFlightHeight = minHeight;
                }

                if (args[i] == "-maxFlightHeight" && i + 1 < args.Length &&
                    float.TryParse(args[i + 1], out float maxHeight)) {
                    _maxFlightHeight = maxHeight;
                }

                if (args[i] == "-primitiveResultSchema" && i + 1 < args.Length &&
                    int.TryParse(args[i + 1], out int resultSchema)) {
                    _enableV4PrimitiveResultLifecycle = resultSchema == 4;
                }

                if (args[i] == "-runtimeInstanceId" && i + 1 < args.Length) {
                    _runtimeInstanceId = args[i + 1];
                }
            }
            if (!string.IsNullOrEmpty(_executionTransportAuditPath)) {
                _executionTransportAudits =
                    new System.Collections.Generic.List<ExecutionTransportAuditRecord>(
                        ExecutionTransportAuditCapacity
                    );
            }
        }

        private void Update() {
            if (!_running) return;

            PollV4ResultTransport();
            PollV4SnapshotTransport();
            if (_enableV4PrimitiveResultLifecycle &&
                _v4RuntimeIntegration != null &&
                _v4RuntimeIntegration.IsActive &&
                _endpointObservationCaptureQueued &&
                _endpointObservationCaptureDeadlineRealtime >= 0f &&
                Time.realtimeSinceStartup >= _endpointObservationCaptureDeadlineRealtime) {
                FailEndpointObservation(
                    "PHYSICS_COMPLETE_OBSERVATION_FAILED",
                    _activeExecution != null ? _activeExecution.executionId : -1L,
                    24,
                    "endpoint capture timeout");
            }
            PublishDiagnostics();
        }

        private void FixedUpdate() {
            if (!_running) return;

            ClearExecutionAcknowledgement();
            PollV4CommandTransport();
            PollCommands();

            if (_col != null)
                _col.SetSpeedHint(_realVelocity.magnitude);

            if (_col != null && _col.HasCollided) {
                _collisionLatchTimer = LatchDuration;

                if (_stopOnCollision) {
                    _frozen = true;
                    _realVelocity = Vector3.zero;
                    _prevVelocity = Vector3.zero;
                    _filteredAcc = Vector3.zero;
                    _targetVelWorld = Vector3.zero;
                    _targetYawRate = 0f;
                    _hasPendingStepTarget = false;
                }
            }
            else if (_collisionLatchTimer > 0f) {
                _collisionLatchTimer -= Time.fixedDeltaTime;
            }

            if (_activeExecution != null) {
                if (_enableV4PrimitiveResultLifecycle && _frozen)
                    FailPrimitiveExecution();
                else
                    ApplyNextPrimitiveExecutionFrame();
            }

            if (!_deterministicExecutionMode)
                ApplyCommandTimeoutBrake();

            bool integrationDue = !_deterministicExecutionMode || _activeExecution != null;
            if (!_frozen && integrationDue)
                StepDynamics();

            PublishDynamicsState();

            if (_clearExecutionAfterPublish) {
                _activeExecution = null;
                _clearExecutionAfterPublish = false;
            }
        }

        private void InitZMQ() {
            AsyncIO.ForceDotNet.Force();

            _sub = new SubscriberSocket();
            _sub.Options.ReceiveHighWatermark = Mathf.Max(1, _recvHwm);
            _sub.Options.Linger = TimeSpan.Zero;
            _sub.Bind($"tcp://*:{_commandSubPort}");
            _sub.Subscribe("");

            _statePub = new PublisherSocket();
            _statePub.Options.SendHighWatermark = Mathf.Max(1, _sendHwm);
            _statePub.Options.Linger = TimeSpan.Zero;
            _statePub.Bind($"tcp://*:{_statePubPort}");

            _depthPub = new PublisherSocket();
            _depthPub.Options.SendHighWatermark = Mathf.Max(1, _sendHwm);
            _depthPub.Options.Linger = TimeSpan.Zero;
            _depthPub.Bind($"tcp://*:{_depthPubPort}");

            if (_enableV4PrimitiveResultLifecycle) {
                _v4ResultTransport = new ZmqPrimitiveExecutionResultTransport(
                    _executionResultPort);
                _v4RuntimeIntegration.AttachResultTransport(_v4ResultTransport);
                _v4SnapshotTransport = new ZmqEndpointObservationSnapshotTransport(
                    _observationSnapshotPort, _runtimeInstanceId);
                _endpointObservationSnapshotService = new EndpointObservationSnapshotService(
                    _endpointObservationCache, _v4SnapshotTransport, _runtimeInstanceId);
                if (_reliableCommandPort > 0) {
                    _v4CommandAdmission = new PrimitiveExecutionCommandAdmission(
                        _runtimeInstanceId);
                    _v4CommandTransport = new ZmqPrimitiveExecutionCommandTransport(
                        _reliableCommandPort, _runtimeInstanceId);
                }
            }
        }

        private void PollV4CommandTransport() {
            if (!_enableV4PrimitiveResultLifecycle ||
                _v4CommandTransport == null || _v4CommandAdmission == null) return;

            byte[] raw;
            while (_v4CommandTransport.TryReceive(out raw)) {
                try {
                    PrimitiveResetV4Request reset =
                        PrimitiveResetV4WireCodec.DeserializeRequest(raw);
                    if (reset.schema_version == 4 &&
                        reset.message_type == "PrimitiveResetRequest") {
                        HandleV4ResetRequest(reset, raw);
                        continue;
                    }
                }
                catch (Exception) {
                    // This payload is a primitive command; its strict codec
                    // and admission path below remain authoritative.
                }
                try {
                    PrimitiveExecutionV4Command command =
                        PrimitiveExecutionCommandWireCodec.DeserializeCommand(raw);
                    PrimitiveExecutionCommandAdmissionResult admission =
                        _v4CommandAdmission.Register(command);
                    if (admission.Receipt != null &&
                        !_v4CommandTransport.SendReceipt(admission.Receipt))
                        Debug.LogWarning("[XMflight] reliable command receipt send failed");
                    if (admission.Outcome !=
                        PrimitiveExecutionCommandAdmissionOutcome.ACCEPTED)
                        continue;
                    if (command.execution_id > long.MaxValue)
                        throw new FormatException("execution_id exceeds Unity command range");

                    var frames = new PrimitiveExecutionFrameMsg[command.frames.Count];
                    for (int index = 0; index < frames.Length; ++index) {
                        PrimitiveExecutionV4Frame frame = command.frames[index];
                        frames[index] = new PrimitiveExecutionFrameMsg {
                            frame_index = (int)frame.frame_index,
                            command_id = frame.command_id,
                            action = frame.action,
                        };
                    }
                    HandleV4PrimitiveExecution(new ControlCommandMsg {
                        schema_version = 4,
                        mode = XMProtocol.ModePrimitiveExecution,
                        command_id = frames.Length > 0 ? frames[0].command_id : -1L,
                        execution_id = (long)command.execution_id,
                        execution_frame_index = -1,
                        execution_frame_count = frames.Length,
                        execution_frames = frames,
                    });
                }
                catch (Exception error) {
                    Debug.LogWarning(
                        $"[XMflight] reliable v4 command rejected: {error.Message}");
                }
            }
        }

        private static bool SameResetIdentity(
            PrimitiveResetV4Request left, PrimitiveResetV4Request right) {
            return left != null && right != null &&
                left.runtime_instance_id == right.runtime_instance_id &&
                left.episode_id == right.episode_id &&
                left.reset_id == right.reset_id;
        }

        private static bool SameResetPose(
            PrimitiveResetV4Request left, PrimitiveResetV4Request right) {
            if (!SameResetIdentity(left, right) || left.start == null || right.start == null ||
                left.goal == null || right.goal == null || left.start.Length != 3 ||
                right.start.Length != 3 || left.goal.Length != 3 || right.goal.Length != 3)
                return false;
            for (int index = 0; index < 3; ++index) {
                if (left.start[index] != right.start[index] ||
                    left.goal[index] != right.goal[index]) return false;
            }
            return true;
        }

        private void HandleV4ResetRequest(
            PrimitiveResetV4Request request, byte[] immutablePayload) {
            if (request == null || request.runtime_instance_id != _runtimeInstanceId ||
                request.start == null || request.start.Length != 3 ||
                request.goal == null || request.goal.Length != 3)
                throw new ArgumentException("reliable reset identity or pose is invalid");

            if (_pendingReset != null) {
                if (SameResetPose(_pendingReset, request)) {
                    if (_pendingResetComplete != null && _v4CommandTransport != null)
                        _v4CommandTransport.SendResetComplete(_pendingResetComplete);
                    return;
                }
                if (_pendingResetComplete == null)
                    throw new InvalidOperationException(
                        "conflicting reliable reset identity while capture is pending");
                _endpointCaptureOwnership?.Invalidate();
                _pendingReset = null;
                _pendingResetComplete = null;
            }

            _endpointEpisodeId = request.episode_id;
            _endpointResetId = request.reset_id;
            _pendingReset = request;
            _pendingResetStateId = -1L;
            _pendingResetStateBytes = null;
            _pendingResetCaptureQueued = false;
            _pendingResetCaptureId = -1L;
            _pendingResetComplete = null;

            HandleTeleport(new ControlCommandMsg {
                mode = XMProtocol.ModeTeleport,
                position = (float[])request.start.Clone(),
                action = new[] { 0.0f, 0.0f, 0.0f, 0.0f },
                execution_id = -1L,
            });
            Debug.Log(
                $"[XMflight] reliable reset accepted episode_id={request.episode_id} " +
                $"reset_id={request.reset_id}");
        }

        private void PollV4ResultTransport() {
            if (!_enableV4PrimitiveResultLifecycle ||
                _v4RuntimeIntegration == null || _v4ResultTransport == null) return;

            byte[] raw;
            while (_v4ResultTransport.TryReceive(out raw)) {
                try {
                    PrimitiveExecutionV4Ack ack =
                        PrimitiveExecutionResultWireCodec.DeserializeAck(raw);
                    PrimitiveExecutionAckOutcome outcome =
                        _v4RuntimeIntegration.ReceiveResultAck(ack);
                    Debug.Log($"[XMflight] v4 result ACK outcome={outcome}");
                }
                catch (Exception e) {
                    Debug.LogWarning($"[XMflight] v4 result ACK rejected: {e.Message}");
                }
            }

            try {
                _v4RuntimeIntegration.PollResultTransport(RuntimeNowMs());
            }
            catch (Exception e) {
                Debug.LogWarning($"[XMflight] v4 result retransmit failed: {e.Message}");
            }
        }

        private void PollV4SnapshotTransport() {
            if (!_enableV4PrimitiveResultLifecycle ||
                _v4SnapshotTransport == null ||
                _endpointObservationSnapshotService == null) return;

            byte[] raw;
            while (_v4SnapshotTransport.TryReceive(out raw)) {
                SnapshotRequestV4 request;
                if (EndpointObservationSnapshotWireCodec.TryDeserializeRequest(raw, out request)) {
                    try {
                        _endpointObservationSnapshotService.HandleRequest(request);
                    }
                    catch (Exception e) {
                        Debug.LogWarning($"[XMflight] snapshot request rejected: {e.Message}");
                    }
                    continue;
                }

                SnapshotAckV4 ack;
                if (EndpointObservationSnapshotWireCodec.TryDeserializeAck(raw, out ack)) {
                    try {
                        _endpointObservationSnapshotService.HandleAck(ack);
                    }
                    catch (Exception e) {
                        Debug.LogWarning($"[XMflight] snapshot ACK rejected: {e.Message}");
                    }
                }
            }
        }

        private void PollCommands() {
            while (_sub != null && _sub.TryReceiveFrameBytes(out byte[] raw)) {
                ControlCommandMsg cmd;
                object[] decodedMessage = null;

                try {
                    if (_enableV4PrimitiveResultLifecycle)
                        LogP0UnityLifecycle(
                            "COMMAND_FRAME_RECEIVED",
                            -1L,
                            -1L,
                            -1L,
                            -1,
                            "raw command frame received; bytes=" + raw.Length);
                    decodedMessage = MessagePackSerializer.Deserialize<object[]>(raw);
                    long receivedExecutionId = -1L;
                    if (_enableV4PrimitiveResultLifecycle &&
                        decodedMessage != null &&
                        decodedMessage.Length > XMProtocol.CommandIndex.ExecutionId) {
                        try {
                            receivedExecutionId = AsLong(
                                decodedMessage[XMProtocol.CommandIndex.ExecutionId]);
                        }
                        catch (Exception) { }
                    }
                    if (_enableV4PrimitiveResultLifecycle)
                        LogP0UnityLifecycle(
                            "COMMAND_FRAME_DECODED",
                            receivedExecutionId,
                            -1L,
                            -1L,
                            -1,
                            "raw command frame decoded; bytes=" + raw.Length);
                    object[] msg = decodedMessage;

                    if (msg == null || msg.Length < XMProtocol.CommandFieldCount) {
                        throw new FormatException(
                            $"Command field count invalid: {(msg == null ? 0 : msg.Length)}"
                        );
                    }

                    int schema = AsInt(msg[XMProtocol.CommandIndex.SchemaVersion]);
                    int expectedSchema = _enableV4PrimitiveResultLifecycle
                        ? 4
                        : XMProtocol.SchemaVersion;
                    if (schema != expectedSchema) {
                        throw new FormatException(
                            $"Command schema mismatch: {schema} != {expectedSchema}"
                        );
                    }

                    int mode = AsInt(msg[XMProtocol.CommandIndex.Mode]);
                    cmd = new ControlCommandMsg {
                        schema_version = schema,
                        mode = mode,
                        action = mode == XMProtocol.ModePrimitiveExecution
                            ? null
                            : AsFloatArray(msg[XMProtocol.CommandIndex.Action], 4, "action"),
                        position = AsOptionalFloatArray(
                            msg[XMProtocol.CommandIndex.Position],
                            3,
                            "position"
                        ),
                        client_time_ns = AsLong(msg[XMProtocol.CommandIndex.ClientTimeNs]),
                        command_id = AsLong(msg[XMProtocol.CommandIndex.CommandId]),
                        execution_id = AsLong(msg[XMProtocol.CommandIndex.ExecutionId]),
                        execution_frame_index = AsInt(msg[XMProtocol.CommandIndex.ExecutionFrameIndex]),
                        execution_frame_count = AsInt(msg[XMProtocol.CommandIndex.ExecutionFrameCount]),
                        execution_frames = mode == XMProtocol.ModePrimitiveExecution
                            ? AsPrimitiveExecutionFrames(
                                msg[XMProtocol.CommandIndex.ExecutionFrames],
                                AsInt(msg[XMProtocol.CommandIndex.ExecutionFrameCount]))
                            : null
                    };
                }
                catch (Exception e) {
                    _cmdDecodeErrors++;
                    if (_enableV4PrimitiveResultLifecycle &&
                        _v4RuntimeIntegration != null) {
                        ulong rejectedExecutionId = 0UL;
                        string rejectionReason = "MALFORMED_COMMAND";
                        if (decodedMessage != null &&
                            decodedMessage.Length > XMProtocol.CommandIndex.SchemaVersion) {
                            try {
                                int decodedSchema = AsInt(
                                    decodedMessage[XMProtocol.CommandIndex.SchemaVersion]);
                                if (decodedSchema != 4)
                                    rejectionReason = "SCHEMA_MISMATCH";
                            }
                            catch (Exception) {
                                rejectionReason = "MALFORMED_COMMAND";
                            }
                        }
                        if (decodedMessage != null &&
                            decodedMessage.Length > XMProtocol.CommandIndex.ExecutionId) {
                            try {
                                long decodedExecutionId = AsLong(
                                    decodedMessage[XMProtocol.CommandIndex.ExecutionId]);
                                if (decodedExecutionId >= 0)
                                    rejectedExecutionId = (ulong)decodedExecutionId;
                            }
                            catch (Exception) { }
                        }
                        _v4RuntimeIntegration.RejectBeforeExecution(
                            rejectedExecutionId,
                            ZeroHash(),
                            rejectionReason,
                            RuntimeNowMs());
                    }
                    if (_enableV4PrimitiveResultLifecycle)
                        Debug.LogError($"[XMflight] v4 command rejected with terminal result: {e.Message}");
                    else
                        Debug.LogWarning($"[XMflight] Command decode failed: {e.Message}");
                    continue;
                }

                _cmdReceived++;
                if (cmd.mode == XMProtocol.ModePrimitiveExecution) {
                    string commandSequenceHash = "";
                    try {
                        List<PrimitiveExecutionV4Frame> receivedFrames =
                            ToV4Frames(cmd.execution_frames);
                        commandSequenceHash = XMProtocolV4.Sha256Hex(
                            XMProtocolV4.CanonicalCommandSequence(receivedFrames));
                    }
                    catch (Exception) {
                        // Diagnostics must not change command handling.
                    }
                    LogP0UnityLifecycle(
                        "EXECUTION_RECEIVED",
                        cmd.execution_id,
                        -1L,
                        -1L,
                        cmd.execution_frame_index,
                        "decoded primitive command; bytes=" + raw.Length +
                        "; command_sequence_hash=" + commandSequenceHash);
                }

                if (cmd.mode == XMProtocol.ModeTeleport) {
                    _modeTeleportCount++;
                    HandleTeleport(cmd);
                }
                else if (cmd.mode == XMProtocol.ModeVelocity) {
                    _modeVelocityCount++;
                    HandleVelocity(cmd);
                }
                else if (cmd.mode == XMProtocol.ModeStep) {
                    _modeStepCount++;
                    HandleStep(cmd);
                }
                else if (cmd.mode == XMProtocol.ModeTrajectory) {
                    _modeTrajectoryCount++;
                    HandleExternalTrajectory(cmd);
                }
                else if (cmd.mode == XMProtocol.ModePrimitiveExecution) {
                    HandlePrimitiveExecution(cmd);
                }
                else {
                    Debug.LogWarning($"[XMflight] Unknown command mode: {cmd.mode}");
                }
            }
        }

        private static int AsInt(object value) {
            if (value == null)
                throw new InvalidCastException("Cannot cast null to int.");

            return Convert.ToInt32(value);
        }

        private static long AsLong(object value) {
            if (value == null)
                throw new InvalidCastException("Cannot cast null to long.");

            return Convert.ToInt64(value);
        }

        private static float AsFloat(object value) {
            if (value == null)
                throw new InvalidCastException("Cannot cast null to float.");

            return Convert.ToSingle(value);
        }

        private static float[] AsOptionalFloatArray(object value, int minLength, string name) {
            if (value == null)
                return null;

            return AsFloatArray(value, minLength, name);
        }

        private static float[] AsFloatArray(object value, int minLength, string name) {
            if (value == null)
                throw new FormatException($"{name} is null.");

            if (value is float[] f) {
                if (f.Length < minLength)
                    throw new FormatException($"{name} length invalid: {f.Length} < {minLength}");
                return f;
            }

            if (value is double[] d) {
                if (d.Length < minLength)
                    throw new FormatException($"{name} length invalid: {d.Length} < {minLength}");

                float[] outArr = new float[d.Length];
                for (int i = 0; i < d.Length; i++)
                    outArr[i] = (float)d[i];

                return outArr;
            }

            if (value is object[] o) {
                if (o.Length < minLength)
                    throw new FormatException($"{name} length invalid: {o.Length} < {minLength}");

                float[] outArr = new float[o.Length];
                for (int i = 0; i < o.Length; i++)
                    outArr[i] = AsFloat(o[i]);

                return outArr;
            }

            if (value is IList list) {
                if (list.Count < minLength)
                    throw new FormatException($"{name} length invalid: {list.Count} < {minLength}");

                float[] outArr = new float[list.Count];
                for (int i = 0; i < list.Count; i++)
                    outArr[i] = AsFloat(list[i]);

                return outArr;
            }

            throw new FormatException($"{name} type invalid: {value.GetType()}");
        }

        private static PrimitiveExecutionFrameMsg[] AsPrimitiveExecutionFrames(
            object value,
            int expectedCount
        ) {
            if (expectedCount <= 0)
                throw new FormatException($"execution frame count invalid: {expectedCount}");
            if (!(value is IList frames) || frames.Count != expectedCount)
                throw new FormatException(
                    $"execution frames invalid: count={(value is IList l ? l.Count : -1)} expected={expectedCount}"
                );

            PrimitiveExecutionFrameMsg[] decoded = new PrimitiveExecutionFrameMsg[expectedCount];
            System.Collections.Generic.HashSet<long> commandIds =
                new System.Collections.Generic.HashSet<long>();
            for (int i = 0; i < expectedCount; i++) {
                if (!(frames[i] is IList frame) || frame.Count != 3)
                    throw new FormatException($"execution frame {i} must contain index, command_id, action");
                int frameIndex = AsInt(frame[0]);
                long commandId = AsLong(frame[1]);
                if (frameIndex != i)
                    throw new FormatException($"execution frame order invalid: got={frameIndex} expected={i}");
                if (!commandIds.Add(commandId))
                    throw new FormatException($"duplicate execution command_id: {commandId}");
                decoded[i] = new PrimitiveExecutionFrameMsg {
                    frame_index = frameIndex,
                    command_id = commandId,
                    action = AsFloatArray(frame[2], 4, $"execution frame {i} action")
                };
            }
            return decoded;
        }

        private static List<PrimitiveExecutionV4Frame> ToV4Frames(
            PrimitiveExecutionFrameMsg[] source) {
            var frames = new List<PrimitiveExecutionV4Frame>(source.Length);
            for (int index = 0; index < source.Length; ++index) {
                PrimitiveExecutionFrameMsg frame = source[index];
                frames.Add(new PrimitiveExecutionV4Frame {
                    frame_index = (uint)frame.frame_index,
                    command_id = frame.command_id,
                    action = (float[])frame.action.Clone()
                });
            }
            return frames;
        }

        private static byte[] ZeroHash() {
            return new byte[32];
        }

        private static ulong RuntimeNowMs() {
            double milliseconds = Time.realtimeSinceStartupAsDouble * 1000.0;
            if (milliseconds <= 0.0) return 0UL;
            return (ulong)milliseconds;
        }

        private static long DiagnosticCaptureId(string depthId) {
            if (string.IsNullOrEmpty(depthId) || !depthId.StartsWith("depth-"))
                return -1L;
            long value;
            return long.TryParse(depthId.Substring("depth-".Length), out value)
                ? value
                : -1L;
        }

        private void LogP0UnityLifecycle(
            string eventName,
            long executionId,
            long captureId,
            long stateId,
            int frameIndex,
            string detail) {
            double monotonicNs = Time.realtimeSinceStartupAsDouble *
                XMConstants.SecondsToNanoseconds;
            long activeExecutionId = _activeExecution != null
                ? _activeExecution.executionId
                : -1L;
            bool integrationActive = _v4RuntimeIntegration != null &&
                _v4RuntimeIntegration.IsActive;
            Debug.Log(
                $"[DEBUG-P0-UNITY-LIFECYCLE] {eventName} " +
                $"execution_id={executionId} capture_id={captureId} state_id={stateId} " +
                $"frame_index={frameIndex} monotonic_ns={(long)monotonicNs} " +
                $"thread_id={System.Threading.Thread.CurrentThread.ManagedThreadId} " +
                $"active_execution_present={(_activeExecution != null ? "true" : "false")} " +
                $"active_execution_id={activeExecutionId} " +
                $"v4_runtime_active={(integrationActive ? "true" : "false")} " +
                $"episode_id={_endpointEpisodeId} reset_id={_endpointResetId} " +
                $"runtime_id={_runtimeInstanceId} " +
                $"detail={detail ?? ""}"
            );
        }

        private void OnCaptureLifecycleDiagnostic(
            XMImageSynthesis.CaptureLifecycleDiagnostic diagnostic) {
            if (diagnostic == null) return;
            long executionId = diagnostic.sourceExecutionId;
            long stateId = diagnostic.endpointStateId;
            int frameIndex = diagnostic.sourceFrameIndex;
            if (diagnostic.snapshot != null) {
                executionId = diagnostic.snapshot.endpointSourceExecutionId;
                stateId = diagnostic.snapshot.endpointStateId;
                frameIndex = diagnostic.snapshot.endpointSourceFrameIndex;
            }
            LogP0UnityLifecycle(
                diagnostic.eventName,
                executionId,
                diagnostic.captureId,
                stateId,
                frameIndex,
                diagnostic.detail);

            if (IsEndpointCaptureFailureEvent(diagnostic.eventName) &&
                _endpointCaptureOwnership != null &&
                diagnostic.captureId >= 0 &&
                !string.IsNullOrEmpty(diagnostic.snapshot != null
                    ? diagnostic.snapshot.endpointRuntimeInstanceId
                    : _runtimeInstanceId)) {
                EndpointObservationCaptureIdentity identity = diagnostic.snapshot != null
                    ? CaptureIdentity(diagnostic.snapshot)
                    : new EndpointObservationCaptureIdentity {
                        runtime_instance_id = _runtimeInstanceId,
                        execution_id = executionId,
                        episode_id = _endpointEpisodeId,
                        reset_id = _endpointResetId,
                        endpoint_state_id = stateId,
                        source_frame_index = frameIndex,
                        capture_id = diagnostic.captureId,
                    };
                EndpointObservationCaptureAcceptOutcome outcome =
                    _endpointCaptureOwnership.TryAccept(identity);
                if (outcome == EndpointObservationCaptureAcceptOutcome.ACCEPTED) {
                    FailEndpointObservation(
                        "PHYSICS_COMPLETE_OBSERVATION_FAILED",
                        stateId,
                        frameIndex,
                        diagnostic.eventName + ": " + diagnostic.detail);
                }
            }
        }

        private void OnV4TerminalResultGenerated(
            PrimitiveExecutionResultTransmission transmission) {
            PrimitiveExecutionV4ObservationRef observation =
                transmission != null ? transmission.EndpointObservationRef : null;
            LogP0UnityLifecycle(
                transmission != null && transmission.Status == "COMPLETE"
                    ? "COMPLETE_GENERATED"
                    : "TERMINAL_RESULT_GENERATED",
                transmission != null ? (long)transmission.ExecutionId : -1L,
                observation != null ? DiagnosticCaptureId(observation.depth_id) : -1L,
                transmission != null && transmission.EndpointStateId.HasValue
                    ? transmission.EndpointStateId.Value
                    : -1L,
                transmission != null ? transmission.LastAppliedFrameIndex : -1,
                transmission != null
                    ? "status=" + transmission.Status + " reason=" + transmission.ReasonCode
                    : "terminal result missing");
        }

        private void OnV4TerminalResultFirstSend(
            PrimitiveExecutionResultTransmission transmission) {
            PrimitiveExecutionV4ObservationRef observation =
                transmission != null ? transmission.EndpointObservationRef : null;
            LogP0UnityLifecycle(
                "RESULT_FIRST_SEND",
                transmission != null ? (long)transmission.ExecutionId : -1L,
                observation != null ? DiagnosticCaptureId(observation.depth_id) : -1L,
                transmission != null && transmission.EndpointStateId.HasValue
                    ? transmission.EndpointStateId.Value
                    : -1L,
                transmission != null ? transmission.LastAppliedFrameIndex : -1,
                transmission != null ? "status=" + transmission.Status : "terminal result missing");
        }

        private void OnV4TerminalResultSendFailed(
            PrimitiveExecutionResultTransmission transmission,
            Exception error) {
            PrimitiveExecutionV4ObservationRef observation =
                transmission != null ? transmission.EndpointObservationRef : null;
            LogP0UnityLifecycle(
                "RESULT_SEND_FAILED",
                transmission != null ? (long)transmission.ExecutionId : -1L,
                observation != null ? DiagnosticCaptureId(observation.depth_id) : -1L,
                transmission != null && transmission.EndpointStateId.HasValue
                    ? transmission.EndpointStateId.Value
                    : -1L,
                transmission != null ? transmission.LastAppliedFrameIndex : -1,
                (transmission != null ? "status=" + transmission.Status + " " : "") +
                "error=" + (error != null ? error.Message : "unknown"));
        }

        private void ClearExecutionAcknowledgement() {
            _appliedExecutionId = -1;
            _appliedExecutionFrameIndex = -1;
            _appliedCommandId = -1;
            _executionStatus = XMProtocol.ExecutionStatusNone;
        }

        private void DisableDeterministicExecution(
            string cancellationReason = "EXTERNAL_OVERRIDE") {
            bool physicsCompleteAwaitingObservation =
                _enableV4PrimitiveResultLifecycle &&
                _v4RuntimeIntegration != null &&
                _v4RuntimeIntegration.IsActive &&
                _v4RuntimeIntegration.AppliedFrameCount == 25U &&
                _endpointObservationCaptureQueued;
            if (_endpointCaptureOwnership != null)
                _endpointCaptureOwnership.Invalidate();
            _endpointObservationCaptureQueued = false;
            _endpointObservationCaptureDeadlineRealtime = -1f;
            _pendingV4TerminalFailure = false;
            _pendingV4TerminalFailureExecutionId = -1L;
            _pendingV4TerminalFailureReason = "";
            _pendingV4TerminalFailureStateId = -1L;
            _endpointStateBytes.Clear();
            if (_enableV4PrimitiveResultLifecycle &&
                _v4RuntimeIntegration != null &&
                _v4RuntimeIntegration.IsActive) {
                if (physicsCompleteAwaitingObservation) {
                    _v4RuntimeIntegration.FailActiveExecution(
                        "PHYSICS_COMPLETE_OBSERVATION_FAILED", RuntimeNowMs());
                }
                else {
                    _v4RuntimeIntegration.CancelActiveExecution(
                        cancellationReason, RuntimeNowMs());
                }
            }
            _deterministicExecutionMode = false;
            _activeExecution = null;
            _clearExecutionAfterPublish = false;
            ClearExecutionAcknowledgement();
        }

        private void HandlePrimitiveExecution(ControlCommandMsg cmd) {
            if (_enableV4PrimitiveResultLifecycle) {
                HandleV4PrimitiveExecution(cmd);
                return;
            }

            if (cmd.execution_id < 0 || cmd.execution_frame_index != -1 ||
                cmd.execution_frame_count <= 0 || cmd.execution_frames == null ||
                cmd.execution_frames.Length != cmd.execution_frame_count) {
                Debug.LogWarning($"[XMflight] Rejecting malformed primitive execution {cmd.execution_id}");
                return;
            }
            if (_ctrlLatencyFrames != 0) {
                Debug.LogError(
                    $"[XMflight] Deterministic primitive execution requires ctrlLatencyFrames=0, got {_ctrlLatencyFrames}"
                );
                return;
            }
            if (_activeExecution != null) {
                Debug.LogWarning(
                    $"[XMflight] Rejecting overlapping primitive execution {cmd.execution_id}; " +
                    $"active={_activeExecution.executionId}"
                );
                return;
            }

            _deterministicExecutionMode = true;
            _activeExecution = new BufferedPrimitiveExecution {
                executionId = cmd.execution_id,
                frames = cmd.execution_frames,
                nextFrameIndex = 0
            };
            _endpointObservationCaptureQueued = false;
            _endpointObservationCaptureDeadlineRealtime = -1f;
            _pendingV4TerminalFailure = false;
            _pendingV4TerminalFailureExecutionId = -1L;
            _pendingV4TerminalFailureReason = "";
            _pendingV4TerminalFailureStateId = -1L;
            _cmdQueue.Clear();
            _lastExecutedCmd = default;
            _hasPendingStepTarget = false;
            _hasReceivedMotionCommand = false;
            _frozen = false;
            _collisionLatchTimer = 0f;
            LogP0UnityLifecycle(
                "EXECUTION_BEGIN",
                cmd.execution_id,
                -1L,
                -1L,
                -1,
                "v4 primitive admitted");
        }

        private void HandleV4PrimitiveExecution(ControlCommandMsg cmd) {
            ulong executionId = cmd.execution_id >= 0 ? (ulong)cmd.execution_id : 0UL;
            if (cmd.execution_id < 0 || cmd.execution_frame_index != -1 ||
                cmd.execution_frame_count <= 0 || cmd.execution_frames == null ||
                cmd.execution_frames.Length != cmd.execution_frame_count) {
                _v4RuntimeIntegration.RejectBeforeExecution(
                    executionId, ZeroHash(), "MALFORMED_COMMAND", RuntimeNowMs());
                return;
            }
            if (_ctrlLatencyFrames != 0) {
                _v4RuntimeIntegration.RejectBeforeExecution(
                    executionId, ZeroHash(), "CTRL_LATENCY_NOT_ZERO", RuntimeNowMs());
                return;
            }
            if (cmd.execution_frame_count != 25) {
                _v4RuntimeIntegration.RejectBeforeExecution(
                    executionId, ZeroHash(), "INVALID_FRAME_COUNT", RuntimeNowMs());
                return;
            }

            List<PrimitiveExecutionV4Frame> v4Frames;
            try {
                v4Frames = ToV4Frames(cmd.execution_frames);
                XMProtocolV4.CanonicalCommandSequence(v4Frames);
            }
            catch (Exception) {
                _v4RuntimeIntegration.RejectBeforeExecution(
                    executionId, ZeroHash(), "MALFORMED_COMMAND", RuntimeNowMs());
                return;
            }
            // RuntimeIntegration.IsActive is authoritative for v4 admission.
            // The manager may clear _activeExecution after frame 24 while the
            // endpoint observation callback still owns the execution.
            if (_v4RuntimeIntegration.IsActive) {
                _v4RuntimeIntegration.RejectExecutionIfBusy(
                    executionId, v4Frames, RuntimeNowMs());
                return;
            }

            _v4RuntimeIntegration.BeginExecution(
                executionId, v4Frames, RuntimeNowMs());
            if (_endpointCaptureOwnership != null)
                _endpointCaptureOwnership.Invalidate();
            _pendingV4TerminalFailure = false;
            _pendingV4TerminalFailureExecutionId = -1L;
            _pendingV4TerminalFailureReason = "";
            _pendingV4TerminalFailureStateId = -1L;
            _deterministicExecutionMode = true;
            _activeExecution = new BufferedPrimitiveExecution {
                executionId = cmd.execution_id,
                frames = cmd.execution_frames,
                nextFrameIndex = 0
            };
            _endpointObservationCaptureQueued = false;
            _cmdQueue.Clear();
            _lastExecutedCmd = default;
            _hasPendingStepTarget = false;
            _hasReceivedMotionCommand = false;
            _frozen = false;
            _collisionLatchTimer = 0f;
        }

        private void ApplyNextPrimitiveExecutionFrame() {
            int index = _activeExecution.nextFrameIndex;
            PrimitiveExecutionFrameMsg frame = _activeExecution.frames[index];
            ApplyPrimitiveAction(frame.action);

            _appliedExecutionId = _activeExecution.executionId;
            _appliedExecutionFrameIndex = frame.frame_index;
            _appliedCommandId = frame.command_id;
            bool complete = index == _activeExecution.frames.Length - 1;
            _executionStatus = complete
                ? XMProtocol.ExecutionStatusComplete
                : XMProtocol.ExecutionStatusFrameApplied;
            _activeExecution.nextFrameIndex++;
            _clearExecutionAfterPublish = complete;
            if (complete)
                LogP0UnityLifecycle(
                    "FRAME24_APPLIED",
                    _activeExecution.executionId,
                    -1L,
                    -1L,
                    frame.frame_index,
                    "state identity assigned during PublishDynamicsState");
        }

        private void FailPrimitiveExecution() {
            if (_enableV4PrimitiveResultLifecycle &&
                _v4RuntimeIntegration != null &&
                _v4RuntimeIntegration.IsActive) {
                // Keep the lifecycle active until the exact terminal state and
                // depth snapshot have been captured.  The FAILED result is
                // immutable and must bind that snapshot before generation.
                _pendingV4TerminalFailure = true;
                _pendingV4TerminalFailureExecutionId = _activeExecution.executionId;
                _pendingV4TerminalFailureReason = "COLLISION";
            }
            _appliedExecutionId = _activeExecution.executionId;
            _appliedExecutionFrameIndex = -1;
            _appliedCommandId = -1;
            _executionStatus = XMProtocol.ExecutionStatusFailed;
            _clearExecutionAfterPublish = true;
        }

        private void ApplyPrimitiveAction(float[] action) {
            Vector3 bodyUnity = XMConverters.RosBodyVelocityToUnityBody(
                action[0],
                action[1],
                action[2]
            );
            Quaternion yawRot = Quaternion.Euler(0f, _drone.rotation.eulerAngles.y, 0f);
            _targetVelWorld = yawRot * bodyUnity;
            _targetYawRate = -action[3];
            _hasPendingStepTarget = false;
        }

        private void HandleTeleport(ControlCommandMsg cmd) {
            if (cmd.position == null || cmd.position.Length < 3)
                return;

            DisableDeterministicExecution("RESET");

            _drone.position = XMConverters.RosToUnityPos(cmd.position);

            if (cmd.action != null && cmd.action.Length >= 4)
                _yawDeg = -cmd.action[3] * Mathf.Rad2Deg;
            else
                _yawDeg = _drone.eulerAngles.y;

            _drone.rotation = Quaternion.Euler(0f, _yawDeg, 0f);
            _smoothPitchDeg = 0f;
            _smoothRollDeg = 0f;

            _targetVelWorld = Vector3.zero;
            _targetYawRate = 0f;

            _realVelocity = Vector3.zero;
            _prevVelocity = Vector3.zero;
            _filteredAcc = Vector3.zero;

            _frozen = false;
            _collisionLatchTimer = 0f;

            _cmdQueue.Clear();
            _lastExecutedCmd = default;

            _hasPendingStepTarget = false;

            _prevFixedPos = _drone.position;
            _maxFixedDelta = 0f;
            _avgFixedDelta = 0f;
            _fixedDeltaCount = 0;

            _modeVelocityCount = 0;
            _modeStepCount = 0;
            _modeTrajectoryCount = 0;
            _modeTeleportCount = 0;
            _hasReceivedMotionCommand = false;
        }

        private void MarkMotionCommandReceived() {
            _lastCommandRealtime = Time.realtimeSinceStartup;
            _hasReceivedMotionCommand = true;
        }

        private void HandleVelocity(ControlCommandMsg cmd) {
            if (cmd.action == null || cmd.action.Length < 4)
                return;

            DisableDeterministicExecution();
            MarkMotionCommandReceived();

            if (_frozen) {
                _frozen = false;
                _collisionLatchTimer = 0f;
            }

            Vector3 bodyUnity = XMConverters.RosBodyVelocityToUnityBody(
                cmd.action[0],
                cmd.action[1],
                cmd.action[2]
            );

            Quaternion yawRot = Quaternion.Euler(0f, _drone.rotation.eulerAngles.y, 0f);
            _targetVelWorld = yawRot * bodyUnity;
            _targetYawRate = -cmd.action[3];

            _hasPendingStepTarget = false;
        }

        private void HandleStep(ControlCommandMsg cmd) {
            if (cmd.action == null || cmd.action.Length < 4)
                return;

            DisableDeterministicExecution();
            MarkMotionCommandReceived();

            if (_frozen) {
                _frozen = false;
                _collisionLatchTimer = 0f;
            }

            Vector3 bodyUnity = XMConverters.RosBodyVelocityToUnityBody(
                cmd.action[0],
                cmd.action[1],
                cmd.action[2]
            );

            Quaternion yawRot = Quaternion.Euler(0f, _drone.rotation.eulerAngles.y, 0f);
            _targetVelWorld = yawRot * bodyUnity;
            _targetYawRate = -cmd.action[3];

            if (cmd.position != null && cmd.position.Length >= 3) {
                _pendingStepTargetWorld = XMConverters.RosToUnityPos(cmd.position);
                _hasPendingStepTarget = true;
            }
            else {
                _hasPendingStepTarget = false;
            }
        }

        private void HandleExternalTrajectory(ControlCommandMsg cmd) {
            HandleStep(cmd);
        }

        private void ApplyCommandTimeoutBrake() {
            if (!_zeroCommandOnTimeout || _commandTimeoutSec <= 0f || !_hasReceivedMotionCommand)
                return;

            if (Time.realtimeSinceStartup - _lastCommandRealtime <= _commandTimeoutSec)
                return;

            _targetVelWorld = Vector3.zero;
            _targetYawRate = 0f;
            _hasPendingStepTarget = false;
            _cmdQueue.Clear();
            _lastExecutedCmd = default;
            _hasReceivedMotionCommand = false;
        }

        private void StepDynamics() {
            float dt = Time.fixedDeltaTime;

            _cmdQueue.Enqueue(new DelayedCmd {
                linearVelWorld = _targetVelWorld,
                yawRate = _targetYawRate,
                hasStepTarget = _hasPendingStepTarget,
                stepTargetPosWorld = _pendingStepTargetWorld
            });

            if (_cmdQueue.Count > _ctrlLatencyFrames)
                _lastExecutedCmd = _cmdQueue.Dequeue();

            DelayedCmd cmd = _lastExecutedCmd;

            Vector3 clampedVel = cmd.linearVelWorld;

            if (cmd.hasStepTarget) {
                Vector3 posErr = cmd.stepTargetPosWorld - _drone.position;

                if (posErr.magnitude > _stepPosErrClamp)
                    posErr = posErr.normalized * _stepPosErrClamp;

                clampedVel += _stepPosKp * posErr;
            }

            if (clampedVel.sqrMagnitude > _maxSpeed * _maxSpeed)
                clampedVel = clampedVel.normalized * _maxSpeed;

            float alphaXY = dt / (_timeConstantXY + dt);
            float alphaZ = dt / (_timeConstantZ + dt);

            Vector3 oldVelocity = _realVelocity;

            _realVelocity.x = Mathf.Lerp(_realVelocity.x, clampedVel.x, alphaXY);
            _realVelocity.z = Mathf.Lerp(_realVelocity.z, clampedVel.z, alphaXY);
            _realVelocity.y = Mathf.Lerp(_realVelocity.y, clampedVel.y, alphaZ);

            Vector3 nextPos = _drone.position + (oldVelocity + _realVelocity) * 0.5f * dt;

            if (!IsVectorValid(nextPos) || !IsVectorValid(_realVelocity)) {
                Debug.LogWarning("[XMflight] Invalid position or velocity. Dynamics reset.");

                _realVelocity = Vector3.zero;
                _prevVelocity = Vector3.zero;
                _filteredAcc = Vector3.zero;

                nextPos = _drone.position;
            }

            _drone.position = nextPos;

            float fixedDelta = Vector3.Distance(_prevFixedPos, _drone.position);
            _prevFixedPos = _drone.position;

            _maxFixedDelta = Mathf.Max(_maxFixedDelta, fixedDelta);
            _fixedDeltaCount++;
            _avgFixedDelta += (fixedDelta - _avgFixedDelta) / Mathf.Max(1, _fixedDeltaCount);

            Vector3 rawAcc = (_realVelocity - oldVelocity) / dt;
            _prevVelocity = _realVelocity;

            _filteredAcc = Vector3.Lerp(
                _filteredAcc,
                rawAcc,
                dt / (_accFilterTime + dt)
            );

            if (!IsVectorValid(_filteredAcc))
                _filteredAcc = Vector3.zero;

            Vector3 localAcc = _drone.InverseTransformDirection(_filteredAcc);

            float targetPitch = Mathf.Clamp(
                Mathf.Atan2(localAcc.z, XMConstants.Gravity) * Mathf.Rad2Deg,
                -_maxTiltAngle,
                _maxTiltAngle
            ) * _visualTiltScale;

            float targetRoll = Mathf.Clamp(
                -Mathf.Atan2(localAcc.x, XMConstants.Gravity) * Mathf.Rad2Deg,
                -_maxTiltAngle,
                _maxTiltAngle
            ) * _visualTiltScale;

            _yawDeg += cmd.yawRate * Mathf.Rad2Deg * dt;

            if (!IsFloatValid(targetPitch) || !IsFloatValid(targetRoll) || !IsFloatValid(_yawDeg)) {
                Debug.LogWarning("[XMflight] Invalid Euler angle. Rotation reset.");

                targetPitch = 0f;
                targetRoll = 0f;
                _smoothPitchDeg = 0f;
                _smoothRollDeg = 0f;
                _yawDeg = _drone.eulerAngles.y;
            }

            float attitudeAlpha = dt / (Mathf.Max(0f, _attitudeFilterTime) + dt);
            float filteredPitch = Mathf.Lerp(_smoothPitchDeg, targetPitch, attitudeAlpha);
            float filteredRoll = Mathf.Lerp(_smoothRollDeg, targetRoll, attitudeAlpha);
            float maxTiltStep = Mathf.Max(0f, _maxTiltRate) * dt;

            if (maxTiltStep > 0f) {
                _smoothPitchDeg = Mathf.MoveTowards(_smoothPitchDeg, filteredPitch, maxTiltStep);
                _smoothRollDeg = Mathf.MoveTowards(_smoothRollDeg, filteredRoll, maxTiltStep);
            }
            else {
                _smoothPitchDeg = filteredPitch;
                _smoothRollDeg = filteredRoll;
            }

            Quaternion targetRot =
                Quaternion.Euler(0f, _yawDeg, 0f) *
                Quaternion.Euler(_smoothPitchDeg, 0f, _smoothRollDeg);

            if (!IsQuaternionValid(targetRot))
                targetRot = _drone.rotation;

            _drone.rotation = Quaternion.RotateTowards(
                _drone.rotation,
                targetRot,
                _maxAngularRate * dt
            );
        }

        private Vector3 AddNoise(Vector3 v, float std) {
            if (std <= 0f) return v;

            return v + new Vector3(
                Gauss(std),
                Gauss(std),
                Gauss(std)
            );
        }

        private static float Gauss(float s) {
            float u1 = 1f - UnityEngine.Random.value;
            float u2 = 1f - UnityEngine.Random.value;

            return s *
                   Mathf.Sqrt(-2f * Mathf.Log(u1)) *
                   Mathf.Sin(2f * Mathf.PI * u2);
        }

        private void PublishDynamicsState() {
            _publishCounter++;
            bool executionAcknowledgementDue = _appliedExecutionId >= 0;
            if (!executionAcknowledgementDue &&
                _publishStride > 1 && (_publishCounter % _publishStride) != 0) return;
            if (_statePub == null || _drone == null) {
                RecordExecutionTransportAudit(
                    stateId: -1,
                    simTimeNs: 0,
                    serializationAttempted: false,
                    serializationSuccess: false,
                    statePublishAttempted: false,
                    trySendReturn: -1
                );
                return;
            }

            int flags = 0;
            bool collided = (_col != null && _col.HasCollided) || _collisionLatchTimer > 0f;
            if (collided)
                flags |= XMProtocol.FlagCollision;
            if (IsAltitudeViolation(_drone.position))
                flags |= XMProtocol.FlagAltitudeViolation;

            float minClearance = _col != null ? _col.MinClearance : 2f;
            CopyFrontClearances(_col != null ? _col.FrontClearances : null, minClearance, _stateFrontClearances);

            XMConverters.UnityToRosPosArrayNonAlloc(AddNoise(_drone.position, _posNoiseStdDev), _statePosRos);
            XMConverters.UnityToRosRotArrayNonAlloc(_drone.rotation, _stateRotRos);
            XMConverters.UnityToRosPosArrayNonAlloc(AddNoise(_realVelocity, _obsNoiseStdDev), _stateVelRos);
            XMConverters.UnityToRosPosArrayNonAlloc(AddNoise(_filteredAcc, _obsNoiseStdDev), _stateAccRos);

            _stateMsg[XMProtocol.DynamicsStateIndex.SchemaVersion] = XMProtocol.SchemaVersion;
            _stateMsg[XMProtocol.DynamicsStateIndex.StateId] = ++_stateId;
            _stateMsg[XMProtocol.DynamicsStateIndex.SimTimeNs] =
                (long)(Time.fixedTimeAsDouble * XMConstants.SecondsToNanoseconds);
            _stateMsg[XMProtocol.DynamicsStateIndex.Flags] = flags;
            _stateMsg[XMProtocol.DynamicsStateIndex.MinClearance] = minClearance;
            _stateMsg[XMProtocol.DynamicsStateIndex.CurrPos] = _statePosRos;
            _stateMsg[XMProtocol.DynamicsStateIndex.CurrRot] = _stateRotRos;
            _stateMsg[XMProtocol.DynamicsStateIndex.CurrVel] = _stateVelRos;
            _stateMsg[XMProtocol.DynamicsStateIndex.CurrAcc] = _stateAccRos;
            _stateMsg[XMProtocol.DynamicsStateIndex.FrontClearances] = _stateFrontClearances;
            _stateMsg[XMProtocol.DynamicsStateIndex.AppliedExecutionId] = _appliedExecutionId;
            _stateMsg[XMProtocol.DynamicsStateIndex.AppliedExecutionFrameIndex] =
                _appliedExecutionFrameIndex;
            _stateMsg[XMProtocol.DynamicsStateIndex.AppliedCommandId] = _appliedCommandId;
            _stateMsg[XMProtocol.DynamicsStateIndex.ExecutionStatus] = _executionStatus;
            _stateMsg[XMProtocol.DynamicsStateIndex.EpisodeId] = _endpointEpisodeId;
            _stateMsg[XMProtocol.DynamicsStateIndex.ResetId] = _endpointResetId;
            _stateMsg[XMProtocol.DynamicsStateIndex.RuntimeInstanceId] = _runtimeInstanceId;

            if (_enableV4PrimitiveResultLifecycle &&
                _v4RuntimeIntegration != null &&
                _v4RuntimeIntegration.IsActive &&
                _activeExecution != null &&
                _appliedExecutionFrameIndex >= 0) {
                ulong simTimeNs = (ulong)Math.Max(
                    0L,
                    (long)_stateMsg[XMProtocol.DynamicsStateIndex.SimTimeNs]);
                _v4RuntimeIntegration.RecordAppliedFrame(
                    (uint)_appliedExecutionFrameIndex,
                    _stateId,
                    simTimeNs,
                    RuntimeNowMs());
                if (_appliedExecutionFrameIndex == 24 &&
                    !_endpointObservationCaptureQueued) {
                    if (_cam == null || _endpointCaptureOwnership == null) {
                        FailEndpointObservation(
                            "PHYSICS_COMPLETE_OBSERVATION_FAILED", _stateId, 24,
                            "frame-24 camera or ownership service unavailable");
                    }
                    else {
                        try {
                            long captureId = _cam.QueueEndpointObservationCapture(
                                _stateId,
                                (long)simTimeNs,
                                _endpointEpisodeId,
                                _endpointResetId,
                                _runtimeInstanceId,
                                _appliedExecutionId,
                                _appliedExecutionFrameIndex);
                            _endpointCaptureOwnership.Begin(
                                new EndpointObservationCaptureIdentity {
                                    runtime_instance_id = _runtimeInstanceId,
                                    execution_id = _appliedExecutionId,
                                    episode_id = _endpointEpisodeId,
                                    reset_id = _endpointResetId,
                                    endpoint_state_id = _stateId,
                                    source_frame_index = _appliedExecutionFrameIndex,
                                    capture_id = captureId,
                                });
                            _endpointObservationCaptureQueued = true;
                            _endpointObservationCaptureDeadlineRealtime =
                                Time.realtimeSinceStartup + EndpointObservationCaptureTimeoutSec;
                        }
                        catch (Exception error) {
                            FailEndpointObservation(
                                "PHYSICS_COMPLETE_OBSERVATION_FAILED",
                                _stateId,
                                24,
                                "endpoint capture queue failed: " + error.Message);
                        }
                    }
                }
            }

            if (_enableV4PrimitiveResultLifecycle &&
                _v4RuntimeIntegration != null &&
                _v4RuntimeIntegration.IsActive &&
                _pendingV4TerminalFailure &&
                !_endpointObservationCaptureQueued) {
                if (_cam == null || _endpointCaptureOwnership == null) {
                    FailEndpointObservation(
                        "PHYSICS_COMPLETE_OBSERVATION_FAILED", _stateId, -1,
                        "terminal failure camera or ownership service unavailable");
                }
                else {
                    try {
                        long captureId = _cam.QueueEndpointObservationCapture(
                            _stateId,
                            Convert.ToInt64(_stateMsg[XMProtocol.DynamicsStateIndex.SimTimeNs]),
                            _endpointEpisodeId,
                            _endpointResetId,
                            _runtimeInstanceId,
                            _pendingV4TerminalFailureExecutionId,
                            -1);
                        _endpointCaptureOwnership.Begin(
                            new EndpointObservationCaptureIdentity {
                                runtime_instance_id = _runtimeInstanceId,
                                execution_id = _pendingV4TerminalFailureExecutionId,
                                episode_id = _endpointEpisodeId,
                                reset_id = _endpointResetId,
                                endpoint_state_id = _stateId,
                                source_frame_index = -1,
                                capture_id = captureId,
                            });
                        _pendingV4TerminalFailureStateId = _stateId;
                        _endpointObservationCaptureQueued = true;
                        _endpointObservationCaptureDeadlineRealtime =
                            Time.realtimeSinceStartup + EndpointObservationCaptureTimeoutSec;
                    }
                    catch (Exception error) {
                        FailEndpointObservation(
                            "PHYSICS_COMPLETE_OBSERVATION_FAILED",
                            _stateId,
                            -1,
                            "terminal failure capture queue failed: " + error.Message);
                    }
                }
            }

            if (_pendingReset != null &&
                _pendingResetComplete == null &&
                !_pendingResetCaptureQueued &&
                _activeExecution == null &&
                _cam != null &&
                _endpointCaptureOwnership != null) {
                try {
                    long captureId = _cam.QueueEndpointObservationCapture(
                        _stateId,
                        Convert.ToInt64(_stateMsg[XMProtocol.DynamicsStateIndex.SimTimeNs]),
                        _pendingReset.episode_id,
                        _pendingReset.reset_id,
                        _runtimeInstanceId,
                        0L,
                        -1);
                    _endpointCaptureOwnership.Begin(
                        new EndpointObservationCaptureIdentity {
                            runtime_instance_id = _runtimeInstanceId,
                            execution_id = 0L,
                            episode_id = _pendingReset.episode_id,
                            reset_id = _pendingReset.reset_id,
                            endpoint_state_id = _stateId,
                            source_frame_index = -1,
                            capture_id = captureId,
                        });
                    _pendingResetStateId = _stateId;
                    _pendingResetCaptureId = captureId;
                    _pendingResetCaptureQueued = true;
                }
                catch (Exception error) {
                    Debug.LogError(
                        "[XMflight] reliable reset endpoint capture queue failed: " +
                        error.Message);
                    _pendingReset = null;
                }
            }

            bool serializationAttempted = false;
            bool serializationSuccess = false;
            bool statePublishAttempted = false;
            int trySendReturn = -1;
            ulong statePublishSequence = ++_statePublishSequence;
            try {
                serializationAttempted = true;
                byte[] raw = MessagePackSerializer.Serialize(_stateMsg);
                serializationSuccess = true;
                if (_enableV4PrimitiveResultLifecycle &&
                    _endpointObservationCaptureQueued &&
                    (_appliedExecutionFrameIndex == 24 ||
                        (_pendingV4TerminalFailure &&
                            _pendingV4TerminalFailureStateId == _stateId))) {
                    // Preserve the exact frame-24 state bytes until the explicitly
                    // bound depth capture arrives.  No later state is eligible.
                    _endpointStateBytes[_stateId] = (byte[])raw.Clone();
                }
                if (_pendingResetCaptureQueued && _pendingResetStateId == _stateId)
                    _pendingResetStateBytes = (byte[])raw.Clone();
                statePublishAttempted = true;
                bool trySendResult = _statePub.TrySendFrame(raw);
                trySendReturn = trySendResult ? 1 : 0;
                if (trySendResult)
                    _stateFramesSent++;
            }
            catch (Exception e) {
                Debug.LogWarning($"[XMflight] Dynamics state serialize/send failed: {e.Message}");
            }
            finally {
                Debug.Log(
                    $"[DEBUG-ENDPOINT-OBS-4E13] STATE_PUBLISH " +
                    $"sequence={statePublishSequence} state_id={_stateId} " +
                    $"frame_index={_appliedExecutionFrameIndex} " +
                    $"execution_id={_appliedExecutionId} " +
                    $"physics_time_ns={_stateMsg[XMProtocol.DynamicsStateIndex.SimTimeNs]} " +
                    $"episode_id={_endpointEpisodeId} reset_id={_endpointResetId} " +
                    $"runtime_id={_runtimeInstanceId} try_send_return={trySendReturn}"
                );
                RecordExecutionTransportAudit(
                    stateId: _stateId,
                    simTimeNs: (long)_stateMsg[XMProtocol.DynamicsStateIndex.SimTimeNs],
                    serializationAttempted: serializationAttempted,
                    serializationSuccess: serializationSuccess,
                    statePublishAttempted: statePublishAttempted,
                    trySendReturn: trySendReturn
                );
            }
        }

        private void RecordExecutionTransportAudit(
            long stateId,
            long simTimeNs,
            bool serializationAttempted,
            bool serializationSuccess,
            bool statePublishAttempted,
            int trySendReturn
        ) {
            if (string.IsNullOrEmpty(_executionTransportAuditPath) ||
                _appliedExecutionId < 0 ||
                _appliedExecutionFrameIndex < 0 ||
                _executionTransportAudits == null) return;
            if (_executionTransportAudits.Count >= ExecutionTransportAuditCapacity) {
                _executionTransportAuditOverflow = true;
                return;
            }
            int frameCount = _activeExecution != null && _activeExecution.frames != null
                ? _activeExecution.frames.Length
                : 0;
            _executionTransportAudits.Add(new ExecutionTransportAuditRecord {
                executionId = _appliedExecutionId,
                frameIndex = _appliedExecutionFrameIndex,
                frameCount = frameCount,
                commandId = _appliedCommandId,
                stateId = stateId,
                simTimeNs = simTimeNs,
                executionStatus = _executionStatus,
                frameApplied = true,
                serializationAttempted = serializationAttempted,
                serializationSuccess = serializationSuccess,
                statePublishAttempted = statePublishAttempted,
                trySendReturn = trySendReturn,
                configuredSendHwm = _sendHwm,
                configuredRecvHwm = _recvHwm
            });
        }

        private static void AppendAuditJsonString(System.Text.StringBuilder payload, string value) {
            payload.Append('"');
            if (!string.IsNullOrEmpty(value)) {
                for (int index = 0; index < value.Length; index++) {
                    char c = value[index];
                    if (c == '"' || c == '\\') payload.Append('\\');
                    if (c == '\n') payload.Append("\\n");
                    else if (c == '\r') payload.Append("\\r");
                    else if (c == '\t') payload.Append("\\t");
                    else payload.Append(c);
                }
            }
            payload.Append('"');
        }

        private void FlushExecutionTransportAudit() {
            if (_executionTransportAuditWritten ||
                string.IsNullOrEmpty(_executionTransportAuditPath)) return;
            _executionTransportAuditWritten = true;
            try {
                string parent = System.IO.Path.GetDirectoryName(_executionTransportAuditPath);
                if (!string.IsNullOrEmpty(parent))
                    System.IO.Directory.CreateDirectory(parent);
                System.Text.StringBuilder payload = new System.Text.StringBuilder();
                payload.Append("{\"contract_id\":");
                AppendAuditJsonString(payload, ExecutionTransportAuditContractId);
                payload.Append(",\"audit_schema_version\":2");
                payload.Append(",\"episode_id\":");
                if (_executionTransportAuditEpisodeId >= 0)
                    payload.Append(_executionTransportAuditEpisodeId);
                else
                    payload.Append("null");
                payload.Append(",\"audit_run_id\":");
                AppendAuditJsonString(payload, _executionTransportAuditRunId);
                payload.Append(",\"unity_version\":");
                AppendAuditJsonString(payload, Application.unityVersion);
                payload.Append(",\"runtime_identity\":");
                AppendAuditJsonString(payload, _executionTransportAuditRuntimeIdentity);
                payload.Append(",\"physical_execution_total\":")
                    .Append(_v4RuntimeIntegration != null
                        ? _v4RuntimeIntegration.PhysicalExecutionCount
                        : 0);
                payload.Append(",\"capture_overflow\":")
                    .Append(_executionTransportAuditOverflow ? "true" : "false");
                payload.Append(",\"configured_send_hwm\":").Append(_sendHwm);
                payload.Append(",\"configured_recv_hwm\":").Append(_recvHwm);
                payload.Append(",\"records\":[");
                int auditCount = _executionTransportAudits != null
                    ? _executionTransportAudits.Count
                    : 0;
                for (int index = 0; index < auditCount; index++) {
                    if (index > 0) payload.Append(',');
                    ExecutionTransportAuditRecord record = _executionTransportAudits[index];
                    payload.Append("{\"execution_id\":").Append(record.executionId);
                    payload.Append(",\"frame_index\":").Append(record.frameIndex);
                    payload.Append(",\"frame_count\":").Append(record.frameCount);
                    payload.Append(",\"command_id\":").Append(record.commandId);
                    payload.Append(",\"state_id\":").Append(record.stateId);
                    payload.Append(",\"sim_time_ns\":").Append(record.simTimeNs);
                    payload.Append(",\"execution_status\":").Append(record.executionStatus);
                    payload.Append(",\"frame_applied\":")
                        .Append(record.frameApplied ? "true" : "false");
                    payload.Append(",\"serialization_attempted\":")
                        .Append(record.serializationAttempted ? "true" : "false");
                    payload.Append(",\"serialization_success\":")
                        .Append(record.serializationSuccess ? "true" : "false");
                    payload.Append(",\"state_publish_attempted\":")
                        .Append(record.statePublishAttempted ? "true" : "false");
                    payload.Append(",\"try_send_return\":");
                    if (record.trySendReturn < 0)
                        payload.Append("null");
                    else
                        payload.Append(record.trySendReturn == 1 ? "true" : "false");
                    payload.Append(",\"configured_send_hwm\":")
                        .Append(record.configuredSendHwm);
                    payload.Append(",\"configured_recv_hwm\":")
                        .Append(record.configuredRecvHwm);
                    payload.Append('}');
                }
                payload.Append("]}\n");
                System.IO.File.WriteAllText(_executionTransportAuditPath, payload.ToString());
            }
            catch (Exception e) {
                Debug.LogWarning(
                    $"[DEBUG-EXEC-TRANSPORT-81660] audit write failed: {e.Message}"
                );
            }
        }

        private static bool IsEndpointCaptureFailureEvent(string eventName) {
            return eventName == "GPU_CALLBACK_ERROR" ||
                eventName == "GPU_PENDING_MISS" ||
                eventName == "ENDPOINT_CAPTURE_PURGED" ||
                eventName == "CAPTURE_TIMEOUT" ||
                eventName == "CAMERA_UNAVAILABLE";
        }

        private static EndpointObservationCaptureIdentity CaptureIdentity(
            XMImageSynthesis.CaptureSnapshot snapshot) {
            return new EndpointObservationCaptureIdentity {
                runtime_instance_id = snapshot.endpointRuntimeInstanceId,
                execution_id = snapshot.endpointSourceExecutionId,
                episode_id = snapshot.endpointEpisodeId,
                reset_id = snapshot.endpointResetId,
                endpoint_state_id = snapshot.endpointStateId,
                source_frame_index = snapshot.endpointSourceFrameIndex,
                capture_id = snapshot.captureId,
            };
        }

        private void FailEndpointObservation(
            string reason,
            long stateId,
            int frameIndex,
            string detail) {
            if (_endpointCaptureOwnership != null)
                _endpointCaptureOwnership.Invalidate();
            _endpointObservationCaptureQueued = false;
            _endpointObservationCaptureDeadlineRealtime = -1f;
            _pendingV4TerminalFailure = false;
            _pendingV4TerminalFailureExecutionId = -1L;
            _pendingV4TerminalFailureReason = "";
            _pendingV4TerminalFailureStateId = -1L;
            if (stateId >= 0)
                _endpointStateBytes.Remove(stateId);
            LogP0UnityLifecycle(
                "ENDPOINT_OBSERVATION_FAILED",
                _activeExecution != null ? _activeExecution.executionId : -1L,
                -1L,
                stateId,
                frameIndex,
                "reason=" + reason + " detail=" + detail);
            if (_v4RuntimeIntegration != null && _v4RuntimeIntegration.IsActive) {
                try {
                    _v4RuntimeIntegration.FailActiveExecution(
                        "PHYSICS_COMPLETE_OBSERVATION_FAILED", RuntimeNowMs());
                }
                catch (Exception error) {
                    Debug.LogError(
                        "[XMflight] failed to record endpoint observation failure: " +
                        error.Message);
                }
            }
        }

        private bool TryFinalizeEndpointObservation(
            XMImageSynthesis.CaptureSnapshot snap) {
            if (_v4RuntimeIntegration == null || !_v4RuntimeIntegration.IsActive)
                return false;
            if (_endpointCaptureOwnership == null ||
                _endpointCaptureOwnership.TryAccept(CaptureIdentity(snap)) !=
                    EndpointObservationCaptureAcceptOutcome.ACCEPTED) {
                LogP0UnityLifecycle(
                    "ENDPOINT_CALLBACK_REJECTED",
                    snap.endpointSourceExecutionId,
                    snap.captureId,
                    snap.endpointStateId,
                    snap.endpointSourceFrameIndex,
                    "callback identity is stale or mismatched");
                return false;
            }

            if (snap.depth == null || snap.depth.Length == 0) {
                FailEndpointObservation(
                    "PHYSICS_COMPLETE_OBSERVATION_FAILED",
                    snap.endpointStateId,
                    snap.endpointSourceFrameIndex,
                    "depth bytes are missing");
                return false;
            }

            byte[] endpointStateBytes;
            if (_endpointObservationCache == null ||
                !_endpointStateBytes.TryGetValue(
                    snap.endpointStateId, out endpointStateBytes) ||
                endpointStateBytes == null || endpointStateBytes.Length == 0) {
                FailEndpointObservation(
                    "PHYSICS_COMPLETE_OBSERVATION_FAILED",
                    snap.endpointStateId,
                    snap.endpointSourceFrameIndex,
                    "frame-24 state bytes are missing");
                return false;
            }

            try {
                EndpointObservationCacheStoreResult storeResult =
                    _endpointObservationCache.StoreTerminalFrame24(
                        snap.endpointRuntimeInstanceId,
                        snap.endpointEpisodeId,
                        snap.endpointResetId,
                        snap.endpointStateId,
                        "depth-" + snap.captureId,
                        (ulong)Math.Max(0L, snap.endpointPhysicsTimeNs),
                        endpointStateBytes,
                        snap.depth);
                ObservationRefV4 observationRef = new ObservationRefV4 {
                    schema_version = 4,
                    runtime_instance_id = snap.endpointRuntimeInstanceId,
                    episode_id = snap.endpointEpisodeId,
                    reset_id = snap.endpointResetId,
                    state_id = snap.endpointStateId,
                    depth_id = "depth-" + snap.captureId,
                    sim_time_ns = (ulong)Math.Max(0L, snap.endpointPhysicsTimeNs),
                };
                EndpointObservationSnapshotV4 authoritativeSnapshot;
                if (!_endpointObservationCache.TryGet(
                        observationRef, out authoritativeSnapshot)) {
                    FailEndpointObservation(
                        "PHYSICS_COMPLETE_OBSERVATION_FAILED",
                        snap.endpointStateId,
                        snap.endpointSourceFrameIndex,
                        "stored endpoint snapshot was not immediately retrievable");
                    return false;
                }
                LogP0UnityLifecycle(
                    "SNAPSHOT_STORED",
                    snap.endpointSourceExecutionId,
                    snap.captureId,
                    snap.endpointStateId,
                    snap.endpointSourceFrameIndex,
                    "outcome=" + storeResult.outcome + " hash=authoritative");
                _endpointStateBytes.Remove(snap.endpointStateId);
                if (!_v4RuntimeIntegration.RecordEndpointObservation(
                        "depth-" + snap.captureId,
                        (ulong)Math.Max(0L, snap.captureTimeNs),
                        snap.endpointEpisodeId,
                        snap.endpointResetId,
                        snap.endpointRuntimeInstanceId,
                        RuntimeNowMs())) {
                    FailEndpointObservation(
                        "PHYSICS_COMPLETE_OBSERVATION_FAILED",
                        snap.endpointStateId,
                        snap.endpointSourceFrameIndex,
                        "runtime rejected authoritative endpoint snapshot");
                    return false;
                }
                _endpointObservationCaptureQueued = false;
                _endpointObservationCaptureDeadlineRealtime = -1f;
                return true;
            }
            catch (Exception error) {
                FailEndpointObservation(
                    "PHYSICS_COMPLETE_OBSERVATION_FAILED",
                    snap.endpointStateId,
                    snap.endpointSourceFrameIndex,
                    "endpoint snapshot store failed: " + error.Message);
                return false;
            }
        }

        private bool TryFinalizeTerminalFailureObservation(
            XMImageSynthesis.CaptureSnapshot snap) {
            if (_v4RuntimeIntegration == null || !_v4RuntimeIntegration.IsActive ||
                !_pendingV4TerminalFailure)
                return false;
            if (_endpointCaptureOwnership == null ||
                _endpointCaptureOwnership.TryAccept(CaptureIdentity(snap)) !=
                    EndpointObservationCaptureAcceptOutcome.ACCEPTED) {
                LogP0UnityLifecycle(
                    "ENDPOINT_CALLBACK_REJECTED",
                    snap.endpointSourceExecutionId,
                    snap.captureId,
                    snap.endpointStateId,
                    snap.endpointSourceFrameIndex,
                    "terminal failure callback identity is stale or mismatched");
                return false;
            }
            if (snap.depth == null || snap.depth.Length == 0) {
                FailEndpointObservation(
                    "PHYSICS_COMPLETE_OBSERVATION_FAILED",
                    snap.endpointStateId,
                    snap.endpointSourceFrameIndex,
                    "terminal failure depth bytes are missing");
                return false;
            }

            byte[] endpointStateBytes;
            if (_endpointObservationCache == null ||
                !_endpointStateBytes.TryGetValue(snap.endpointStateId, out endpointStateBytes) ||
                endpointStateBytes == null || endpointStateBytes.Length == 0) {
                FailEndpointObservation(
                    "PHYSICS_COMPLETE_OBSERVATION_FAILED",
                    snap.endpointStateId,
                    snap.endpointSourceFrameIndex,
                    "terminal failure state bytes are missing");
                return false;
            }

            try {
                EndpointObservationCacheStoreResult storeResult =
                    _endpointObservationCache.StoreTerminalFrame24(
                        snap.endpointRuntimeInstanceId,
                        snap.endpointEpisodeId,
                        snap.endpointResetId,
                        snap.endpointStateId,
                        "depth-" + snap.captureId,
                        (ulong)Math.Max(0L, snap.endpointPhysicsTimeNs),
                        endpointStateBytes,
                        snap.depth);
                ObservationRefV4 observationRef = new ObservationRefV4 {
                    schema_version = 4,
                    runtime_instance_id = snap.endpointRuntimeInstanceId,
                    episode_id = snap.endpointEpisodeId,
                    reset_id = snap.endpointResetId,
                    state_id = snap.endpointStateId,
                    depth_id = "depth-" + snap.captureId,
                    sim_time_ns = (ulong)Math.Max(0L, snap.endpointPhysicsTimeNs),
                };
                EndpointObservationSnapshotV4 stored;
                if (!_endpointObservationCache.TryGet(observationRef, out stored))
                    throw new InvalidOperationException(
                        "stored terminal failure snapshot was not immediately retrievable");
                LogP0UnityLifecycle(
                    "SNAPSHOT_STORED",
                    snap.endpointSourceExecutionId,
                    snap.captureId,
                    snap.endpointStateId,
                    snap.endpointSourceFrameIndex,
                    "terminal_failure outcome=" + storeResult.outcome + " hash=authoritative");
                _endpointStateBytes.Remove(snap.endpointStateId);
                if (!_v4RuntimeIntegration.RecordTerminalFailureObservation(
                        _pendingV4TerminalFailureReason,
                        snap.endpointStateId,
                        (ulong)Math.Max(0L, snap.endpointPhysicsTimeNs),
                        "depth-" + snap.captureId,
                        (ulong)Math.Max(0L, snap.captureTimeNs),
                        snap.endpointEpisodeId,
                        snap.endpointResetId,
                        snap.endpointRuntimeInstanceId,
                        RuntimeNowMs())) {
                    FailEndpointObservation(
                        "PHYSICS_COMPLETE_OBSERVATION_FAILED",
                        snap.endpointStateId,
                        snap.endpointSourceFrameIndex,
                        "runtime rejected terminal failure observation");
                    return false;
                }
                _pendingV4TerminalFailure = false;
                _pendingV4TerminalFailureExecutionId = -1L;
                _pendingV4TerminalFailureReason = "";
                _pendingV4TerminalFailureStateId = -1L;
                _endpointObservationCaptureQueued = false;
                _endpointObservationCaptureDeadlineRealtime = -1f;
                return true;
            }
            catch (Exception error) {
                FailEndpointObservation(
                    "PHYSICS_COMPLETE_OBSERVATION_FAILED",
                    snap.endpointStateId,
                    snap.endpointSourceFrameIndex,
                    "terminal failure snapshot store failed: " + error.Message);
                return false;
            }
        }

        private bool TryFinalizeReliableResetObservation(
            XMImageSynthesis.CaptureSnapshot snap) {
            if (_pendingReset == null || !_pendingResetCaptureQueued ||
                _endpointCaptureOwnership == null)
                return false;
            EndpointObservationCaptureIdentity identity = CaptureIdentity(snap);
            if (identity.runtime_instance_id != _runtimeInstanceId ||
                identity.execution_id != 0L || identity.source_frame_index != -1 ||
                identity.episode_id != _pendingReset.episode_id ||
                identity.reset_id != _pendingReset.reset_id ||
                identity.endpoint_state_id != _pendingResetStateId ||
                identity.capture_id != _pendingResetCaptureId ||
                _endpointCaptureOwnership.TryAccept(identity) !=
                    EndpointObservationCaptureAcceptOutcome.ACCEPTED) {
                Debug.LogWarning("[XMflight] stale reliable reset capture callback rejected");
                return false;
            }
            if (snap.depth == null || snap.depth.Length == 0 ||
                _pendingResetStateBytes == null || _pendingResetStateBytes.Length == 0) {
                Debug.LogError("[XMflight] reliable reset endpoint snapshot is incomplete");
                _pendingReset = null;
                _pendingResetCaptureQueued = false;
                return false;
            }

            string depthId = "depth-" + snap.captureId;
            try {
                _endpointObservationCache.StoreTerminalFrame24(
                    _runtimeInstanceId,
                    _pendingReset.episode_id,
                    _pendingReset.reset_id,
                    snap.endpointStateId,
                    depthId,
                    (ulong)Math.Max(0L, snap.endpointPhysicsTimeNs),
                    _pendingResetStateBytes,
                    snap.depth);
                ObservationRefV4 observationRef = new ObservationRefV4 {
                    schema_version = 4,
                    runtime_instance_id = _runtimeInstanceId,
                    episode_id = _pendingReset.episode_id,
                    reset_id = _pendingReset.reset_id,
                    state_id = snap.endpointStateId,
                    depth_id = depthId,
                    sim_time_ns = (ulong)Math.Max(0L, snap.endpointPhysicsTimeNs),
                };
                EndpointObservationSnapshotV4 stored;
                if (!_endpointObservationCache.TryGet(observationRef, out stored))
                    throw new InvalidOperationException("reset snapshot was not immediately retrievable");
                _pendingResetComplete = new PrimitiveResetV4Complete {
                    runtime_instance_id = _runtimeInstanceId,
                    episode_id = _pendingReset.episode_id,
                    reset_id = _pendingReset.reset_id,
                    observation_ref = observationRef,
                };
                if (_v4CommandTransport == null ||
                    !_v4CommandTransport.SendResetComplete(_pendingResetComplete))
                    Debug.LogWarning("[XMflight] reliable reset completion first send failed");
                _pendingResetCaptureQueued = false;
                _pendingResetStateBytes = null;
                _pendingResetStateId = -1L;
                _pendingResetCaptureId = -1L;
                _endpointCaptureOwnership.Invalidate();
                return true;
            }
            catch (Exception error) {
                Debug.LogError(
                    "[XMflight] reliable reset snapshot finalization failed: " +
                    error.Message);
                _pendingReset = null;
                _pendingResetComplete = null;
                _pendingResetCaptureQueued = false;
                return false;
            }
        }

        private void OnFrameReady(XMImageSynthesis.CaptureSnapshot snap) {
            if (snap == null) {
                FailEndpointObservation(
                    "PHYSICS_COMPLETE_OBSERVATION_FAILED", -1L, 24,
                    "endpoint capture callback was null");
                return;
            }

            if (snap.hasEndpointObservationBinding &&
                _v4RuntimeIntegration != null &&
                _v4RuntimeIntegration.IsActive) {
                bool finalized = snap.endpointSourceFrameIndex == 24
                    ? TryFinalizeEndpointObservation(snap)
                    : snap.endpointSourceFrameIndex == -1
                        ? TryFinalizeTerminalFailureObservation(snap)
                        : false;
                if (!finalized) return;
            }

            if (snap.hasEndpointObservationBinding && _pendingReset != null &&
                _pendingResetCaptureQueued)
                TryFinalizeReliableResetObservation(snap);

            // Telemetry remains best-effort.  The authoritative snapshot and
            // COMPLETE result above do not depend on PUB/SUB availability.
            if (_depthPub == null || _cam == null || snap.depth == null) return;

            int flags = snap.collision ? XMProtocol.FlagCollision : 0;
            if (IsAltitudeViolation(snap.pos))
                flags |= XMProtocol.FlagAltitudeViolation;

            XMConverters.UnityToRosPosArrayNonAlloc(snap.pos, _depthPosRos);
            XMConverters.UnityToRosRotArrayNonAlloc(snap.rot, _depthRotRos);
            XMConverters.UnityToRosPosArrayNonAlloc(AddNoise(snap.vel, _obsNoiseStdDev), _depthVelRos);
            XMConverters.UnityToRosPosArrayNonAlloc(AddNoise(snap.acc, _obsNoiseStdDev), _depthAccRos);
            XMConverters.UnityToRosPosArrayNonAlloc(snap.forward, _depthForwardRos);
            CopyFrontClearances(snap.frontClearances, snap.minClearance, _depthFrontClearances);

            _depthCameraFields[0] = _cam.fx;
            _depthCameraFields[1] = _cam.fy;
            _depthCameraFields[2] = _cam.cx;
            _depthCameraFields[3] = _cam.cy;
            _depthCameraFields[4] = _cam.OutputWidth;
            _depthCameraFields[5] = _cam.OutputHeight;
            _depthCameraFields[6] = _cam.MinDepthRange;
            _depthCameraFields[7] = _cam.MaxDepthRange;

            _depthMetaFields[0] = _cam.OutputWidth;
            _depthMetaFields[1] = _cam.OutputHeight;
            _depthMetaFields[2] = XMProtocol.DepthEncoding16UC1;
            _depthMetaFields[3] = _cam.OutputWidth * XMConstants.DepthBytesPerPixel16UC1;
            _depthMetaFields[4] = XMProtocol.ByteOrderLittleEndian;

            _depthMetaMsg[XMProtocol.DepthFrameIndex.SchemaVersion] = XMProtocol.SchemaVersion;
            _depthMetaMsg[XMProtocol.DepthFrameIndex.CaptureId] = snap.captureId;
            _depthMetaMsg[XMProtocol.DepthFrameIndex.SimTimeNs] = snap.simTimeNs;
            _depthMetaMsg[XMProtocol.DepthFrameIndex.Flags] = flags;
            _depthMetaMsg[XMProtocol.DepthFrameIndex.CapturePos] = _depthPosRos;
            _depthMetaMsg[XMProtocol.DepthFrameIndex.CaptureRot] = _depthRotRos;
            _depthMetaMsg[XMProtocol.DepthFrameIndex.CaptureVel] = _depthVelRos;
            _depthMetaMsg[XMProtocol.DepthFrameIndex.CaptureAcc] = _depthAccRos;
            _depthMetaMsg[XMProtocol.DepthFrameIndex.CaptureForward] = _depthForwardRos;
            _depthMetaMsg[XMProtocol.DepthFrameIndex.Camera] = _depthCameraFields;
            _depthMetaMsg[XMProtocol.DepthFrameIndex.DepthMeta] = _depthMetaFields;
            _depthMetaMsg[XMProtocol.DepthFrameIndex.MinClearance] = snap.minClearance;
            _depthMetaMsg[XMProtocol.DepthFrameIndex.FrontClearances] = _depthFrontClearances;
            _depthMetaMsg[XMProtocol.DepthFrameIndex.EndpointStateId] =
                snap.hasEndpointObservationBinding ? snap.endpointStateId : -1L;
            _depthMetaMsg[XMProtocol.DepthFrameIndex.EndpointPhysicsTimeNs] =
                snap.hasEndpointObservationBinding ? snap.endpointPhysicsTimeNs : -1L;
            _depthMetaMsg[XMProtocol.DepthFrameIndex.CaptureTimeNs] = snap.captureTimeNs;
            _depthMetaMsg[XMProtocol.DepthFrameIndex.EpisodeId] =
                snap.hasEndpointObservationBinding ? snap.endpointEpisodeId : _endpointEpisodeId;
            _depthMetaMsg[XMProtocol.DepthFrameIndex.ResetId] =
                snap.hasEndpointObservationBinding ? snap.endpointResetId : _endpointResetId;
            _depthMetaMsg[XMProtocol.DepthFrameIndex.RuntimeInstanceId] =
                snap.hasEndpointObservationBinding
                    ? snap.endpointRuntimeInstanceId
                    : _runtimeInstanceId;

            int depthTrySendReturn = -1;
            ulong depthPublishSequence = ++_depthPublishSequence;
            try {
                byte[] metaBytes = MessagePackSerializer.Serialize(_depthMetaMsg);
                _depthMsg.Clear();
                _depthMsg.Append(metaBytes);
                _depthMsg.Append(snap.depth);
                bool depthTrySend = _depthPub.TrySendMultipartMessage(_depthMsg);
                depthTrySendReturn = depthTrySend ? 1 : 0;
                if (depthTrySend)
                    _depthFramesSent++;
            }
            catch (Exception e) {
                Debug.LogWarning($"[XMflight] Depth frame serialize/send failed: {e.Message}");
            }

            long activeExecutionIdAtPublish = _activeExecution != null
                ? _activeExecution.executionId
                : -1L;
            int activeNextFrameAtPublish = _activeExecution != null
                ? _activeExecution.nextFrameIndex
                : -1;
            Debug.Log(
                $"[DEBUG-ENDPOINT-OBS-4E13] DEPTH_PUBLISH " +
                $"sequence={depthPublishSequence} depth_id=depth-{snap.captureId} " +
                $"frame_index={snap.endpointSourceFrameIndex} " +
                $"physics_state_id={(snap.hasEndpointObservationBinding ? snap.endpointStateId : -1L)} " +
                $"physics_time_ns={(snap.hasEndpointObservationBinding ? snap.endpointPhysicsTimeNs : -1L)} " +
                $"capture_time_ns={snap.captureTimeNs} " +
                $"episode_id={(snap.hasEndpointObservationBinding ? snap.endpointEpisodeId : _endpointEpisodeId)} " +
                $"reset_id={(snap.hasEndpointObservationBinding ? snap.endpointResetId : _endpointResetId)} " +
                $"runtime_id={(snap.hasEndpointObservationBinding ? snap.endpointRuntimeInstanceId : _runtimeInstanceId)} " +
                $"source_execution_id={(snap.hasEndpointObservationBinding ? snap.endpointSourceExecutionId : -1L)} " +
                $"active_execution_id_at_publish={activeExecutionIdAtPublish} " +
                $"active_next_frame_at_publish={activeNextFrameAtPublish} " +
                $"applied_execution_id_at_publish={_appliedExecutionId} " +
                $"applied_frame_index_at_publish={_appliedExecutionFrameIndex} " +
                $"has_endpoint_binding={snap.hasEndpointObservationBinding} " +
                $"try_send_return={depthTrySendReturn}"
            );

        }

        private static void CopyFrontClearances(float[] src, float fallback, float[] dst) {
            if (src != null && src.Length >= 3) {
                dst[0] = src[0];
                dst[1] = src[1];
                dst[2] = src[2];
                return;
            }

            dst[0] = fallback;
            dst[1] = fallback;
            dst[2] = fallback;
        }

        private void PublishDiagnostics() {
            if (!_enableDiagnostics) return;
            _diagTimer += Time.unscaledDeltaTime;

            if (_diagTimer < 10f)
                return;

            _diagTimer = 0f;

            string frontStr = _col != null && _col.FrontClearances != null
                ? $"front=[{_col.FrontClearances[0]:F2},{_col.FrontClearances[1]:F2},{_col.FrontClearances[2]:F2}]"
                : "front=[-1.00,-1.00,-1.00]";

            Debug.Log(
                $"[XMflight] state_sent={_stateFramesSent}, depth_sent={_depthFramesSent}, cmd_rx={_cmdReceived}, " +
                $"mode_vel={_modeVelocityCount}, mode_step={_modeStepCount}, mode_traj={_modeTrajectoryCount}, mode_tp={_modeTeleportCount}, " +
                $"cmd_decode_errors={_cmdDecodeErrors}, vel={_realVelocity.magnitude:F2}m/s, " +
                $"fixed_delta_avg={_avgFixedDelta:F3}m, fixed_delta_max={_maxFixedDelta:F3}m, " +
                $"clearance={(_col != null ? _col.MinClearance : -1f):F2}m, {frontStr}"
            );

            _maxFixedDelta = 0f;
            _avgFixedDelta = 0f;
            _fixedDeltaCount = 0;
        }

        private bool IsAltitudeViolation(Vector3 unityWorldPosition) {
            if (!_enableAltitudeViolationFlag) return false;

            float lower = _minFlightHeight - Mathf.Max(0f, _altitudeViolationMargin);
            float upper = _maxFlightHeight + Mathf.Max(0f, _altitudeViolationMargin);
            return unityWorldPosition.y < lower || unityWorldPosition.y > upper;
        }

        private static bool IsVectorValid(Vector3 v) {
            return !float.IsNaN(v.x) &&
                   !float.IsNaN(v.y) &&
                   !float.IsNaN(v.z) &&
                   !float.IsInfinity(v.x) &&
                   !float.IsInfinity(v.y) &&
                   !float.IsInfinity(v.z);
        }

        private static bool IsFloatValid(float f) {
            return !float.IsNaN(f) &&
                   !float.IsInfinity(f);
        }

        private static bool IsQuaternionValid(Quaternion q) {
            return !float.IsNaN(q.x) &&
                   !float.IsNaN(q.y) &&
                   !float.IsNaN(q.z) &&
                   !float.IsNaN(q.w) &&
                   !float.IsInfinity(q.x) &&
                   !float.IsInfinity(q.y) &&
                   !float.IsInfinity(q.z) &&
                   !float.IsInfinity(q.w);
        }

        private void OnDisable() {
            if (_enableV4PrimitiveResultLifecycle &&
                _v4RuntimeIntegration != null &&
                _v4RuntimeIntegration.IsActive)
                DisableDeterministicExecution("DISABLE");
            else if (_endpointCaptureOwnership != null)
                _endpointCaptureOwnership.Invalidate();
        }

        private void OnDestroy() {
            if (_enableV4PrimitiveResultLifecycle &&
                _v4RuntimeIntegration != null &&
                _v4RuntimeIntegration.IsActive)
                DisableDeterministicExecution("DESTROY");
            else if (_endpointCaptureOwnership != null)
                _endpointCaptureOwnership.Invalidate();
            _running = false;
            FlushExecutionTransportAudit();

            if (_cam != null) {
                _cam.OnCaptureLifecycleDiagnostic -= OnCaptureLifecycleDiagnostic;
                _cam.OnFrameReady -= OnFrameReady;
            }

            _sub?.Close();
            _sub?.Dispose();
            _sub = null;

            _statePub?.Close();
            _statePub?.Dispose();
            _statePub = null;

            _depthPub?.Close();
            _depthPub?.Dispose();
            _depthPub = null;

            _v4ResultTransport?.Close();
            _v4ResultTransport = null;
            _v4CommandTransport?.Close();
            _v4CommandTransport = null;
            _v4SnapshotTransport?.Close();
            _v4SnapshotTransport = null;

            NetMQConfig.Cleanup();
        }
    }
}
