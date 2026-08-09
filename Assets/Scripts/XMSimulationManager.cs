// filename: Assets/Scripts/XMSimulationManager.cs

using UnityEngine;
using NetMQ;
using NetMQ.Sockets;
using MessagePack;
using System;
using System.Collections;

namespace XMflight
{
    public sealed class XMSimulationManager : MonoBehaviour {
        [Header("Optional Unified Config")]
        [SerializeField] private XMConfig _config;

        [Header("ZMQ")]
        [SerializeField] private int _commandSubPort = 10253;
        [SerializeField] private int _statePubPort = 10254;
        [SerializeField] private int _depthPubPort = 11254;
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
        private bool _completeExecutionAfterPublish;
        private BufferedPrimitiveExecution _activeExecution;
        private long _appliedExecutionId = -1;
        private int _appliedExecutionFrameIndex = -1;
        private long _appliedCommandId = -1;
        private int _executionStatus = XMProtocol.ExecutionStatusNone;

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

        private bool _running;

        private long _stateFramesSent;
        private long _depthFramesSent;
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

                _cam.OnFrameReady += OnFrameReady;
            }

            InitZMQ();
            _running = true;

            Debug.Log(
                $"[XMflight] Started. cmd_sub=tcp://*:{_commandSubPort}, " +
                $"state_pub=tcp://*:{_statePubPort}, depth_pub=tcp://*:{_depthPubPort}, " +
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
            }
        }

        private void Update() {
            if (!_running) return;

            PublishDiagnostics();
        }

        private void FixedUpdate() {
            if (!_running) return;

            ClearExecutionAcknowledgement();
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

            if (_activeExecution != null)
                ApplyNextPrimitiveExecutionFrame();

            if (!_deterministicExecutionMode)
                ApplyCommandTimeoutBrake();

            bool integrationDue = !_deterministicExecutionMode || _activeExecution != null;
            if (!_frozen && integrationDue)
                StepDynamics();

            PublishDynamicsState();

            if (_completeExecutionAfterPublish) {
                _activeExecution = null;
                _completeExecutionAfterPublish = false;
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
        }

        private void PollCommands() {
            while (_sub != null && _sub.TryReceiveFrameBytes(out byte[] raw)) {
                ControlCommandMsg cmd;

                try {
                    object[] msg = MessagePackSerializer.Deserialize<object[]>(raw);

                    if (msg == null || msg.Length < XMProtocol.CommandFieldCount) {
                        throw new FormatException(
                            $"Command field count invalid: {(msg == null ? 0 : msg.Length)}"
                        );
                    }

                    int schema = AsInt(msg[XMProtocol.CommandIndex.SchemaVersion]);
                    if (schema != XMProtocol.SchemaVersion) {
                        throw new FormatException(
                            $"Command schema mismatch: {schema} != {XMProtocol.SchemaVersion}"
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
                    Debug.LogWarning($"[XMflight] Command decode failed: {e.Message}");
                    continue;
                }

                _cmdReceived++;

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

        private void ClearExecutionAcknowledgement() {
            _appliedExecutionId = -1;
            _appliedExecutionFrameIndex = -1;
            _appliedCommandId = -1;
            _executionStatus = XMProtocol.ExecutionStatusNone;
        }

        private void DisableDeterministicExecution() {
            _deterministicExecutionMode = false;
            _activeExecution = null;
            _completeExecutionAfterPublish = false;
            ClearExecutionAcknowledgement();
        }

        private void HandlePrimitiveExecution(ControlCommandMsg cmd) {
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
            _completeExecutionAfterPublish = complete;
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

            DisableDeterministicExecution();

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
            if (_publishStride > 1 && (_publishCounter % _publishStride) != 0) return;
            if (_statePub == null || _drone == null) return;

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

            try {
                byte[] raw = MessagePackSerializer.Serialize(_stateMsg);
                if (_statePub.TrySendFrame(raw))
                    _stateFramesSent++;
            }
            catch (Exception e) {
                Debug.LogWarning($"[XMflight] Dynamics state serialize/send failed: {e.Message}");
            }
        }

        private void OnFrameReady(XMImageSynthesis.CaptureSnapshot snap) {
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

            try {
                byte[] metaBytes = MessagePackSerializer.Serialize(_depthMetaMsg);
                _depthMsg.Clear();
                _depthMsg.Append(metaBytes);
                _depthMsg.Append(snap.depth);
                if (_depthPub.TrySendMultipartMessage(_depthMsg))
                    _depthFramesSent++;
            }
            catch (Exception e) {
                Debug.LogWarning($"[XMflight] Depth frame serialize/send failed: {e.Message}");
            }
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

        private void OnDestroy() {
            _running = false;

            if (_cam != null)
                _cam.OnFrameReady -= OnFrameReady;

            _sub?.Close();
            _sub?.Dispose();
            _sub = null;

            _statePub?.Close();
            _statePub?.Dispose();
            _statePub = null;

            _depthPub?.Close();
            _depthPub?.Dispose();
            _depthPub = null;

            NetMQConfig.Cleanup();
        }
    }
}
