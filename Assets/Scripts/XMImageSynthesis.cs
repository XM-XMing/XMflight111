// filename: Assets/Scripts/XMImageSynthesis.cs
using UnityEngine;
using UnityEngine.Rendering;
using System;
using System.Collections.Generic;

namespace XMflight
{
    [RequireComponent(typeof(Camera))]
    public sealed class XMImageSynthesis : MonoBehaviour {
        [Header("D435i Simulation")]
        [SerializeField] private float _targetHFov = 87.0f;
        [SerializeField] private float _targetVFov = 58.0f;
        [SerializeField] private float _maxDepthRange = 6.0f;
        [SerializeField] private float _minDepthRange = 0.3f;
        [SerializeField] private int _outputWidth = 848;
        [SerializeField] private int _outputHeight = 480;

        [Header("Physical Mount Relative to Drone Center")]
        [SerializeField] private Vector3 _cameraOffset = Vector3.zero;
        [SerializeField] private Vector3 _cameraEuler = Vector3.zero;

        [Header("Resources")]
        [SerializeField] private Shader _uberReplacementShader;
        [SerializeField] private Shader _depthVisualizeShader;
        [SerializeField] private bool _showDepthInGameView = true;

        [Header("Depth Noise")]
        [SerializeField] private bool _enableDepthNoise = false;
        [SerializeField] private float _baseDepthNoiseStd = 0.002f;
        [SerializeField] private float _depthNoiseQuadraticK = 0.0015f;
        [SerializeField, Range(0f, 1f)] private float _randomDropoutProb = 0f;

        public int OutputWidth => _outputWidth;
        public int OutputHeight => _outputHeight;
        public float MaxDepthRange => _maxDepthRange;
        public float MinDepthRange => _minDepthRange;
        public float TargetHFov => _targetHFov;
        public float TargetVFov => _targetVFov;

        [HideInInspector] public float fx, fy, cx, cy;

        public sealed class CaptureSnapshot {
            public long captureId;
            public float captureTime;
            public long simTimeNs;
            public Vector3 pos;
            public Quaternion rot;
            public Vector3 vel;
            public Vector3 acc;
            public Vector3 forward;
            public bool collision;
            public float minClearance;
            public float[] frontClearances;
            public byte[] depth;
        }

        public Action<CaptureSnapshot> OnFrameReady;
        public Func<(Vector3 vel, Vector3 acc, bool col, float clear, float[] front)> OnRequestDynamics;

        private Camera _mainCamera;
        private Camera _depthCamera;
        private RenderTexture _depthRT;
        private Material _vizMat;
        private long _captureId;

        private ushort[] _depth16Buf;
        private byte[][] _depthBytesPool;
        private int _poolIdx;

        private readonly Dictionary<long, CaptureSnapshot> _pending = new Dictionary<long, CaptureSnapshot>(16);
        private readonly List<long> _staleKeys = new List<long>(16);

        private bool _alive;

        private void Awake() {
            _mainCamera = GetComponent<Camera>();
            SetupCameras(_outputWidth, _outputHeight, _targetHFov, _targetVFov);
        }

        private void OnEnable() {
            _alive = true;
            if (_depthRT == null)
                SetupCameras(_outputWidth, _outputHeight, _targetHFov, _targetVFov);
        }

        public void ApplyConfig(XMConfig cfg) {
            if (cfg == null) return;

            _targetHFov = cfg.targetHFovDeg;
            _targetVFov = cfg.targetVFovDeg;
            _minDepthRange = cfg.minDepthRange;
            _maxDepthRange = cfg.maxDepthRange;
            _outputWidth = cfg.outputWidth;
            _outputHeight = cfg.outputHeight;
            _cameraOffset = cfg.cameraOffset;
            _cameraEuler = cfg.cameraEuler;

            _showDepthInGameView = cfg.showDepthInGameView;

            _enableDepthNoise = cfg.enableDepthNoise;
            _baseDepthNoiseStd = cfg.baseDepthNoiseStd;
            _depthNoiseQuadraticK = cfg.depthNoiseQuadraticK;
            _randomDropoutProb = cfg.randomDropoutProb;

            SetupCameras(_outputWidth, _outputHeight, _targetHFov, _targetVFov);
        }

        public void SetupCameras(int w, int h, float hFov, float vFov) {
            _outputWidth = Mathf.Max(XMConstants.MinImageSize, w);
            _outputHeight = Mathf.Max(XMConstants.MinImageSize, h);
            _targetHFov = Mathf.Clamp(
                hFov,
                XMConstants.MinCameraFovDeg,
                XMConstants.MaxCameraFovDeg
            );
            _targetVFov = Mathf.Clamp(
                vFov,
                XMConstants.MinCameraFovDeg,
                XMConstants.MaxCameraFovDeg
            );

            fx = (_outputWidth * 0.5f) / Mathf.Tan(_targetHFov * 0.5f * Mathf.Deg2Rad);
            fy = (_outputHeight * 0.5f) / Mathf.Tan(_targetVFov * 0.5f * Mathf.Deg2Rad);
            cx = _outputWidth * 0.5f;
            cy = _outputHeight * 0.5f;

            if (_uberReplacementShader == null)
                _uberReplacementShader = Shader.Find(XMConstants.UberDepthShaderName);

            if (_depthVisualizeShader == null)
                _depthVisualizeShader = Shader.Find(XMConstants.DepthVisualizeShaderName);

            if (_depthVisualizeShader != null && _vizMat == null)
                _vizMat = new Material(_depthVisualizeShader);

            if (_depthCamera == null) {
                GameObject go = new GameObject("D435i_Depth_Sim");
                go.transform.SetParent(transform, false);
                _depthCamera = go.AddComponent<Camera>();
            }

            _depthCamera.transform.localPosition = _cameraOffset;
            _depthCamera.transform.localEulerAngles = _cameraEuler;

            _depthCamera.CopyFrom(_mainCamera);
            _depthCamera.enabled = false;
            _depthCamera.clearFlags = CameraClearFlags.SolidColor;
            _depthCamera.backgroundColor = Color.black;
            _depthCamera.nearClipPlane = XMConstants.DepthCameraNearClip;
            _depthCamera.farClipPlane = _maxDepthRange + XMConstants.DepthFarClipPadding;
            _depthCamera.fieldOfView = _targetVFov;
            _depthCamera.aspect = (float)_outputWidth / _outputHeight;
            _depthCamera.projectionMatrix = BuildProjectionFromIntrinsics(
                fx, fy, cx, cy, _outputWidth, _outputHeight,
                _depthCamera.nearClipPlane, _depthCamera.farClipPlane
            );
            _depthCamera.nonJitteredProjectionMatrix = _depthCamera.projectionMatrix;

            ReleaseRT();

            _depthRT = new RenderTexture(_outputWidth, _outputHeight, 24, RenderTextureFormat.RFloat) {
                filterMode = FilterMode.Point,
                wrapMode = TextureWrapMode.Clamp,
                name = "XM_D435i_Depth_RFloat"
            };
            _depthRT.Create();

            int pixelCount = _outputWidth * _outputHeight;
            _depth16Buf = new ushort[pixelCount];

            _depthBytesPool = new byte[XMConstants.DefaultDepthPoolSize][];
            for (int i = 0; i < XMConstants.DefaultDepthPoolSize; i++) {
                _depthBytesPool[i] = new byte[pixelCount * XMConstants.DepthBytesPerPixel16UC1];
            }
        }


        private static Matrix4x4 BuildProjectionFromIntrinsics(
            float fx, float fy, float cx, float cy, int width, int height, float near, float far
        ) {
            float left = -cx * near / fx;
            float right = (width - cx) * near / fx;
            float top = cy * near / fy;
            float bottom = -(height - cy) * near / fy;

            Matrix4x4 m = new Matrix4x4();
            m[0, 0] = 2f * near / (right - left);
            m[0, 2] = (right + left) / (right - left);
            m[1, 1] = 2f * near / (top - bottom);
            m[1, 2] = (top + bottom) / (top - bottom);
            m[2, 2] = -(far + near) / (far - near);
            m[2, 3] = -(2f * far * near) / (far - near);
            m[3, 2] = -1f;
            return m;
        }

        private void OnRenderImage(RenderTexture source, RenderTexture destination) {
            if (!_alive || _depthCamera == null || _uberReplacementShader == null || _depthRT == null) {
                Graphics.Blit(source, destination);
                return;
            }

            long id = ++_captureId;
            double simTime = Time.timeAsDouble;
            long simTimeNs = (long)(simTime * XMConstants.SecondsToNanoseconds);

            var dyn = OnRequestDynamics?.Invoke()
                      ?? (Vector3.zero, Vector3.zero, false, _maxDepthRange,
                          new float[3] { _maxDepthRange, _maxDepthRange, _maxDepthRange });

            _pending[id] = new CaptureSnapshot {
                captureId = id,
                captureTime = (float)simTime,
                simTimeNs = simTimeNs,
                pos = _depthCamera.transform.position,
                rot = _depthCamera.transform.rotation,
                forward = _depthCamera.transform.forward,
                vel = dyn.vel,
                acc = dyn.acc,
                collision = dyn.col,
                minClearance = dyn.clear,
                frontClearances = dyn.front != null
                    ? (float[])dyn.front.Clone()
                    : new float[3] { dyn.clear, dyn.clear, dyn.clear }
            };

            Shader.SetGlobalFloat("_MinDepth", _minDepthRange);
            Shader.SetGlobalFloat("_MaxDepth", _maxDepthRange);

            _depthCamera.targetTexture = _depthRT;
            _depthCamera.RenderWithShader(_uberReplacementShader, "RenderType");

            AsyncGPUReadback.Request(_depthRT, 0, TextureFormat.RFloat, req => {
                if (!_alive) return;

                if (req.hasError || !_pending.TryGetValue(id, out CaptureSnapshot frame))
                {
                    _pending.Remove(id);
                    return;
                }

                frame.depth = ConvertTo16UC1(req.GetData<float>());
                OnFrameReady?.Invoke(frame);

                _pending.Remove(id);
                PurgeStale(id);
            });

            if (_showDepthInGameView && _vizMat != null) {
                _vizMat.SetFloat("_MaxDepth", _maxDepthRange);
                Graphics.Blit(_depthRT, destination, _vizMat);
            }
            else {
                Graphics.Blit(source, destination);
            }
        }

        private byte[] ConvertTo16UC1(Unity.Collections.NativeArray<float> src)
        {
            int n = src.Length;

            for (int i = 0; i < n; i++) {
                float d = src[i];

                if (d < _minDepthRange || d > _maxDepthRange || d <= 0f ||
                    float.IsNaN(d) || float.IsInfinity(d)) {
                    _depth16Buf[i] = 0;
                    continue;
                }

                if (_enableDepthNoise) {
                    float sigma = _baseDepthNoiseStd + _depthNoiseQuadraticK * d * d;
                    d += GaussianNoise(sigma);

                    if (_randomDropoutProb > 0f && UnityEngine.Random.value < _randomDropoutProb) {
                        _depth16Buf[i] = 0;
                        continue;
                    }

                    if (d < _minDepthRange || d > _maxDepthRange) {
                        _depth16Buf[i] = 0;
                        continue;
                    }
                }

                _depth16Buf[i] = (ushort)Mathf.Clamp(d * XMConstants.DepthMetersToMillimeters, 0f, ushort.MaxValue);
            }

            _poolIdx = (_poolIdx + 1) % XMConstants.DefaultDepthPoolSize;
            byte[] currentBuf = _depthBytesPool[_poolIdx];
            Buffer.BlockCopy(_depth16Buf, 0, currentBuf, 0, currentBuf.Length);

            return currentBuf;
        }

        private static float GaussianNoise(float stdDev) {
            float u1 = 1f - UnityEngine.Random.value;
            float u2 = 1f - UnityEngine.Random.value;
            return stdDev * Mathf.Sqrt(-2f * Mathf.Log(u1)) * Mathf.Sin(2f * Mathf.PI * u2);
        }

        private void PurgeStale(long currentId)
        {
            if (_pending.Count == 0) return;

            _staleKeys.Clear();

            foreach (KeyValuePair<long, CaptureSnapshot> kvp in _pending) {
                if (currentId - kvp.Key >  XMConstants.DefaultDepthPoolSize)
                    _staleKeys.Add(kvp.Key);
            }

            foreach (long k in _staleKeys)
                _pending.Remove(k);
        }

        private void ReleaseRT() {
            if (_depthRT == null) return;

            if (_depthCamera != null)
                _depthCamera.targetTexture = null;

            _depthRT.Release();

            if (Application.isPlaying)
                Destroy(_depthRT);
            else
                DestroyImmediate(_depthRT);

            _depthRT = null;
        }

        private void OnDisable() {
            _alive = false;
            ReleaseRT();
            _pending.Clear();

            if (_vizMat != null) {
                if (Application.isPlaying)
                    Destroy(_vizMat);
                else
                    DestroyImmediate(_vizMat);

                _vizMat = null;
            }
        }

        private void OnDestroy() {
            _alive = false;
            ReleaseRT();
            _pending.Clear();

            if (_vizMat != null) {
                if (Application.isPlaying)
                    Destroy(_vizMat);
                else
                    DestroyImmediate(_vizMat);

                _vizMat = null;
            }
        }
    }
}
