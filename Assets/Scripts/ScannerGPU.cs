// filename: Assets/Scripts/Other/ScannerGPU.cs
using UnityEngine;
using System.Collections.Generic;
using System.IO;
using System;
#if UNITY_EDITOR
using UnityEditor;
#endif

namespace XMflight
{
    public class ScannerGPU : MonoBehaviour
    {
        public enum ScanMode
        {
            FullScan,
            HeightRange
        }

        [Header("Core settings")]
        public ComputeShader computeShader;
        public string outputFileName = "forest_allmap_point_cloud.bin";

        [Header("Scan Parameters")]
        [Min(0f)]
        public float density = 200f;

        [Min(1)]
        public int batchSize = 100;

        [Header("Scan Mode")]
        public ScanMode scanMode = ScanMode.FullScan;

        [Tooltip("最低离地高度，单位：米。仅 HeightRange 模式使用")]
        public float minHeightAboveGround = 0f;

        [Tooltip("最高离地高度，单位：米。仅 HeightRange 模式使用")]
        public float maxHeightAboveGround = 6f;

        [Tooltip("树根节点 Y 与实际地面不一致时使用。仅 HeightRange 模式使用")]
        public float groundHeightOffset = 0f;

        private const int MAX_POINTS_PER_BATCH = 20000000;
        private Transform _forestRoot;

        private sealed class MeshInstanceGroup
        {
            public readonly List<Matrix4x4> matrices = new List<Matrix4x4>();
            public readonly List<float> groundHeights = new List<float>();
        }

        [ContextMenu("Start Scan")]
        public void StartScan()
        {
            if (computeShader == null)
            {
                Debug.LogError("No ComputeShader!");
                return;
            }

            _forestRoot = GameObject.Find("Generated_Forest")?.transform;
            if (_forestRoot == null)
            {
                Debug.LogError("No Find 'Generated_Forest'");
                return;
            }

            bool useHeightFilter = scanMode == ScanMode.HeightRange;
            float minHeight = Mathf.Min(minHeightAboveGround, maxHeightAboveGround);
            float maxHeight = Mathf.Max(minHeightAboveGround, maxHeightAboveGround);

            string dataDir = Path.GetFullPath(
                Path.Combine(Application.dataPath, "/home/xm/XM/xm_ws/src/planning/data/map_data"));

            if (!Directory.Exists(dataDir))
                Directory.CreateDirectory(dataDir);

            string fullPath = Path.Combine(dataDir, outputFileName);

            Debug.Log(
                $"<color=cyan>Start Scan...</color> " +
                $"density:{density} | batch:{batchSize} | " +
                $"scanMode:{scanMode}" +
                (useHeightFilter
                    ? $" | heightRange:[{minHeight:F2}, {maxHeight:F2}]m"
                    : string.Empty));

            var sw = System.Diagnostics.Stopwatch.StartNew();

            var meshGroups = new Dictionary<Mesh, MeshInstanceGroup>();
            int treeCount = _forestRoot.childCount;

            for (int i = 0; i < treeCount; i++)
            {
                Transform tree = _forestRoot.GetChild(i);

                float groundY = tree.position.y +
                    (useHeightFilter ? groundHeightOffset : 0f);

                var renderers = tree.GetComponentsInChildren<MeshRenderer>();

                foreach (var mr in renderers)
                {
                    if (!mr.enabled)
                        continue;

                    if (mr.gameObject.CompareTag("EditorOnly"))
                        continue;

                    MeshFilter mf = mr.GetComponent<MeshFilter>();
                    if (mf == null || mf.sharedMesh == null)
                        continue;

                    if (!meshGroups.TryGetValue(mf.sharedMesh, out MeshInstanceGroup group))
                    {
                        group = new MeshInstanceGroup();
                        meshGroups.Add(mf.sharedMesh, group);
                    }

                    group.matrices.Add(mr.transform.localToWorldMatrix);
                    group.groundHeights.Add(groundY);
                }
            }

            Debug.Log(
                $"Collect over. Mesh groups:{meshGroups.Count}, " +
                $"trees:{treeCount}. Start GPU processing...");

            long totalPoints = 0;
            int totalBatches = 0;

            try
            {
                using (FileStream fs = new FileStream(
                    fullPath,
                    FileMode.Create,
                    FileAccess.Write))
                {
                    byte[] placeholder = new byte[4];
                    fs.Write(placeholder, 0, placeholder.Length);

                    foreach (var kvp in meshGroups)
                    {
                        Mesh mesh = kvp.Key;
                        MeshInstanceGroup group = kvp.Value;

                        Vector3[] verts = mesh.vertices;
                        int[] indices = mesh.triangles;

                        if (verts.Length == 0 || indices.Length == 0)
                            continue;

                        ComputeBuffer vertBuffer = null;
                        ComputeBuffer idxBuffer = null;

                        try
                        {
                            vertBuffer = new ComputeBuffer(verts.Length, 12);
                            idxBuffer = new ComputeBuffer(indices.Length, 4);

                            vertBuffer.SetData(verts);
                            idxBuffer.SetData(indices);

                            int triCount = indices.Length / 3;
                            int kernelIndex = computeShader.FindKernel("CSMain");
                            int threadGroupsX = Mathf.CeilToInt(triCount / 64f);

                            for (int i = 0; i < group.matrices.Count; i += batchSize)
                            {
                                int count = Mathf.Min(
                                    batchSize,
                                    group.matrices.Count - i);

                                List<Matrix4x4> batchMatrices =
                                    group.matrices.GetRange(i, count);

                                List<float> batchGroundHeights =
                                    group.groundHeights.GetRange(i, count);

                                int pointsWritten = ProcessBatchSafe(
                                    fs,
                                    kernelIndex,
                                    vertBuffer,
                                    idxBuffer,
                                    batchMatrices,
                                    batchGroundHeights,
                                    triCount,
                                    threadGroupsX,
                                    minHeight,
                                    maxHeight,
                                    useHeightFilter);

                                totalPoints += pointsWritten;
                                totalBatches++;

#if UNITY_EDITOR
                                if (totalBatches % 5 == 0)
                                {
                                    float progress =
                                        group.matrices.Count > 0
                                            ? (float)(i + count) / group.matrices.Count
                                            : 1f;

                                    EditorUtility.DisplayProgressBar(
                                        "Scanning...",
                                        $"Mesh: {mesh.name} | " +
                                        $"Batch: {totalBatches} | " +
                                        $"Points: {totalPoints:N0}",
                                        progress);
                                }
#endif
                            }
                        }
                        finally
                        {
                            vertBuffer?.Release();
                            idxBuffer?.Release();
                        }
                    }

                    fs.Seek(0, SeekOrigin.Begin);

                    int headerPointCount =
                        totalPoints > int.MaxValue
                            ? int.MaxValue
                            : (int)totalPoints;

                    fs.Write(
                        BitConverter.GetBytes(headerPointCount),
                        0,
                        sizeof(int));
                }
            }
            finally
            {
#if UNITY_EDITOR
                EditorUtility.ClearProgressBar();
#endif
            }

            sw.Stop();

            float mb = new FileInfo(fullPath).Length / 1024f / 1024f;

            Debug.Log(
                $"<color=green>Scan OK!</color>\n" +
                $"SumPoint: {totalPoints:N0}\n" +
                $"FileSize: {mb:F2} MB\n" +
                $"ScanMode: {scanMode}\n" +
                (useHeightFilter
                    ? $"HeightRange: [{minHeight:F2}, {maxHeight:F2}]m\n"
                    : string.Empty) +
                $"Time: {sw.Elapsed.TotalSeconds:F2}s\n" +
                $"Path: {fullPath}");
        }

        private int ProcessBatchSafe(
            FileStream fs,
            int kernel,
            ComputeBuffer vb,
            ComputeBuffer ib,
            List<Matrix4x4> mats,
            List<float> groundHeights,
            int triCount,
            int groupsX,
            float minHeight,
            float maxHeight,
            bool useHeightFilter)
        {
            if (mats == null || mats.Count == 0)
                return 0;

            if (groundHeights == null || groundHeights.Count != mats.Count)
            {
                Debug.LogError(
                    $"Ground height count mismatch: " +
                    $"matrices={mats.Count}, groundHeights={groundHeights?.Count ?? 0}");
                return 0;
            }

            ComputeBuffer matBuffer = null;
            ComputeBuffer groundHeightBuffer = null;
            ComputeBuffer resultBuffer = null;
            ComputeBuffer countBuffer = null;

            try
            {
                matBuffer = new ComputeBuffer(mats.Count, 64);
                groundHeightBuffer = new ComputeBuffer(groundHeights.Count, 4);

                matBuffer.SetData(mats);
                groundHeightBuffer.SetData(groundHeights);

                resultBuffer = new ComputeBuffer(
                    MAX_POINTS_PER_BATCH,
                    12,
                    ComputeBufferType.Append);

                resultBuffer.SetCounterValue(0);

                computeShader.SetBuffer(kernel, "_Vertices", vb);
                computeShader.SetBuffer(kernel, "_Indices", ib);
                computeShader.SetBuffer(kernel, "_Matrices", matBuffer);
                computeShader.SetBuffer(kernel, "_GroundHeights", groundHeightBuffer);
                computeShader.SetBuffer(kernel, "_ResultPoints", resultBuffer);

                computeShader.SetFloat("_Density", density);
                computeShader.SetFloat(
                    "_Seed",
                    UnityEngine.Random.value * 1000f);

                computeShader.SetInt("_IndicesCount", triCount * 3);

                computeShader.SetInt(
                    "_EnableHeightFilter",
                    useHeightFilter ? 1 : 0);

                computeShader.SetFloat(
                    "_MinHeightAboveGround",
                    minHeight);

                computeShader.SetFloat(
                    "_MaxHeightAboveGround",
                    maxHeight);

                computeShader.Dispatch(
                    kernel,
                    groupsX,
                    mats.Count,
                    1);

                countBuffer = new ComputeBuffer(
                    1,
                    4,
                    ComputeBufferType.IndirectArguments);

                ComputeBuffer.CopyCount(
                    resultBuffer,
                    countBuffer,
                    0);

                int[] countArr = new int[1];
                countBuffer.GetData(countArr);

                int validPointCount = countArr[0];
                int readCount = validPointCount;

                if (validPointCount > MAX_POINTS_PER_BATCH)
                {
                    readCount = MAX_POINTS_PER_BATCH;

                    Debug.LogWarning(
                        $"Buffer Loading Over: " +
                        $"{validPointCount} > {MAX_POINTS_PER_BATCH}. " +
                        $"Some points were truncated.");
                }

                if (readCount > 0)
                {
                    int floatCount = readCount * 3;
                    float[] gpuPointsFlat = new float[floatCount];

                    resultBuffer.GetData(
                        gpuPointsFlat,
                        0,
                        0,
                        floatCount);

                    byte[] bytes = new byte[floatCount * sizeof(float)];

                    Buffer.BlockCopy(
                        gpuPointsFlat,
                        0,
                        bytes,
                        0,
                        bytes.Length);

                    fs.Write(bytes, 0, bytes.Length);
                }

                return readCount;
            }
            finally
            {
                matBuffer?.Release();
                groundHeightBuffer?.Release();
                resultBuffer?.Release();
                countBuffer?.Release();
            }
        }

#if UNITY_EDITOR
        private void OnValidate()
        {
            density = Mathf.Max(0f, density);
            batchSize = Mathf.Max(1, batchSize);
        }
#endif
    }

#if UNITY_EDITOR
    [CustomEditor(typeof(ScannerGPU))]
    public class ScannerGPUEditor : Editor
    {
        private SerializedProperty _computeShader;
        private SerializedProperty _outputFileName;
        private SerializedProperty _density;
        private SerializedProperty _batchSize;
        private SerializedProperty _scanMode;
        private SerializedProperty _minHeightAboveGround;
        private SerializedProperty _maxHeightAboveGround;
        private SerializedProperty _groundHeightOffset;

        private void OnEnable()
        {
            _computeShader = serializedObject.FindProperty("computeShader");
            _outputFileName = serializedObject.FindProperty("outputFileName");
            _density = serializedObject.FindProperty("density");
            _batchSize = serializedObject.FindProperty("batchSize");
            _scanMode = serializedObject.FindProperty("scanMode");
            _minHeightAboveGround =
                serializedObject.FindProperty("minHeightAboveGround");
            _maxHeightAboveGround =
                serializedObject.FindProperty("maxHeightAboveGround");
            _groundHeightOffset =
                serializedObject.FindProperty("groundHeightOffset");
        }

        public override void OnInspectorGUI()
        {
            serializedObject.Update();

            EditorGUILayout.LabelField(
                "Core Settings",
                EditorStyles.boldLabel);
            EditorGUILayout.PropertyField(_computeShader);
            EditorGUILayout.PropertyField(_outputFileName);

            EditorGUILayout.Space(6);
            EditorGUILayout.LabelField(
                "Scan Parameters",
                EditorStyles.boldLabel);
            EditorGUILayout.PropertyField(_density);
            EditorGUILayout.PropertyField(_batchSize);

            EditorGUILayout.Space(6);
            EditorGUILayout.LabelField(
                "Scan Mode",
                EditorStyles.boldLabel);
            EditorGUILayout.PropertyField(
                _scanMode,
                new GUIContent("Mode"));

            ScannerGPU.ScanMode currentMode =
                (ScannerGPU.ScanMode)_scanMode.enumValueIndex;

            if (currentMode == ScannerGPU.ScanMode.HeightRange)
            {
                EditorGUILayout.Space(4);
                EditorGUILayout.BeginVertical(EditorStyles.helpBox);

                EditorGUILayout.LabelField(
                    "Height Above Ground",
                    EditorStyles.boldLabel);

                EditorGUILayout.PropertyField(
                    _minHeightAboveGround,
                    new GUIContent("Min Height"));

                EditorGUILayout.PropertyField(
                    _maxHeightAboveGround,
                    new GUIContent("Max Height"));

                EditorGUILayout.PropertyField(
                    _groundHeightOffset,
                    new GUIContent("Ground Offset"));

                EditorGUILayout.EndVertical();
            }

            serializedObject.ApplyModifiedProperties();

            EditorGUILayout.Space(8);

            ScannerGPU scanner = (ScannerGPU)target;

            if (GUILayout.Button("Start Scan", GUILayout.Height(28)))
            {
                EditorApplication.delayCall += () =>
                {
                    if (scanner != null)
                        scanner.StartScan();
                };
            }
        }
    }
#endif
}
