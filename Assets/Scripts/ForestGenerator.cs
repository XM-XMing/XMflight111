using UnityEngine;
using System;
using System.Collections;
using System.Collections.Generic;
using System.IO;
using System.Linq;

#if UNITY_EDITOR
using UnityEditor;
#endif

namespace XMflight 
{
    public class ForestGenerator : MonoBehaviour {
        public enum MapMode { Square, Corridor }
        public enum SpawnType { PerlinNoise, Uniform }

        [Serializable]
        private sealed class SquareSettings {
            [Min(0.01f)] public float mapSize = 200f;
            [Min(0)] public int treeCount = 4000;
            [Min(0f)] public float centerClearRadius = 2.5f;
        }

        [Serializable]
        private sealed class CorridorSettings {
            [Min(0.01f)] public float width = 10f;
            [Min(0.01f)] public float length = 40f;
            [Min(0)] public int treeCount = 30;
            [Min(0f)] public float endpointClearRadius = 2.5f;
        }

        [Header("资源")]
        [SerializeField] private GameObject[] _treePrefabs;
        [SerializeField] private Terrain _targetTerrain;

        [Header("生成参数")]
        [SerializeField] private MapMode _mapMode = MapMode.Square;
        [SerializeField] private SpawnType _spawnType = SpawnType.Uniform;

        [Header("地图与树")]
        [SerializeField] private SquareSettings _squareSettings = new SquareSettings();
        [SerializeField] private CorridorSettings _corridorSettings = new CorridorSettings();

        [Header("碰撞体")]
        [SerializeField] private float _protectionHeight = 4.0f;
        [SerializeField] private string _obstacleLayerName = "Obstacle";

        [Header("分布控制")]
        [SerializeField] private float _minTreeDistance = 2.0f;
        [SerializeField] private float _noiseScale = 0.1f;
        [SerializeField, Range(0, 1)]
        private float _densityThreshold = 0.4f;

        [Header("随机与倾斜")]
        [SerializeField] private float _minScale = 1.0f;
        [SerializeField] private float _maxScale = 1.3f;
        [SerializeField, Range(0, 45)]
        private float _maxLeanAngle = 0.0f;
        [SerializeField] private int _seed = 50505;

        [Header("点云导出")]
        [SerializeField] private string _pointCloudSaveName = "forest_point_cloud.bin";
        [SerializeField] private string _pointCloudOutputDirectory = "/home/xm/XM/xm_ws/src/planning/data/map_data";
        [SerializeField] private float _finalScanHeight = 4.0f;
        [SerializeField] private float _surfaceDensity = 2000f;

        [Header("去噪")]
        [SerializeField] private bool _enableDenoise = true;
        [SerializeField] private float _denoiseThreshold = 0.2f;

        private const string ForestRootName = "Generated_Forest";
        private const int MaxAttemptsPerTree = 50;
        private const int MaxPointsPerTri = 1000;
        private const int DenoiseKNeighbors = 3;
        private const int WriteBufferPointCapacity = 65536;

        private static readonly string[] FoliageKeywords = { "leaf", "leaves", "crown", "canopy", "foliage"};

        private GameObject _forestParent;
        private readonly Dictionary<int, Mesh> _meshCache = new Dictionary<int, Mesh>();
        private readonly Dictionary<int, Vector3[]> _templateCache = new Dictionary<int, Vector3[]>();

        [ContextMenu("Generate Forest")]
        public void GenerateForest() {
            if (!ValidateGenerationInputs(out int obstacleLayer))
                return;

            ClearForest();
            _meshCache.Clear();
            _templateCache.Clear();
            UnityEngine.Random.InitState(_seed == 0 ? Environment.TickCount : _seed);
            float noiseOffset = UnityEngine.Random.Range(0f, 1000f);
            _forestParent = new GameObject(ForestRootName);
            _forestParent.transform.position = Vector3.zero;

            GetActiveGenerationSettings(
                out float minX,
                out float maxX,
                out float minZ,
                out float maxZ,
                out int targetTreeCount,
                out float squareClearRadius,
                out float corridorEndpointClearRadius);

            float effectiveMinTreeDistance = Mathf.Max(0.01f, _minTreeDistance);
            var spatialGrid = new SpatialHash2D(effectiveMinTreeDistance);
            int placed = 0;
            int attempts = 0;
            int maxAttempts = targetTreeCount * MaxAttemptsPerTree;

#if UNITY_EDITOR
            EditorUtility.DisplayProgressBar("生成森林", "种植中...", 0f);
#endif

            while (placed < targetTreeCount && attempts < maxAttempts) {
                attempts++;

                float x = UnityEngine.Random.Range(minX, maxX);
                float z = UnityEngine.Random.Range(minZ, maxZ);
                var candidate = new Vector2(x, z);

                if (IsInsideClearZone(
                    candidate,
                    minX,
                    maxX,
                    minZ,
                    maxZ,
                    squareClearRadius,
                    corridorEndpointClearRadius))
                    continue;

                if (!PassDistributionFilter(
                    candidate,
                    noiseOffset,
                    spatialGrid,
                    effectiveMinTreeDistance,
                    minX,
                    minZ))
                    continue;

                float y = _targetTerrain != null
                    ? _targetTerrain.SampleHeight(new Vector3(x, 0, z)) + _targetTerrain.transform.position.y
                    : 0f;

                if (!PlaceTree(new Vector3(x, y, z), obstacleLayer))
                    continue;

                spatialGrid.Insert(candidate);
                placed++;

#if UNITY_EDITOR
                if (attempts % 500 == 0 && targetTreeCount > 0)
                    EditorUtility.DisplayProgressBar(
                        "生成森林",
                        $"已种 {placed}/{targetTreeCount}...",
                        (float)placed / targetTreeCount);
#endif
            }

#if UNITY_EDITOR
            EditorUtility.ClearProgressBar();
#endif

            Debug.Log(
                $"[ForestGenerator] 模式: {_mapMode}, 范围 X[{minX}, {maxX}] Z[{minZ}, {maxZ}], " +
                $"生成完成: {placed}/{targetTreeCount} 棵 (尝试 {attempts} 次)");
        }

        private bool ValidateGenerationInputs(out int obstacleLayer) {
            obstacleLayer = LayerMask.NameToLayer(_obstacleLayerName);
            if (obstacleLayer < 0) {
                Debug.LogError($"[ForestGenerator] Layer '{_obstacleLayerName}' 不存在，请先在 Project Settings 中创建");
                return false;
            }

            if (_treePrefabs == null || _treePrefabs.Length == 0 || !_treePrefabs.Any(p => p != null)) {
                Debug.LogError("[ForestGenerator] Tree Prefabs 为空，无法生成森林");
                return false;
            }

            if (_minScale <= 0f || _maxScale <= 0f) {
                Debug.LogError("[ForestGenerator] Min/Max Scale 必须大于 0");
                return false;
            }

            if (_surfaceDensity < 0f) {
                Debug.LogError("[ForestGenerator] Surface Density 不能小于 0");
                return false;
            }

            return true;
        }

        private void GetActiveGenerationSettings(
            out float minX,
            out float maxX,
            out float minZ,
            out float maxZ,
            out int treeCount,
            out float squareClearRadius,
            out float corridorEndpointClearRadius)
        {
            if (_mapMode == MapMode.Square) {
                float mapSize = Mathf.Max(0.01f, _squareSettings.mapSize);
                float halfMap = mapSize * 0.5f;

                minX = -halfMap;
                maxX = halfMap;
                minZ = -halfMap;
                maxZ = halfMap;
                treeCount = Mathf.Max(0, _squareSettings.treeCount);
                squareClearRadius = Mathf.Max(0f, _squareSettings.centerClearRadius);
                corridorEndpointClearRadius = 0f;
                return;
            }

            float width = Mathf.Max(0.01f, _corridorSettings.width);
            float length = Mathf.Max(0.01f, _corridorSettings.length);
            float halfWidth = width * 0.5f;

            minX = -halfWidth;
            maxX = halfWidth;
            minZ = 0f;
            maxZ = length;
            treeCount = Mathf.Max(0, _corridorSettings.treeCount);
            squareClearRadius = 0f;
            corridorEndpointClearRadius = Mathf.Max(0f, _corridorSettings.endpointClearRadius);
        }

        private bool IsInsideClearZone(
            Vector2 pos,
            float minX,
            float maxX,
            float minZ,
            float maxZ,
            float squareClearRadius,
            float corridorEndpointClearRadius)
        {
            if (_mapMode == MapMode.Square) {
                return pos.sqrMagnitude <= squareClearRadius * squareClearRadius;
            }

            float centerX = (minX + maxX) * 0.5f;
            var startPoint = new Vector2(centerX, minZ);
            var endPoint = new Vector2(centerX, maxZ);
            float sqrRadius = corridorEndpointClearRadius * corridorEndpointClearRadius;

            return (pos - startPoint).sqrMagnitude <= sqrRadius ||
                   (pos - endPoint).sqrMagnitude <= sqrRadius;
        }

        private bool PassDistributionFilter(
            Vector2 pos,
            float noiseOff,
            SpatialHash2D grid,
            float minTreeDistance,
            float minX,
            float minZ)
        {
            if (grid.HasNeighborWithin(pos, minTreeDistance))
                return false;

            if (_spawnType == SpawnType.PerlinNoise) {
                float n = Mathf.PerlinNoise(
                    (pos.x - minX) * _noiseScale + noiseOff,
                    (pos.y - minZ) * _noiseScale + noiseOff);
                return n >= _densityThreshold;
            }

            return true;
        }

        private bool PlaceTree(Vector3 worldPos, int layer) {
            GameObject prefab = PickTreePrefab();
            if (prefab == null)
                return false;

            var tree = Instantiate(prefab, worldPos, Quaternion.identity, _forestParent.transform);

            tree.transform.rotation = Quaternion.Euler(
                UnityEngine.Random.Range(-_maxLeanAngle, _maxLeanAngle),
                UnityEngine.Random.Range(0f, 360f),
                UnityEngine.Random.Range(-_maxLeanAngle, _maxLeanAngle));

            float minScale = Mathf.Min(_minScale, _maxScale);
            float maxScale = Mathf.Max(_minScale, _maxScale);
            tree.transform.localScale = Vector3.one * UnityEngine.Random.Range(minScale, maxScale);

            foreach (var c in tree.GetComponentsInChildren<Collider>()) {
                if (Application.isPlaying)
                    Destroy(c);
                else
                    DestroyImmediate(c);
            }

            bool anyAdded = false;
            foreach (var mf in tree.GetComponentsInChildren<MeshFilter>()) {
                if (mf.sharedMesh == null)
                    continue;
                if (IsFoliageMesh(mf.gameObject.name))
                    continue;

                Mesh colMesh = GetOrCreateCollisionMesh(mf.sharedMesh);
                if (colMesh == null)
                    continue;

                var mc = mf.gameObject.AddComponent<MeshCollider>();
                mc.sharedMesh = colMesh;
                mc.convex = false;
                mc.isTrigger = false;
                SetLayerRecursive(mf.gameObject, layer);
                anyAdded = true;
            }

            if (!anyAdded) {
                MeshFilter best = null;
                int bestVert = 0;
                foreach (var mf in tree.GetComponentsInChildren<MeshFilter>()) {
                    if (mf.sharedMesh == null)
                        continue;
                    if (IsFoliageMesh(mf.gameObject.name))
                        continue;

                    int vc = mf.sharedMesh.vertexCount;
                    if (vc > bestVert) {
                        bestVert = vc;
                        best = mf;
                    }
                }

                if (best != null)  {
                    Mesh colMesh = GetOrCreateCollisionMesh(best.sharedMesh);
                    if (colMesh != null) {
                        var mc = best.gameObject.AddComponent<MeshCollider>();
                        mc.sharedMesh = colMesh;
                        mc.convex = false;
                        mc.isTrigger = false;
                        SetLayerRecursive(best.gameObject, layer);
                    }
                }
            }

            return true;
        }

        private GameObject PickTreePrefab() {
            int n = _treePrefabs == null ? 0 : _treePrefabs.Length;
            if (n == 0)
                return null;

            int start = UnityEngine.Random.Range(0, n);
            for (int i = 0; i < n; i++) {
                GameObject prefab = _treePrefabs[(start + i) % n];
                if (prefab != null)
                    return prefab;
            }

            return null;
        }

        private static bool IsFoliageMesh(string nodeName) {
            string lower = nodeName.ToLowerInvariant();
            foreach (string kw in FoliageKeywords)
                if (lower.Contains(kw))
                    return true;
            return false;
        }

        private Mesh GetOrCreateCollisionMesh(Mesh source) {
            int id = source.GetInstanceID();
            if (_meshCache.TryGetValue(id, out Mesh cached))
                return cached;

            Vector3[] srcVerts = source.vertices;
            int[] srcTris = source.triangles;
            var keepTris = new List<int>(srcTris.Length);

            for (int i = 0; i < srcTris.Length; i += 3) {
                if (srcVerts[srcTris[i]].y >= _protectionHeight &&
                    srcVerts[srcTris[i + 1]].y >= _protectionHeight &&
                    srcVerts[srcTris[i + 2]].y >= _protectionHeight)
                    continue;

                keepTris.Add(srcTris[i]);
                keepTris.Add(srcTris[i + 1]);
                keepTris.Add(srcTris[i + 2]);
            }

            if (keepTris.Count == 0) {
                _meshCache[id] = null;
                return null;
            }

            var oldToNew = new Dictionary<int, int>(keepTris.Count);
            var newVerts = new List<Vector3>(keepTris.Count);
            var remappedTris = new int[keepTris.Count];

            for (int i = 0; i < keepTris.Count; i++) {
                int oldIdx = keepTris[i];
                if (!oldToNew.TryGetValue(oldIdx, out int newIdx)) {
                    newIdx = newVerts.Count;
                    oldToNew[oldIdx] = newIdx;
                    newVerts.Add(srcVerts[oldIdx]);
                }
                remappedTris[i] = newIdx;
            }

            var newMesh = new Mesh { name = source.name + "_Col" };
            newMesh.indexFormat = newVerts.Count > 65535
                ? UnityEngine.Rendering.IndexFormat.UInt32
                : UnityEngine.Rendering.IndexFormat.UInt16;

            newMesh.SetVertices(newVerts);
            newMesh.SetTriangles(remappedTris, 0);
            newMesh.RecalculateBounds();

            _meshCache[id] = newMesh;
            return newMesh;
        }

        private static void SetLayerRecursive(GameObject go, int layer) {
            go.layer = layer;
            foreach (Transform child in go.transform)
                SetLayerRecursive(child.gameObject, layer);
        }

        [ContextMenu("Scan Mesh Collider")]
        public void ScanCollidersDirectly() {
            if (_forestParent == null)
                _forestParent = GameObject.Find(ForestRootName);

            if (_forestParent == null) {
                Debug.LogError("[ForestGenerator] 未找到森林根节点，请先执行 Generate Forest");
                return;
            }

            _templateCache.Clear();
            string path = BuildExportPath();
            Debug.Log($"[ForestGenerator] 开始扫描，高度限制: {_finalScanHeight}m → {path}");
            var sw = System.Diagnostics.Stopwatch.StartNew();
            long totalWritten = 0;

            using (var fs = new FileStream(path, FileMode.Create))
            using (var bw = new BinaryWriter(fs)) {
                bw.Write((int)0);

                var groups = _forestParent.GetComponentsInChildren<MeshCollider>()
                    .Where(mc => mc.sharedMesh != null)
                    .GroupBy(mc => mc.sharedMesh)
                    .ToList();

                if (groups.Count == 0)
                    Debug.LogWarning("[ForestGenerator] 未找到可扫描的 MeshCollider，输出点云为空");

                for (int g = 0; g < groups.Count; g++) {
                    Mesh mesh = groups[g].Key;

#if UNITY_EDITOR
                    EditorUtility.DisplayProgressBar("扫描点云", $"网格 {g + 1}/{groups.Count}: {mesh.name}", (float)g / groups.Count);
#endif

                    Vector3[] template = GetOrCreateTemplate(mesh);
                    if (template.Length == 0)
                        continue;

                    Matrix4x4[] matrices = groups[g]
                        .Select(mc => mc.transform.localToWorldMatrix)
                        .ToArray();

                    totalWritten += TransformAndWrite(bw, template, matrices);
                }

                bw.Flush();
                const long bytesPerPoint = sizeof(float) * 3L;
                long payloadBytes = fs.Length - sizeof(int);
                if (payloadBytes < 0 || payloadBytes % bytesPerPoint != 0)
                    throw new InvalidDataException(
                        $"Invalid point cloud payload length: fileBytes={fs.Length}, payloadBytes={payloadBytes}");

                long actualWritten = payloadBytes / bytesPerPoint;
                if (actualWritten != totalWritten)
                    throw new InvalidDataException(
                        $"Point count mismatch while writing: counted={totalWritten}, actual={actualWritten}");
                if (actualWritten > int.MaxValue)
                    throw new InvalidDataException(
                        $"Point count exceeds int32 header capacity: {actualWritten}");

                fs.Seek(0, SeekOrigin.Begin);
                bw.Write((int)actualWritten);
                bw.Flush();
            }

#if UNITY_EDITOR
            EditorUtility.ClearProgressBar();
            AssetDatabase.Refresh();
#endif

            sw.Stop();
            Debug.Log($"[ForestGenerator] 扫描完成: {totalWritten:N0} 点，耗时 {sw.ElapsedMilliseconds}ms");

        }

        private Vector3[] GetOrCreateTemplate(Mesh mesh) {
            int id = mesh.GetInstanceID();
            if (_templateCache.TryGetValue(id, out Vector3[] cached))
                return cached;

            var pts = SampleMeshSurface(mesh);
            if (_enableDenoise && pts.Count > DenoiseKNeighbors)
                pts = DenoisePoints(pts);

            Vector3[] arr = pts.ToArray();
            _templateCache[id] = arr;
            return arr;
        }

        private List<Vector3> SampleMeshSurface(Mesh mesh) {
            var pts = new List<Vector3>(8192);
            float density = Mathf.Max(0f, _surfaceDensity);
            if (density <= 0f)
                return pts;

            int[] tris = mesh.triangles;
            Vector3[] verts = mesh.vertices;

            for (int i = 0; i < tris.Length; i += 3) {
                Vector3 v0 = verts[tris[i]];
                Vector3 v1 = verts[tris[i + 1]];
                Vector3 v2 = verts[tris[i + 2]];
                float area = Vector3.Cross(v1 - v0, v2 - v0).magnitude * 0.5f;
                float raw = area * density;
                if (raw <= 0f)
                    continue;
                int count = Mathf.Min(Mathf.FloorToInt(raw), MaxPointsPerTri);

                if (UnityEngine.Random.value < raw - count)
                    count++;

                for (int k = 0; k < count; k++) {
                    float r1 = Mathf.Sqrt(UnityEngine.Random.value);
                    float r2 = UnityEngine.Random.value;
                    pts.Add(v0 * (1f - r1) + v1 * (r1 * (1f - r2)) + v2 * (r1 * r2));
                }
            }

            return pts;
        }

        private List<Vector3> DenoisePoints(List<Vector3> points) {
            if (_denoiseThreshold <= 1e-5f)
                return points;

            float cell = _denoiseThreshold;
            float sqThr = _denoiseThreshold * _denoiseThreshold;
            int n = points.Count;

            var grid = new Dictionary<long, List<int>>(n / 4);
            for (int i = 0; i < n; i++) {
                long key = HashCell(points[i], cell);
                if (!grid.TryGetValue(key, out var list)) {
                    list = new List<int>(8);
                    grid[key] = list;
                }
                list.Add(i);
            }

            var keep = new BitArray(n, false);
            int kept = 0;

            for (int i = 0; i < n; i++) {
                Vector3 p = points[i];
                int cx = Mathf.FloorToInt(p.x / cell);
                int cy = Mathf.FloorToInt(p.y / cell);
                int cz = Mathf.FloorToInt(p.z / cell);
                int neighbors = 0;
                bool enough = false;

                for (int dx = -1; dx <= 1 && !enough; dx++)
                    for (int dy = -1; dy <= 1 && !enough; dy++)
                        for (int dz = -1; dz <= 1 && !enough; dz++) {
                            long nKey = PackCell(cx + dx, cy + dy, cz + dz);
                            if (!grid.TryGetValue(nKey, out var indices))
                                continue;

                            foreach (int idx in indices) {
                                if (idx == i)
                                    continue;
                                if ((points[idx] - p).sqrMagnitude >= sqThr)
                                    continue;

                                if (++neighbors >= DenoiseKNeighbors) {
                                    enough = true;
                                    break;
                                }
                            }
                        }

                if (enough) {
                    keep[i] = true;
                    kept++;
                }
            }

            var result = new List<Vector3>(kept);
            for (int i = 0; i < n; i++)
                if (keep[i])
                    result.Add(points[i]);

            return result;
        }

        private long TransformAndWrite(BinaryWriter bw, Vector3[] template, Matrix4x4[] matrices)
        {
            long written = 0;
            float localHeightLimit = Mathf.Max(0f, Mathf.Min(_protectionHeight, _finalScanHeight)) - 0.01f;
            Vector3[] writeBuffer = new Vector3[WriteBufferPointCapacity];
            int buffered = 0;

            for (int i = 0; i < matrices.Length; i++) {
                Matrix4x4 mat = matrices[i];
                float rootY = mat.m13;

                for (int t = 0; t < template.Length; t++) {
                    Vector3 localPoint = template[t];
                    if (localPoint.y >= localHeightLimit)
                        continue;

                    Vector3 worldPoint = mat.MultiplyPoint3x4(localPoint);
                    if (worldPoint.y - rootY > _finalScanHeight)
                        continue;

                    writeBuffer[buffered++] = worldPoint;
                    if (buffered >= writeBuffer.Length) {
                        WritePoints(bw, writeBuffer, buffered);
                        written += buffered;
                        buffered = 0;
                    }
                }
            }

            if (buffered > 0) {
                WritePoints(bw, writeBuffer, buffered);
                written += buffered;
            }

            return written;
        }

        private static void WritePoints(BinaryWriter bw, Vector3[] points, int count) {
            for (int i = 0; i < count; i++) {
                Vector3 p = points[i];
                bw.Write(p.x);
                bw.Write(p.y);
                bw.Write(p.z);
            }
        }

        private string BuildExportPath() {
            string name = string.IsNullOrWhiteSpace(_pointCloudSaveName)
                ? "forest_point_cloud.bin"
                : _pointCloudSaveName.Trim();
            if (!name.EndsWith(".bin", StringComparison.OrdinalIgnoreCase))
                name = Path.ChangeExtension(name, ".bin");

            string dir = string.IsNullOrWhiteSpace(_pointCloudOutputDirectory)
                ? Path.Combine(Application.dataPath, "..", "data", "map_data")
                : _pointCloudOutputDirectory.Trim();
            if (!Path.IsPathRooted(dir))
                dir = Path.Combine(Application.dataPath, "..", dir);
            dir = Path.GetFullPath(dir);
            Directory.CreateDirectory(dir);
            return Path.Combine(dir, name);
        }

        private static long HashCell(Vector3 p, float cell) {
            return PackCell(
                Mathf.FloorToInt(p.x / cell),
                Mathf.FloorToInt(p.y / cell),
                Mathf.FloorToInt(p.z / cell));
        }

        private static long PackCell(int x, int y, int z) {
            const long mask = 0x1FFFFF;
            return ((long)(x & mask) << 42) | ((long)(y & mask) << 21) | (long)(z & mask);
        }

        [ContextMenu("Clear Forest")]
        public void ClearForest() {
            var old = GameObject.Find(ForestRootName);
            if (old == null)
                return;

            if (Application.isPlaying)
                Destroy(old);
            else
                DestroyImmediate(old);
        }

        private sealed class SpatialHash2D {
            private readonly float _cellSize;
            private readonly Dictionary<long, List<Vector2>> _cells = new Dictionary<long, List<Vector2>>();

            public SpatialHash2D(float minDist) {
                _cellSize = minDist;
            }

            public void Insert(Vector2 p) {
                long key = Key(p);
                if (!_cells.TryGetValue(key, out var list)) {
                    list = new List<Vector2>(4);
                    _cells[key] = list;
                }
                list.Add(p);
            }

            public bool HasNeighborWithin(Vector2 p, float radius) {
                float sqr = radius * radius;
                int cx = Mathf.FloorToInt(p.x / _cellSize);
                int cy = Mathf.FloorToInt(p.y / _cellSize);

                for (int dx = -1; dx <= 1; dx++)
                    for (int dy = -1; dy <= 1; dy++) {
                        long k = Pack(cx + dx, cy + dy);
                        if (!_cells.TryGetValue(k, out var list))
                            continue;

                        foreach (var q in list)
                            if ((q - p).sqrMagnitude < sqr)
                                return true;
                    }

                return false;
            }

            private long Key(Vector2 p) {
                return Pack(Mathf.FloorToInt(p.x / _cellSize), Mathf.FloorToInt(p.y / _cellSize));
            }

            private static long Pack(int x, int y) {
                return ((long)x << 32) | (uint)y;
            }
        }
    }

#if UNITY_EDITOR
    [CustomEditor(typeof(ForestGenerator))]
    public sealed class ForestGeneratorEditor : Editor {
        private SerializedProperty _treePrefabs;
        private SerializedProperty _targetTerrain;

        private SerializedProperty _mapMode;
        private SerializedProperty _spawnType;

        private SerializedProperty _squareMapSize;
        private SerializedProperty _squareTreeCount;
        private SerializedProperty _squareCenterClearRadius;

        private SerializedProperty _corridorWidth;
        private SerializedProperty _corridorLength;
        private SerializedProperty _corridorTreeCount;
        private SerializedProperty _corridorEndpointClearRadius;

        private SerializedProperty _protectionHeight;
        private SerializedProperty _obstacleLayerName;

        private SerializedProperty _minTreeDistance;
        private SerializedProperty _noiseScale;
        private SerializedProperty _densityThreshold;

        private SerializedProperty _minScale;
        private SerializedProperty _maxScale;
        private SerializedProperty _maxLeanAngle;
        private SerializedProperty _seed;

        private SerializedProperty _pointCloudSaveName;
        private SerializedProperty _pointCloudOutputDirectory;
        private SerializedProperty _finalScanHeight;
        private SerializedProperty _surfaceDensity;

        private SerializedProperty _enableDenoise;
        private SerializedProperty _denoiseThreshold;

        private void OnEnable() {
            _treePrefabs = serializedObject.FindProperty("_treePrefabs");
            _targetTerrain = serializedObject.FindProperty("_targetTerrain");

            _mapMode = serializedObject.FindProperty("_mapMode");
            _spawnType = serializedObject.FindProperty("_spawnType");

            SerializedProperty squareSettings = serializedObject.FindProperty("_squareSettings");
            _squareMapSize = squareSettings.FindPropertyRelative("mapSize");
            _squareTreeCount = squareSettings.FindPropertyRelative("treeCount");
            _squareCenterClearRadius = squareSettings.FindPropertyRelative("centerClearRadius");

            SerializedProperty corridorSettings = serializedObject.FindProperty("_corridorSettings");
            _corridorWidth = corridorSettings.FindPropertyRelative("width");
            _corridorLength = corridorSettings.FindPropertyRelative("length");
            _corridorTreeCount = corridorSettings.FindPropertyRelative("treeCount");
            _corridorEndpointClearRadius = corridorSettings.FindPropertyRelative("endpointClearRadius");

            _protectionHeight = serializedObject.FindProperty("_protectionHeight");
            _obstacleLayerName = serializedObject.FindProperty("_obstacleLayerName");

            _minTreeDistance = serializedObject.FindProperty("_minTreeDistance");
            _noiseScale = serializedObject.FindProperty("_noiseScale");
            _densityThreshold = serializedObject.FindProperty("_densityThreshold");

            _minScale = serializedObject.FindProperty("_minScale");
            _maxScale = serializedObject.FindProperty("_maxScale");
            _maxLeanAngle = serializedObject.FindProperty("_maxLeanAngle");
            _seed = serializedObject.FindProperty("_seed");

            _pointCloudSaveName = serializedObject.FindProperty("_pointCloudSaveName");
            _pointCloudOutputDirectory = serializedObject.FindProperty("_pointCloudOutputDirectory");
            _finalScanHeight = serializedObject.FindProperty("_finalScanHeight");
            _surfaceDensity = serializedObject.FindProperty("_surfaceDensity");

            _enableDenoise = serializedObject.FindProperty("_enableDenoise");
            _denoiseThreshold = serializedObject.FindProperty("_denoiseThreshold");
        }

        public override void OnInspectorGUI() {
            serializedObject.Update();

            EditorGUILayout.PropertyField(_treePrefabs, new GUIContent("Tree Prefabs"), true);
            EditorGUILayout.PropertyField(_targetTerrain, new GUIContent("Target Terrain"));

            EditorGUILayout.PropertyField(_mapMode, new GUIContent("地图模式"));
            EditorGUILayout.PropertyField(_spawnType, new GUIContent("分布状态"));

            ForestGenerator.MapMode mode =
                (ForestGenerator.MapMode)_mapMode.enumValueIndex;

            if (mode == ForestGenerator.MapMode.Square) {
                EditorGUILayout.PropertyField(_squareMapSize, new GUIContent("地图尺寸 (m)"));
                EditorGUILayout.PropertyField(_squareTreeCount, new GUIContent("树木总数"));
                EditorGUILayout.PropertyField(
                    _squareCenterClearRadius,
                    new GUIContent("中心无树半径 (m)"));

                float size = Mathf.Max(0f, _squareMapSize.floatValue);
                EditorGUILayout.HelpBox(
                    $"范围:X {-size * 0.5f:0.##} ~{size * 0.5f:0.##}," +
                    $"Z {-size * 0.5f:0.##} ~ {size * 0.5f:0.##}",
                    MessageType.Info);
            }
            else {
                EditorGUILayout.PropertyField(_corridorWidth, new GUIContent("地图宽度 X (m)"));
                EditorGUILayout.PropertyField(_corridorLength, new GUIContent("地图长度 Z (m)"));
                EditorGUILayout.PropertyField(_corridorTreeCount, new GUIContent("树木总数"));
                EditorGUILayout.PropertyField(_corridorEndpointClearRadius, new GUIContent("端点无树半径 (m)"));

                float width = Mathf.Max(0f, _corridorWidth.floatValue);
                float length = Mathf.Max(0f, _corridorLength.floatValue);
                float radius = Mathf.Max(0f, _corridorEndpointClearRadius.floatValue);
                EditorGUILayout.HelpBox(
                    $"范围:X {-width * 0.5f:0.##} ~ {width * 0.5f:0.##},Z 0 ~ {length:0.##}\n" +
                    $"无树区：(0, 0) 与 (0, {length:0.##})，半径 {radius:0.##}m",
                    MessageType.Info);
            }

            EditorGUILayout.PropertyField(_protectionHeight, new GUIContent("Protection Height"));
            EditorGUILayout.PropertyField(_obstacleLayerName, new GUIContent("Obstacle Layer Name"));

            EditorGUILayout.PropertyField(_minTreeDistance, new GUIContent("Min Tree Distance"));

            ForestGenerator.SpawnType spawnType =
                (ForestGenerator.SpawnType)_spawnType.enumValueIndex;

            if (spawnType == ForestGenerator.SpawnType.PerlinNoise) {
                EditorGUILayout.PropertyField(_noiseScale, new GUIContent("Noise Scale"));
                EditorGUILayout.PropertyField(_densityThreshold, new GUIContent("Density Threshold"));
            }

            EditorGUILayout.PropertyField(_minScale, new GUIContent("Min Scale"));
            EditorGUILayout.PropertyField(_maxScale, new GUIContent("Max Scale"));
            EditorGUILayout.PropertyField(_maxLeanAngle, new GUIContent("Max Lean Angle"));
            EditorGUILayout.PropertyField(_seed, new GUIContent("Seed"));

            EditorGUILayout.PropertyField(_pointCloudSaveName, new GUIContent("Point Cloud Save Name"));
            EditorGUILayout.PropertyField(_pointCloudOutputDirectory, new GUIContent("Point Cloud Output Directory"));
            EditorGUILayout.PropertyField(_finalScanHeight, new GUIContent("Final Scan Height"));
            EditorGUILayout.PropertyField(_surfaceDensity, new GUIContent("Surface Density"));


            EditorGUILayout.PropertyField(_enableDenoise, new GUIContent("Enable Denoise"));
            if (_enableDenoise.boolValue)
                EditorGUILayout.PropertyField(_denoiseThreshold, new GUIContent("Denoise Threshold"));

            serializedObject.ApplyModifiedProperties();

            EditorGUILayout.Space(10f);

            if (GUILayout.Button("Generate Forest", GUILayout.Height(28f))) {
                foreach (UnityEngine.Object item in targets) {
                    var generator = (ForestGenerator)item;
                    generator.GenerateForest();
                    EditorUtility.SetDirty(generator);
                }
            }

            using (new EditorGUILayout.HorizontalScope()) {
                if (GUILayout.Button("Scan Mesh Collider")) {
                    foreach (UnityEngine.Object item in targets)
                        ((ForestGenerator)item).ScanCollidersDirectly();
                }


                if (GUILayout.Button("Clear Forest")) {
                    foreach (UnityEngine.Object item in targets)
                        ((ForestGenerator)item).ClearForest();
                }
            }
        }

        private static void DrawSection(string title) {
            EditorGUILayout.Space(8f);
            EditorGUILayout.LabelField(title, EditorStyles.boldLabel);
        }
    }
#endif
}
