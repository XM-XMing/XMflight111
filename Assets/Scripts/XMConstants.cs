// filename: Assets/Scripts/XMConstants.cs
namespace XMflight
{
    public static class XMConstants {
        public const double SecondsToNanoseconds = 1_000_000_000.0;

        public const float Gravity = 9.81f;
        public const float CollisionLatchDuration = 0.2f;
        public const float DepthMetersToMillimeters = 1000f;

        public const int DepthBytesPerPixel16UC1 = 2;
        public const int DefaultDepthPoolSize = 8;

        public const int MinImageSize = 16;
        public const float MinCameraFovDeg = 1f;
        public const float MaxCameraFovDeg = 179f;

        public const float DepthCameraNearClip = 0.1f;
        public const float DepthFarClipPadding = 5.0f;

        public const int RaycastBatchSize = 32;
        public const int MinRayCount = 32;
        public const int OverlapBufferSize = 32;

        public const string UberDepthShaderName = "Hidden/XM_UberReplacement";
        public const string DepthVisualizeShaderName = "Hidden/XM_DepthVisualize";
    }
}