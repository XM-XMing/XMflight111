// filename: Assets/Scripts/XMProtocol.cs
using MessagePack;

namespace XMflight
{
    public static class XMProtocol {
        public const int SchemaVersion = 3;
        public const int ModeVelocity = 0;
        public const int ModeTeleport = 1;
        public const int ModeStep = 2;
        public const int ModeTrajectory = 3;
        public const int ModePrimitiveExecution = 4;

        public const int ExecutionStatusNone = 0;
        public const int ExecutionStatusFrameApplied = 1;
        public const int ExecutionStatusComplete = 2;

        public const int DepthEncoding16UC1 = 1;
        public const int ByteOrderLittleEndian = 0;
        public const int FlagCollision = 1 << 0;
        public const int FlagAltitudeViolation = 1 << 1;

        public const int DynamicsStateFieldCount = 14;
        public const int DepthFrameFieldCount = 13;
        public const int CommandFieldCount = 10;
        public const int CameraFieldCount = 8;
        public const int DepthMetaFieldCount = 5;

        public static class DynamicsStateIndex {
            public const int SchemaVersion = 0;
            public const int StateId = 1;
            public const int SimTimeNs = 2;
            public const int Flags = 3;
            public const int MinClearance = 4;
            public const int CurrPos = 5;
            public const int CurrRot = 6;
            public const int CurrVel = 7;
            public const int CurrAcc = 8;
            public const int FrontClearances = 9;
            public const int AppliedExecutionId = 10;
            public const int AppliedExecutionFrameIndex = 11;
            public const int AppliedCommandId = 12;
            public const int ExecutionStatus = 13;
        }

        public static class DepthFrameIndex {
            public const int SchemaVersion = 0;
            public const int CaptureId = 1;
            public const int SimTimeNs = 2;
            public const int Flags = 3;
            public const int CapturePos = 4;
            public const int CaptureRot = 5;
            public const int CaptureVel = 6;
            public const int CaptureAcc = 7;
            public const int CaptureForward = 8;
            public const int Camera = 9;
            public const int DepthMeta = 10;
            public const int MinClearance = 11;
            public const int FrontClearances = 12;
        }

        public static class CommandIndex {
            public const int SchemaVersion = 0;
            public const int Mode = 1;
            public const int Action = 2;
            public const int Position = 3;
            public const int ClientTimeNs = 4;
            public const int CommandId = 5;
            public const int ExecutionId = 6;
            public const int ExecutionFrameIndex = 7;
            public const int ExecutionFrameCount = 8;
            public const int ExecutionFrames = 9;
        }
    }

    public sealed class ControlCommandMsg {
        public int schema_version;
        public int mode;
        public float[] action;
        public float[] position;
        public long client_time_ns;
        public long command_id;
        public long execution_id;
        public int execution_frame_index;
        public int execution_frame_count;
        public PrimitiveExecutionFrameMsg[] execution_frames;
    }

    public sealed class PrimitiveExecutionFrameMsg {
        public int frame_index;
        public long command_id;
        public float[] action;
    }
}
