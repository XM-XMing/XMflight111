// filename: Assets/Scripts/XMConverters.cs
using System.Collections.Generic;
using System.Runtime.CompilerServices;
using UnityEngine;

namespace XMflight
{
    public static class XMConverters {

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Vector3 RosToUnityPos(IList<float> l)
            => new Vector3(-l[1], l[2], l[0]);

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Vector3 RosToUnityPos(float[] l)
            => new Vector3(-l[1], l[2], l[0]);

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float[] UnityToRosPosArray(Vector3 p)
            => new float[3] { p.z, -p.x, p.y };

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float[] UnityToRosRotArray(Quaternion q)
            => new float[4] { q.z, -q.x, q.y, -q.w };


        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void UnityToRosPosArrayNonAlloc(Vector3 p, float[] dst) {
            dst[0] = p.z;
            dst[1] = -p.x;
            dst[2] = p.y;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void UnityToRosRotArrayNonAlloc(Quaternion q, float[] dst) {
            dst[0] = q.z;
            dst[1] = -q.x;
            dst[2] = q.y;
            dst[3] = -q.w;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Vector3 RosBodyVelocityToUnityBody(float vx, float vy, float vz)
            => new Vector3(-vy, vz, vx);
    }
}
