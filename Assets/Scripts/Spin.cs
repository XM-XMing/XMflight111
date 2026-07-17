// filename: Assets/Scripts/Spin.cs
using UnityEngine;

namespace XMflight
{
    public class Spin : MonoBehaviour {
        [Header("Settings")]
        public float Speed = 2000f; 
        public Vector3 RotationAxis = Vector3.up; 

        void Update() {
            transform.Rotate(RotationAxis, Speed * Time.deltaTime);
        }
    }
}