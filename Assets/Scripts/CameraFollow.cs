// filename: Assets/Scripts/Other/CameraFollow.cs
using UnityEngine;

namespace XMflight
{
    public class CameraFollow : MonoBehaviour {
        public Transform target;

        [Header("Follow Settings")]
        public Vector3 offset = new Vector3(0, 1, -2);
        
        [Range(0.1f, 20f)]
        public float smoothSpeed = 10f;       
        [Range(0.1f, 20f)]
        public float rotationSmoothness = 15f;  

        public void SetTarget(Transform t) {
            target = t;
        }

        void LateUpdate() {
            if (target == null) return;
            
            Vector3 desiredPos = target.TransformPoint(offset);
            
            transform.position = Vector3.Lerp(transform.position, desiredPos, smoothSpeed * Time.deltaTime);

            Quaternion targetRotation = Quaternion.LookRotation(target.position - transform.position);
            transform.rotation = Quaternion.Slerp(transform.rotation, targetRotation, rotationSmoothness * Time.deltaTime);
        }
    }
}