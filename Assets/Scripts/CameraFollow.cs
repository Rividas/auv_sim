using UnityEngine;

public class CameraFollow : MonoBehaviour
{
    [Header("Target Settings")]
    public Transform target;
    public bool lookAtTarget = true;

    [Header("Position Settings")]
    public float followDistance = 8f;
    public float followHeight = 3f;
    public float followOffset = 0f;

    [Header("Smooth Settings")]
    public bool smoothFollow = true;
    public float smoothSpeed = 5f;

    [Header("Rotation Settings")]
    public bool manualRotation = false;
    public float rotationSpeed = 100f;
    public float minVerticalAngle = -30f;
    public float maxVerticalAngle = 60f;

    [Header("Debug")]
    public bool showGizmos = true;

    private Vector3 currentVelocity;
    private float horizontalRotation = 0f;
    private float verticalRotation = 30f;
    private Camera cam;

    void Start()
    {
        cam = GetComponent<Camera>();

        if (target == null)
        {
            GameObject auv = GameObject.Find("AUV_Root");
            if (auv != null)
            {
                target = auv.transform;
                Debug.Log("CameraFollow: Target auto-assigned to AUV_Root");
            }
            else
            {
                Debug.LogWarning("CameraFollow: No target found! Assign manually.");
            }
        }
    }

    void LateUpdate()
    {
        if (target == null) return;

        if (manualRotation)
        {
            horizontalRotation += Input.GetAxis("Mouse X") * rotationSpeed * Time.deltaTime;
            verticalRotation -= Input.GetAxis("Mouse Y") * rotationSpeed * Time.deltaTime;
            verticalRotation = Mathf.Clamp(verticalRotation, minVerticalAngle, maxVerticalAngle);
        }

        Vector3 targetPosition;

        if (manualRotation)
        {
            Quaternion rotation = Quaternion.Euler(verticalRotation, horizontalRotation, 0);
            Vector3 offset = rotation * new Vector3(0, 0, -followDistance);
            targetPosition = target.position + offset + Vector3.up * followHeight;
        }
        else
        {
            Vector3 behindTarget = target.position - target.forward * followDistance;
            targetPosition = behindTarget + Vector3.up * followHeight + target.forward * followOffset;
        }

        if (smoothFollow)
        {
            transform.position = Vector3.SmoothDamp(transform.position, targetPosition, ref currentVelocity, 1f / smoothSpeed);
        }
        else
        {
            transform.position = targetPosition;
        }

        if (lookAtTarget)
        {
            transform.LookAt(target.position + Vector3.up * 1f);
        }
    }

    void OnDrawGizmos()
    {
        if (!showGizmos || target == null) return;

        Gizmos.color = Color.cyan;
        Gizmos.DrawLine(transform.position, target.position);
        Gizmos.DrawWireSphere(target.position, 0.5f);
    }
}