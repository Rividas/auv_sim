using UnityEngine;

public class WaypointVisualizer : MonoBehaviour
{
    public Vector3[] waypoints;
    public float sphereSize = 0.5f;
    public Color waypointColor = Color.green;
    public Color lineColor = Color.yellow;

    void OnDrawGizmos()
    {
        if (waypoints == null || waypoints.Length == 0) return;

        Gizmos.color = waypointColor;
        foreach (var wp in waypoints)
        {
            Gizmos.DrawWireSphere(wp, sphereSize);
        }

        Gizmos.color = lineColor;
        for (int i = 0; i < waypoints.Length - 1; i++)
        {
            Gizmos.DrawLine(waypoints[i], waypoints[i + 1]);
        }


        if (waypoints.Length > 2)
        {
            Gizmos.DrawLine(waypoints[waypoints.Length - 1], waypoints[0]);
        }
    }
}