using System;
using System.Net;
using System.Net.Sockets;
using System.Text;
using System.Threading;
using System.Threading.Tasks;
using UnityEngine;
using System.Collections.Generic;
using System.Globalization;
using Random = UnityEngine.Random;

public class AUVController : MonoBehaviour
{
    [Header("Network Settings")]
    public int listenPort = 5000;
    public int sendPort = 5001;

    [Header("Thrusters")]
    public Transform thrusterVLeft;
    public Transform thrusterVRight;
    public Transform thrusterHLeft;
    public Transform thrusterHRight;

    [Header("Physics")]
    public float buoyancyForce = 490.5f;
    public float waterDrag = 50f;
    public float waterAngularDrag = 50f;

    [Header("Sonar Settings")]
    [Tooltip("Дальность сонара в метрах")]
    public float sonarMaxRange = 15f;
    [Tooltip("Угол обзора сонара по горизонтали (±градусы)")]
    public float sonarHorizontalFOV = 60f;
    [Tooltip("Угол обзора сонара по вертикали (±градусы)")]
    public float sonarVerticalFOV = 20f;
    [Tooltip("Количество лучей по горизонтали")]
    public int sonarHorizontalRays = 24;
    [Tooltip("Количество лучей по вертикали")]
    public int sonarVerticalRays = 3;
    [Tooltip("Слой объектов, которые считаются препятствиями")]
    public LayerMask obstacleLayer = ~0;
    [Tooltip("Добавлять шум к измерениям (реализм)")]
    public bool addSonarNoise = true;
    [Tooltip("Амплитуда шума (доля от расстояния)")]
    public float sonarNoiseAmplitude = 0.03f;

    [Header("Telemetry")]
    public float telemetryRate = 20f;

    private UdpClient udpListener;
    private CancellationTokenSource cancellationTokenSource;
    private float[] thrusterForces = new float[4];
    private Rigidbody rb;
    private bool isInitialized = false;
    private Task receiveTask;
    private float telemetryTimer = 0f;

    private List<SonarReading> sonarReadings = new List<SonarReading>();
    private struct SonarReading
    {
        public float angle;
        public float dist;
        public SonarReading(float a, float d) { angle = a; dist = d; }
    }

    void Start()
    {
        rb = GetComponent<Rigidbody>();
        cancellationTokenSource = new CancellationTokenSource();

        try
        {
            udpListener = new UdpClient(listenPort);
            udpListener.EnableBroadcast = true;
            isInitialized = true;
            Debug.Log($"AUV Simulator started. Listening on port {listenPort}");
            Debug.Log($"Sonar: {sonarHorizontalRays}×{sonarVerticalRays} rays, FOV ±{sonarHorizontalFOV}°/{sonarVerticalFOV}°, range {sonarMaxRange}m");
            receiveTask = ReceiveData(cancellationTokenSource.Token);
        }
        catch (Exception e)
        {
            Debug.LogError($"Failed to initialize UDP: {e.Message}");
            isInitialized = false;
        }
    }

    void FixedUpdate()
    {
        if (!isInitialized || !gameObject.activeInHierarchy) return;

        ApplyHydrodynamics();
        ApplyThrusterForces();

        telemetryTimer += Time.fixedDeltaTime;
        if (telemetryTimer >= 1f / telemetryRate)
        {
            SendTelemetry();
            telemetryTimer = 0f;
        }
    }

    void ApplyHydrodynamics()
    {
        rb.AddForce(Vector3.up * buoyancyForce, ForceMode.Force);

        Vector3 velocity = rb.linearVelocity;
        if (velocity.magnitude > 0.01f)
        {
            Vector3 dragForce = -velocity.normalized * velocity.magnitude * waterDrag * 0.7f;
            dragForce += -velocity * waterDrag * 0.3f;
            rb.AddForce(dragForce, ForceMode.Force);
        }

        Vector3 angVelocity = rb.angularVelocity;
        if (angVelocity.magnitude > 0.01f)
        {
            Vector3 angDragTorque = -angVelocity.normalized * angVelocity.magnitude * waterAngularDrag;
            rb.AddTorque(angDragTorque, ForceMode.Force);
        }
    }

    void ApplyThrusterForces()
    {
        if (thrusterVLeft != null)
            rb.AddForceAtPosition(thrusterVLeft.up * thrusterForces[0], thrusterVLeft.position, ForceMode.Force);

        if (thrusterVRight != null)
            rb.AddForceAtPosition(thrusterVRight.up * thrusterForces[1], thrusterVRight.position, ForceMode.Force);

        if (thrusterHLeft != null)
            rb.AddForceAtPosition(thrusterHLeft.forward * thrusterForces[2], thrusterHLeft.position, ForceMode.Force);

        if (thrusterHRight != null)
            rb.AddForceAtPosition(thrusterHRight.forward * thrusterForces[3], thrusterHRight.position, ForceMode.Force);
    }

    List<SonarReading> ScanSonar()
    {
        sonarReadings.Clear();

        Vector3 sonarOrigin = transform.position;
        Quaternion sonarBase = transform.rotation;

        for (int v = 0; v < sonarVerticalRays; v++)
        {
            float vAngle = Mathf.Lerp(-sonarVerticalFOV, sonarVerticalFOV, (float)v / (sonarVerticalRays - 1));

            for (int h = 0; h < sonarHorizontalRays; h++)
            {
                float hAngle = Mathf.Lerp(-sonarHorizontalFOV, sonarHorizontalFOV, (float)h / (sonarHorizontalRays - 1));

                Quaternion rayRot = Quaternion.Euler(vAngle, hAngle, 0);
                Vector3 rayDir = sonarBase * rayRot * Vector3.forward;

                if (Physics.Raycast(sonarOrigin, rayDir, out RaycastHit hit, sonarMaxRange, obstacleLayer.value))
                {
                    float dist = hit.distance;

                    if (addSonarNoise && sonarNoiseAmplitude > 0)
                    {
                        float noise = Random.Range(-1f, 1f) * sonarNoiseAmplitude;
                        dist *= (1f + noise);
                        dist = Mathf.Max(0.1f, dist);
                    }

                    sonarReadings.Add(new SonarReading(hAngle, dist));

#if UNITY_EDITOR
                    Debug.DrawRay(sonarOrigin, rayDir * dist, Color.yellow, 0.1f);
#endif
                }
                else
                {
                    sonarReadings.Add(new SonarReading(hAngle, sonarMaxRange + 1f));
                }
            }
        }

        return sonarReadings;
    }

    void SendTelemetry()
    {
        if (!isInitialized) return;

        var sensors = ScanSonar();

        var sb = new StringBuilder(2048);
        sb.Append("{\"pos\":{\"x\":");
        sb.Append(rb.position.x.ToString("F3", CultureInfo.InvariantCulture));
        sb.Append(",\"y\":");
        sb.Append(rb.position.y.ToString("F3", CultureInfo.InvariantCulture));
        sb.Append(",\"z\":");
        sb.Append(rb.position.z.ToString("F3", CultureInfo.InvariantCulture));
        sb.Append("},\"rot\":{\"x\":");
        sb.Append(rb.rotation.eulerAngles.x.ToString("F3", CultureInfo.InvariantCulture));
        sb.Append(",\"y\":");
        sb.Append(rb.rotation.eulerAngles.y.ToString("F3", CultureInfo.InvariantCulture));
        sb.Append("},\"sensors\":[");

        for (int i = 0; i < sensors.Count; i++)
        {
            if (i > 0) sb.Append(",");
            sb.Append("{\"angle\":");
            sb.Append(sensors[i].angle.ToString("F1", CultureInfo.InvariantCulture));
            sb.Append(",\"dist\":");
            sb.Append(sensors[i].dist.ToString("F2", CultureInfo.InvariantCulture));
            sb.Append("}");
        }

        sb.Append("]}");

        byte[] data = Encoding.UTF8.GetBytes(sb.ToString());

        try
        {
            udpListener.Send(data, data.Length, new IPEndPoint(IPAddress.Parse("127.0.0.1"), sendPort));
        }
        catch (Exception e)
        {
            Debug.LogError($"Send error: {e.Message}");
        }
    }

    async Task ReceiveData(CancellationToken token)
    {
        while (!token.IsCancellationRequested && isInitialized)
        {
            try
            {
                var result = await udpListener.ReceiveAsync();
                string json = Encoding.UTF8.GetString(result.Buffer);
                ParseThrusterCommand(json);
            }
            catch (ObjectDisposedException)
            {
                break;
            }
            catch (SocketException ex) when (ex.SocketErrorCode == SocketError.Interrupted)
            {
                break;
            }
            catch (Exception e)
            {
                if (!token.IsCancellationRequested && isInitialized)
                {
                    Debug.LogWarning($"Network receive error: {e.Message}");
                }
                await Task.Delay(100);
            }
        }
    }

    void ParseThrusterCommand(string json)
    {
        try
        {
            int startIndex = json.IndexOf("[", StringComparison.Ordinal);
            int endIndex = json.IndexOf("]", StringComparison.Ordinal);

            if (startIndex == -1 || endIndex == -1 || endIndex <= startIndex)
            {
                return;
            }

            string arrayContent = json.Substring(startIndex + 1, endIndex - startIndex - 1);
            string[] parts = arrayContent.Split(',');

            if (parts.Length != 4)
            {
                return;
            }

            for (int i = 0; i < 4; i++)
            {
                thrusterForces[i] = float.Parse(parts[i].Trim(), CultureInfo.InvariantCulture);
            }
        }
        catch (Exception e)
        {
            Debug.LogWarning($"Failed to parse command: {e.Message}");
        }
    }

    async void OnDestroy()
    {
        if (cancellationTokenSource != null)
        {
            cancellationTokenSource.Cancel();
            if (receiveTask != null)
            {
                try
                {
                    await Task.WhenAny(receiveTask, Task.Delay(1000));
                }
                catch (Exception) { }
            }
            cancellationTokenSource.Dispose();
            cancellationTokenSource = null;
        }

        if (udpListener != null)
        {
            udpListener.Close();
            udpListener.Dispose();
            udpListener = null;
        }

        Debug.Log("AUV Simulator stopped.");
    }
}