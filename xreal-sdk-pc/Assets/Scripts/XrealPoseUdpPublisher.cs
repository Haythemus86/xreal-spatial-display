using System.Globalization;
using System.Net.Sockets;
using UnityEngine;
using UnityEngine.XR;
using UnityEngine.XR.Management;

/// <summary>
/// Starts the configured XREAL XR loader and publishes its latest head pose
/// to the native application over loopback UDP.
/// </summary>
public sealed class XrealPoseUdpPublisher : MonoBehaviour
{
    private const int DestinationPort = 45871;
    private static readonly CultureInfo Invariant = CultureInfo.InvariantCulture;

    private UdpClient _udp;
    private InputDevice _head;
    private bool _ownsLoader;
    private bool _originCaptured;
    private Vector3 _originPosition;
    private Quaternion _originRotation = Quaternion.identity;
    private float _nextSendTime;
    private uint _sequence;

    private void Awake()
    {
        _udp = new UdpClient();
        _udp.Connect("127.0.0.1", DestinationPort);

        XRManagerSettings manager = XRGeneralSettings.Instance != null
            ? XRGeneralSettings.Instance.Manager
            : null;
        if (manager == null)
        {
            Debug.LogError("XREAL XR Manager is not configured for Standalone.");
            enabled = false;
            return;
        }

        if (manager.activeLoader == null)
        {
            manager.InitializeLoaderSync();
            _ownsLoader = manager.activeLoader != null;
        }

        if (manager.activeLoader == null)
        {
            Debug.LogError("XREAL XR Loader failed to initialize.");
            enabled = false;
            return;
        }

        if (_ownsLoader)
        {
            manager.StartSubsystems();
        }

        _head = InputDevices.GetDeviceAtXRNode(XRNode.Head);
        Debug.Log("XREAL pose publisher started; waiting for head tracking.");
    }

    private void Update()
    {
        if (!_head.isValid)
        {
            _head = InputDevices.GetDeviceAtXRNode(XRNode.Head);
        }

        bool valid = _head.isValid
            && _head.TryGetFeatureValue(CommonUsages.devicePosition, out Vector3 position)
            && _head.TryGetFeatureValue(CommonUsages.deviceRotation, out Quaternion rotation)
            && IsFinite(position) && IsFinite(rotation);

        if (valid)
        {
            if (!_originCaptured || Input.GetKeyDown(KeyCode.R))
            {
                _originPosition = position;
                _originRotation = rotation;
                _originCaptured = true;
            }

            if (Time.unscaledTime >= _nextSendTime)
            {
                _nextSendTime = Time.unscaledTime + (1.0f / 120.0f);
                Vector3 relativePosition = Quaternion.Inverse(_originRotation)
                    * (position - _originPosition);
                Quaternion relativeRotation = Quaternion.Inverse(_originRotation) * rotation;
                SendPose(true, relativePosition, relativeRotation);
            }
        }
        else if (Time.unscaledTime >= _nextSendTime)
        {
            _nextSendTime = Time.unscaledTime + (1.0f / 120.0f);
            SendPose(false, Vector3.zero, Quaternion.identity);
        }
    }

    private void SendPose(bool valid, Vector3 position, Quaternion rotation)
    {
        string packet = string.Format(Invariant,
            "1,{0},{1},{2:R},{3:R},{4:R},{5:R},{6:R},{7:R},{8:R},{9:R}",
            _sequence++, valid ? 1 : 0, Time.realtimeSinceStartupAsDouble,
            position.x, position.y, position.z,
            rotation.x, rotation.y, rotation.z, rotation.w);

        try
        {
            byte[] bytes = System.Text.Encoding.ASCII.GetBytes(packet);
            _udp.Send(bytes, bytes.Length);
        }
        catch (SocketException exception)
        {
            Debug.LogWarning("Pose UDP send failed: " + exception.Message);
        }
    }

    private static bool IsFinite(Vector3 value) =>
        IsFinite(value.x) && IsFinite(value.y) && IsFinite(value.z);

    private static bool IsFinite(Quaternion value) =>
        IsFinite(value.x) && IsFinite(value.y) && IsFinite(value.z) && IsFinite(value.w);

    private static bool IsFinite(float value) =>
        !float.IsNaN(value) && !float.IsInfinity(value);

    private void OnDestroy()
    {
        _udp?.Close();

        if (_ownsLoader && XRGeneralSettings.Instance != null)
        {
            XRManagerSettings manager = XRGeneralSettings.Instance.Manager;
            manager.StopSubsystems();
            manager.DeinitializeLoader();
        }
    }
}
