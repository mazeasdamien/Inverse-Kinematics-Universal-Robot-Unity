using System.Collections.Generic;
using UnityEngine;
using URRobot;

// Runtime control for the IK targets of every robot in the scene.
//
// Mouse: click a robot (or its target) to select it, then drag to move the
// target in the camera plane; the scroll wheel pushes it towards or away from
// the camera. Keyboard: rotate the flange, cycle the solution branch, switch
// robot, toggle a demo orbit. An on-screen legend lists the bindings.
[RequireComponent(typeof(Camera))]
public class IKTargetController : MonoBehaviour
{
    [Tooltip("Robots controlled by this camera. Leave empty to pick them all up on Start.")]
    public List<IK_toolkit> robots = new List<IK_toolkit>();

    [Header("Interaction")]
    [Tooltip("Metres per scroll notch when pushing the target along the view direction.")]
    public float scrollSpeed = 0.25f;
    [Tooltip("Degrees per second when rotating the flange with the keyboard.")]
    public float rotationSpeed = 90f;
    [Tooltip("Radius of the clickable handle drawn at each target, in metres.")]
    public float handleRadius = 0.06f;

    [Header("Demo orbit")]
    [Tooltip("Sweep every target along a Lissajous path so the scene animates itself.")]
    public bool demoOrbit;
    public float demoSpeed = 0.35f;

    [Header("On-screen legend")]
    public bool showLegend = true;

    int selected;
    Camera cam;
    Plane dragPlane;
    Vector3 dragOffset;
    bool dragging;

    readonly List<Vector3> restPositions = new List<Vector3>();
    readonly List<Quaternion> restRotations = new List<Quaternion>();
    float demoTime;

    void Awake()
    {
        cam = GetComponent<Camera>();
        if (robots.Count == 0)
            robots.AddRange(FindObjectsByType<IK_toolkit>(FindObjectsInactive.Exclude));

        foreach (var r in robots)
        {
            restPositions.Add(r != null && r.ik != null ? r.ik.localPosition : Vector3.zero);
            restRotations.Add(r != null && r.ik != null ? r.ik.localRotation : Quaternion.identity);
        }
    }

    IK_toolkit Current => robots.Count > 0 ? robots[Mathf.Clamp(selected, 0, robots.Count - 1)] : null;

    void Update()
    {
        if (robots.Count == 0)
            return;

        if (demoOrbit)
        {
            UpdateDemoOrbit();
            return;
        }

        HandleSelection();
        HandleDrag();
        HandleKeyboard();
    }

    void HandleSelection()
    {
        if (!Input.GetMouseButtonDown(0))
            return;

        // Pick the target whose handle is nearest to the click, in screen space.
        Vector2 mouse = Input.mousePosition;
        int best = -1;
        float bestDistance = float.MaxValue;
        for (int i = 0; i < robots.Count; i++)
        {
            if (robots[i] == null || robots[i].ik == null)
                continue;
            Vector3 screen = cam.WorldToScreenPoint(robots[i].ik.position);
            if (screen.z <= 0)
                continue;
            float d = Vector2.Distance(mouse, screen);
            if (d < bestDistance)
            {
                bestDistance = d;
                best = i;
            }
        }
        if (best < 0)
            return;

        // Selecting is generous; starting a drag requires hitting the handle.
        selected = best;
        Transform target = robots[best].ik;
        Vector3 targetScreen = cam.WorldToScreenPoint(target.position);
        float handlePixels = HandlePixelRadius(target.position);
        if (Vector2.Distance(mouse, targetScreen) > handlePixels)
            return;

        dragPlane = new Plane(-cam.transform.forward, target.position);
        if (dragPlane.Raycast(cam.ScreenPointToRay(mouse), out float enter))
        {
            dragOffset = target.position - cam.ScreenPointToRay(mouse).GetPoint(enter);
            dragging = true;
        }
    }

    void HandleDrag()
    {
        if (dragging && Input.GetMouseButton(0))
        {
            var target = Current != null ? Current.ik : null;
            if (target == null)
                return;
            Ray ray = cam.ScreenPointToRay(Input.mousePosition);
            if (dragPlane.Raycast(ray, out float enter))
                target.position = ray.GetPoint(enter) + dragOffset;
        }
        else
        {
            dragging = false;
        }

        float scroll = Input.GetAxis("Mouse ScrollWheel");
        if (Mathf.Abs(scroll) > 0.0001f && Current != null && Current.ik != null)
        {
            Current.ik.position += cam.transform.forward * (scroll * scrollSpeed * 10f);
            // keep the drag plane anchored to the new depth
            dragPlane = new Plane(-cam.transform.forward, Current.ik.position);
        }
    }

    void HandleKeyboard()
    {
        var toolkit = Current;
        if (toolkit == null || toolkit.ik == null)
            return;

        if (Input.GetKeyDown(KeyCode.Tab))
            selected = (selected + 1) % robots.Count;

        if (Input.GetKeyDown(KeyCode.Space))
        {
            toolkit.selectionMode = toolkit.selectionMode == IK_toolkit.SolutionSelectionMode.Manual
                ? IK_toolkit.SolutionSelectionMode.ClosestToCurrent
                : IK_toolkit.SolutionSelectionMode.Manual;
        }

        if (Input.GetKeyDown(KeyCode.PageUp))
            toolkit.solutionID = (toolkit.solutionID + 1) % URKinematics.SolutionCount;
        if (Input.GetKeyDown(KeyCode.PageDown))
            toolkit.solutionID = (toolkit.solutionID + URKinematics.SolutionCount - 1) % URKinematics.SolutionCount;
        for (int i = 0; i < URKinematics.SolutionCount; i++)
            if (Input.GetKeyDown(KeyCode.Alpha1 + i))
                toolkit.solutionID = i;

        float step = rotationSpeed * Time.deltaTime;
        if (Input.GetKey(KeyCode.Q)) toolkit.ik.Rotate(Vector3.up, -step, Space.World);
        if (Input.GetKey(KeyCode.E)) toolkit.ik.Rotate(Vector3.up, step, Space.World);
        if (Input.GetKey(KeyCode.Z)) toolkit.ik.Rotate(Vector3.right, -step, Space.World);
        if (Input.GetKey(KeyCode.C)) toolkit.ik.Rotate(Vector3.right, step, Space.World);

        if (Input.GetKeyDown(KeyCode.R))
            ResetTargets();
        if (Input.GetKeyDown(KeyCode.O))
            demoOrbit = !demoOrbit;
        if (Input.GetKeyDown(KeyCode.H))
            showLegend = !showLegend;
    }

    void UpdateDemoOrbit()
    {
        demoTime += Time.deltaTime * demoSpeed;
        for (int i = 0; i < robots.Count; i++)
        {
            var toolkit = robots[i];
            if (toolkit == null || toolkit.ik == null || i >= restPositions.Count)
                continue;

            // Amplitude scales with the arm so every robot sweeps a similar
            // fraction of its own workspace.
            float span = (float)URKinematics.Reach(URKinematics.UR16e) * 0.18f;
            float phase = demoTime + i * 2.1f;
            Vector3 offset = new Vector3(
                Mathf.Sin(phase) * span,
                Mathf.Sin(phase * 1.3f) * span * 0.6f,
                Mathf.Cos(phase * 0.7f) * span);
            toolkit.ik.localPosition = restPositions[i] + offset;
            toolkit.ik.localRotation = restRotations[i] * Quaternion.Euler(0, Mathf.Sin(phase * 0.9f) * 25f, 0);
        }

        if (Input.GetKeyDown(KeyCode.O))
            demoOrbit = false;
        if (Input.GetKeyDown(KeyCode.R))
            ResetTargets();
        if (Input.GetKeyDown(KeyCode.H))
            showLegend = !showLegend;
    }

    void ResetTargets()
    {
        for (int i = 0; i < robots.Count && i < restPositions.Count; i++)
        {
            if (robots[i] == null || robots[i].ik == null)
                continue;
            robots[i].ik.localPosition = restPositions[i];
            robots[i].ik.localRotation = restRotations[i];
        }
    }

    float HandlePixelRadius(Vector3 worldPosition)
    {
        Vector3 a = cam.WorldToScreenPoint(worldPosition);
        Vector3 b = cam.WorldToScreenPoint(worldPosition + cam.transform.up * handleRadius);
        return Mathf.Max(18f, Vector2.Distance(a, b));
    }

    void OnGUI()
    {
        if (!showLegend)
            return;

        var toolkit = Current;
        var style = new GUIStyle(GUI.skin.label) { fontSize = 13, richText = true };
        var box = new GUIStyle(GUI.skin.box) { alignment = TextAnchor.UpperLeft, padding = new RectOffset(12, 12, 10, 10) };

        string status = "no robot in scene";
        if (toolkit != null)
        {
            string reach = $"{URKinematics.Reach(URKinematics.UR16e) * 1000f:F0} mm reach";
            status = toolkit.ValidCount == 0
                ? $"<b>{toolkit.name}</b>  ({reach})\ntarget out of reach"
                : $"<b>{toolkit.name}</b>  ({reach})\n{toolkit.ValidCount}/8 solutions - branch {toolkit.AppliedIndex} - {toolkit.selectionMode}";
        }

        GUI.Box(new Rect(12, 12, 330, 210), GUIContent.none, box);
        GUILayout.BeginArea(new Rect(24, 22, 310, 195));
        GUILayout.Label(status, style);
        GUILayout.Space(6);
        GUILayout.Label(
            "<b>Drag</b> the green handle to move the target\n" +
            "<b>Scroll</b>  push the target in depth\n" +
            "<b>Tab</b>  next robot        <b>1-8</b>  solution branch\n" +
            "<b>Q / E</b>  yaw             <b>Z / C</b>  pitch\n" +
            "<b>Space</b>  manual / closest-to-current\n" +
            "<b>O</b>  demo orbit          <b>R</b>  reset\n" +
            "<b>H</b>  hide this panel", style);
        GUILayout.EndArea();
    }

    void OnDrawGizmos()
    {
        if (robots == null)
            return;
        for (int i = 0; i < robots.Count; i++)
        {
            if (robots[i] == null || robots[i].ik == null)
                continue;
            Gizmos.color = i == selected ? new Color(0.2f, 1f, 0.5f) : new Color(1f, 1f, 1f, 0.35f);
            Gizmos.DrawWireSphere(robots[i].ik.position, handleRadius);
        }
    }
}
