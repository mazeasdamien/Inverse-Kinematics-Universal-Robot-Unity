using System.IO;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;
using URRobot;

// Renders the README images from the main scene without opening the editor UI:
//   Unity -batchmode -quit -projectPath . -executeMethod ScreenshotCapture.CaptureAll
// Output goes to Docs/ as PNG.
//
// The rig is driven straight through URKinematics rather than through the
// IK_toolkit component: a -batchmode session does not run Update, and it does
// not always resolve Assembly-CSharp MonoBehaviours loaded from a scene file.
public static class ScreenshotCapture
{
    const int Width = 1800;
    const int Height = 1000;
    const string OutputDir = "Docs";
    const string RobotRootName = "UR_16e";
    const string TargetName = "IK- MOVE THIS GAMEOBJECT";

    static readonly string[] JointNames =
        { "Base", "Shoulder", "Elbow", "Wrist1", "Wrist2", "Wrist3" };

    [MenuItem("Tools/UR16e/Capture README screenshots")]
    public static void CaptureAll()
    {
        var scene = EditorSceneManager.OpenScene("Assets/Scenes/main.unity", OpenSceneMode.Single);
        Directory.CreateDirectory(OutputDir);

        Transform root = Find(scene, RobotRootName);
        Transform target = Find(scene, TargetName);
        if (root == null || target == null)
        {
            Debug.LogError($"scene is missing '{RobotRootName}' or '{TargetName}'");
            return;
        }

        var joints = new Transform[6];
        for (int i = 0; i < 6; i++)
        {
            joints[i] = FindDescendant(root, JointNames[i]);
            if (joints[i] == null)
            {
                Debug.LogError("missing joint transform: " + JointNames[i]);
                return;
            }
        }

        var cam = Object.FindAnyObjectByType<Camera>();
        cam.clearFlags = CameraClearFlags.SolidColor;
        cam.backgroundColor = new Color(0.075f, 0.082f, 0.098f);
        cam.allowMSAA = true;

        var solutions = new double[6, URKinematics.SolutionCount];
        var valid = new bool[URKinematics.SolutionCount];

        // 1. Hero shot: the arm reaching for the target.
        int applied = SolveAndApply(target, joints, solutions, valid, 5);
        FrameRobot(cam, root, yaw: -34f, pitch: 14f, fov: 40f, margin: 1.25f);
        Capture(cam, "hero.png");

        // 2. Same target, a different branch of the same solution set.
        int alternate = FirstValidOtherThan(valid, applied);
        SolveAndApply(target, joints, solutions, valid, alternate);
        FrameRobot(cam, root, yaw: -34f, pitch: 14f, fov: 40f, margin: 1.25f);
        Capture(cam, "branch.png");

        // 3. The eight analytic branches drawn over the arm, framed on the union
        // of the arm and every candidate skeleton so nothing is cropped.
        Vector3[][] chains = CollectChains(target, joints, solutions, valid);
        SolveAndApply(target, joints, solutions, valid, applied);
        Bounds shot = RobotBounds(root);
        foreach (var chain in chains)
        {
            if (chain == null) continue;
            foreach (var p in chain) shot.Encapsulate(p);
        }
        FrameBounds(cam, shot, yaw: -34f, pitch: 12f, fov: 40f, margin: 1.15f);
        CaptureWithSkeletons(cam, chains, valid, applied, "solutions.png");

        Debug.Log("screenshots written to " + Path.GetFullPath(OutputDir));
    }

    // Runs the analytic solver for the current target pose and poses the rig with
    // the requested branch (or the closest valid one). Returns the applied branch.
    static int SolveAndApply(Transform target, Transform[] joints,
                             double[,] solutions, bool[] valid, int branch)
    {
        RigidTransform pose = IK_toolkit.TargetToRobotFrame(target.localPosition, target.localRotation);
        int count = URKinematics.Solve(pose, URKinematics.UR16e, solutions, valid);
        if (count == 0)
        {
            Debug.LogWarning("target out of reach; rig left as-is");
            return -1;
        }
        if (branch < 0 || !valid[branch])
            branch = FirstValidOtherThan(valid, -1);

        for (int i = 0; i < 6; i++)
            joints[i].localRotation = IK_toolkit.JointRotation(i, solutions[i, branch]);
        return branch;
    }

    static int FirstValidOtherThan(bool[] valid, int exclude)
    {
        for (int c = 0; c < valid.Length; c++)
            if (valid[c] && c != exclude)
                return c;
        return exclude;
    }

    static Transform Find(UnityEngine.SceneManagement.Scene scene, string name)
    {
        foreach (var go in scene.GetRootGameObjects())
        {
            if (go.name == name)
                return go.transform;
            Transform hit = FindDescendant(go.transform, name);
            if (hit != null)
                return hit;
        }
        return null;
    }

    static Transform FindDescendant(Transform parent, string name)
    {
        foreach (Transform child in parent)
        {
            if (child.name == name)
                return child;
            Transform hit = FindDescendant(child, name);
            if (hit != null)
                return hit;
        }
        return null;
    }

    // Frames the whole robot from the renderer bounds, so the shot stays correct
    // whatever pose the arm is in and whatever scale the rig root carries.
    static void FrameRobot(Camera cam, Transform root, float yaw, float pitch, float fov, float margin)
        => FrameBounds(cam, RobotBounds(root), yaw, pitch, fov, margin);

    static Bounds RobotBounds(Transform root)
    {
        var renderers = root.GetComponentsInChildren<Renderer>();
        if (renderers.Length == 0)
            return new Bounds(root.position, Vector3.one);
        Bounds bounds = renderers[0].bounds;
        for (int i = 1; i < renderers.Length; i++)
            bounds.Encapsulate(renderers[i].bounds);
        return bounds;
    }

    static void FrameBounds(Camera cam, Bounds bounds, float yaw, float pitch, float fov, float margin)
    {
        // Fit the bounding sphere in the narrower of the two frustum angles.
        // The aspect must be forced: off-screen rendering does not inherit it.
        cam.aspect = (float)Width / Height;
        float radius = bounds.extents.magnitude * margin;
        float vertical = fov * Mathf.Deg2Rad * 0.5f;
        float distance = radius / Mathf.Sin(vertical);

        Quaternion rotation = Quaternion.Euler(pitch, yaw, 0f);
        cam.fieldOfView = fov;
        cam.transform.rotation = rotation;
        cam.transform.position = bounds.center - rotation * Vector3.forward * distance;
        cam.nearClipPlane = Mathf.Max(0.05f, distance * 0.05f);
        cam.farClipPlane = distance * 6f;
    }

    static Texture2D Render(Camera cam)
    {
        var rt = new RenderTexture(Width, Height, 24, RenderTextureFormat.ARGB32) { antiAliasing = 8 };
        cam.targetTexture = rt;
        cam.aspect = (float)Width / Height;
        cam.Render();
        RenderTexture.active = rt;
        var tex = new Texture2D(Width, Height, TextureFormat.RGB24, false);
        tex.ReadPixels(new Rect(0, 0, Width, Height), 0, 0);
        RenderTexture.active = null;
        cam.targetTexture = null;
        Object.DestroyImmediate(rt);
        return tex;
    }

    static void Save(Texture2D tex, string fileName)
    {
        tex.Apply();
        File.WriteAllBytes(Path.Combine(OutputDir, fileName), tex.EncodeToPNG());
        Object.DestroyImmediate(tex);
        Debug.Log("wrote " + fileName);
    }

    static void Capture(Camera cam, string fileName) => Save(Render(cam), fileName);

    // World-space joint positions for each valid branch, read back from the rig
    // itself: posing the real transforms is the only way the overlay is
    // guaranteed to line up with what the camera renders.
    static Vector3[][] CollectChains(Transform target, Transform[] joints,
                                     double[,] solutions, bool[] valid)
    {
        var chains = new Vector3[URKinematics.SolutionCount][];
        for (int c = 0; c < URKinematics.SolutionCount; c++)
        {
            if (!valid[c])
                continue;
            for (int i = 0; i < 6; i++)
                joints[i].localRotation = IK_toolkit.JointRotation(i, solutions[i, c]);

            var chain = new Vector3[8];
            for (int i = 0; i < 6; i++)
                chain[i] = joints[i].position;
            chain[6] = joints[5].position;
            chain[7] = target.position;   // every branch converges on the flange
            chains[c] = chain;
        }
        return chains;
    }

    // Renders the scene, then overlays every valid solution as a stick figure so
    // the README can show the eight-branch structure in one image.
    static void CaptureWithSkeletons(Camera cam, Vector3[][] chains, bool[] valid,
                                     int applied, string fileName)
    {
        var tex = Render(cam);
        var appliedColor = new Color(0.25f, 1f, 0.55f);
        var candidateColor = new Color(0.45f, 0.72f, 1f);

        for (int c = 0; c < URKinematics.SolutionCount; c++)
        {
            if (!valid[c] || c == applied || chains[c] == null) continue;
            DrawSkeleton(tex, cam, chains[c], candidateColor, 2);
        }
        if (applied >= 0 && chains[applied] != null)
            DrawSkeleton(tex, cam, chains[applied], appliedColor, 4);

        Save(tex, fileName);
    }

    // Camera.WorldToScreenPoint maps into the *screen* rectangle, which in batch
    // mode has nothing to do with the render texture we are drawing into. Project
    // through the camera matrices instead and scale to the texture ourselves.
    // Texture space and NDC both put the origin at the bottom left.
    static bool ProjectToTexture(Camera cam, Vector3 world, out Vector2 pixel)
    {
        Matrix4x4 viewProjection = cam.projectionMatrix * cam.worldToCameraMatrix;
        Vector4 clip = viewProjection * new Vector4(world.x, world.y, world.z, 1f);
        if (clip.w <= 1e-5f)
        {
            pixel = default;
            return false;
        }
        pixel = new Vector2(
            (clip.x / clip.w * 0.5f + 0.5f) * Width,
            (clip.y / clip.w * 0.5f + 0.5f) * Height);
        return true;
    }

    static void DrawSkeleton(Texture2D tex, Camera cam, Vector3[] pts, Color color, int thickness)
    {
        for (int i = 0; i < pts.Length - 1; i++)
        {
            if (ProjectToTexture(cam, pts[i], out Vector2 a) &&
                ProjectToTexture(cam, pts[i + 1], out Vector2 b))
                DrawLine(tex, a, b, color, thickness);
        }
        foreach (var p in pts)
        {
            if (ProjectToTexture(cam, p, out Vector2 s))
                DrawDisc(tex, (int)s.x, (int)s.y, thickness + 2, color);
        }
    }

    static void DrawLine(Texture2D tex, Vector2 a, Vector2 b, Color color, int thickness)
    {
        int steps = Mathf.CeilToInt(Vector2.Distance(a, b));
        for (int i = 0; i <= steps; i++)
        {
            Vector2 p = Vector2.Lerp(a, b, steps == 0 ? 0 : i / (float)steps);
            DrawDisc(tex, (int)p.x, (int)p.y, thickness, color);
        }
    }

    static void DrawDisc(Texture2D tex, int cx, int cy, int radius, Color color)
    {
        for (int dy = -radius; dy <= radius; dy++)
            for (int dx = -radius; dx <= radius; dx++)
            {
                if (dx * dx + dy * dy > radius * radius) continue;
                int x = cx + dx, y = cy + dy;
                if (x < 0 || y < 0 || x >= tex.width || y >= tex.height) continue;
                tex.SetPixel(x, y, color);
            }
    }
}
