using System.Collections.Generic;
using UnityEngine;
using URRobot;

// Drives the UR16e rig from the "ik" target using the analytic solver in
// URKinematics. The solver works in the robot's right-handed DH base frame;
// this component converts Unity's left-handed local pose with the same
// Y-mirror the original implementation used (M * T * M, M = diag(1,-1,1)).
[ExecuteAlways]
public class IK_toolkit : MonoBehaviour
{
    public enum SolutionSelectionMode
    {
        // Always apply the branch chosen by solutionID (0-7).
        Manual,
        // Apply the valid branch closest to the current joints, avoiding
        // configuration jumps while the target moves.
        ClosestToCurrent,
    }

    [Tooltip("Target the robot flange follows. Must stay a child of the robot root.")]
    public Transform ik;
    [Tooltip("Solution branch (0-7) applied in Manual mode.")]
    [Range(0, 7)] public int solutionID;
    public SolutionSelectionMode selectionMode = SolutionSelectionMode.Manual;
    [Tooltip("Joint transforms, in order: Base, Shoulder, Elbow, Wrist1, Wrist2, Wrist3.")]
    public List<Transform> robot = new List<Transform>();
    [Tooltip("Joint angles (radians) of the solution currently applied to the rig.")]
    public List<double> goodSolution = new List<double>();

    [Header("Gizmos")]
    public bool drawTargetFrame = true;
    public bool drawSolutionSkeletons = true;

    // Solver state, exposed read-only for the custom inspector and gizmos.
    readonly double[,] solutions = new double[6, URKinematics.SolutionCount];
    readonly bool[] valid = new bool[URKinematics.SolutionCount];
    readonly double[] appliedJoints = new double[6];
    readonly double[] fkJoints = new double[6];
    readonly Vector3[] skeletonPoints = new Vector3[7];
    int appliedIndex = -1;
    int validCount;
    bool hasSolved;

    public double[,] Solutions => solutions;
    public bool[] Valid => valid;
    public int AppliedIndex => appliedIndex;
    public int ValidCount => validCount;
    public bool HasSolved => hasSolved;

    // Dirty tracking: the solver only runs when the target or settings change.
    Vector3 lastTargetPosition;
    Quaternion lastTargetRotation;
    int lastSolutionID = -1;
    SolutionSelectionMode lastMode;
    bool needsSolve = true;

    public URKinematics.DHParameters DH => URKinematics.UR16e;

    void OnEnable()
    {
        // Seed the joint reference from the last serialized solution so
        // ClosestToCurrent and the wrist-singularity pin start without a jump.
        if (goodSolution != null && goodSolution.Count == 6)
            for (int i = 0; i < 6; i++)
                appliedJoints[i] = goodSolution[i];
        needsSolve = true;
    }

    void OnValidate() => needsSolve = true;

    void Update()
    {
        if (ik == null || robot == null || robot.Count != 6)
            return;
        for (int i = 0; i < 6; i++)
            if (robot[i] == null)
                return;

        Vector3 p = ik.localPosition;
        Quaternion q = ik.localRotation;
        // Exact component comparison, not Vector3/Quaternion ==: their approximate
        // equality would let the flange sit up to ~0.16 degrees off the target.
        if (!needsSolve && SameTarget(p, q)
            && solutionID == lastSolutionID && selectionMode == lastMode)
            return;
        lastTargetPosition = p;
        lastTargetRotation = q;
        lastSolutionID = solutionID;
        lastMode = selectionMode;
        needsSolve = false;

        RigidTransform target = TargetToRobotFrame(p, q);
        validCount = URKinematics.Solve(target, DH, solutions, valid,
                                        wristSingularTheta6: appliedJoints[5]);
        hasSolved = true;

        int index = selectionMode == SolutionSelectionMode.Manual
            ? Mathf.Clamp(solutionID, 0, URKinematics.SolutionCount - 1)
            : URKinematics.ClosestSolution(solutions, valid, appliedJoints);

        if (index < 0 || !valid[index])
        {
            // Unreachable (or the chosen branch is): keep the current pose.
            // The inspector reports the status; no per-frame error spam.
            appliedIndex = -1;
            return;
        }

        appliedIndex = index;
        for (int i = 0; i < 6; i++)
        {
            appliedJoints[i] = solutions[i, index];
            robot[i].localRotation = JointRotation(i, appliedJoints[i]);
        }

        if (goodSolution.Count != 6)
        {
            goodSolution.Clear();
            for (int i = 0; i < 6; i++)
                goodSolution.Add(appliedJoints[i]);
        }
        else
        {
            for (int i = 0; i < 6; i++)
                goodSolution[i] = appliedJoints[i];
        }
    }

    // Inverse of TargetToRobotFrame: a flange pose in the DH base frame becomes
    // the local pose the IK target must have under the robot root.
    public static void RobotFrameToTarget(in RigidTransform t, out Vector3 position, out Quaternion rotation)
    {
        position = new Vector3((float)t.px, -(float)t.py, (float)t.pz);
        // M * R * M with M = diag(1,-1,1): negate the entries with exactly one Y index
        var m = Matrix4x4.identity;
        m.m00 = (float)t.r00; m.m01 = -(float)t.r01; m.m02 = (float)t.r02;
        m.m10 = -(float)t.r10; m.m11 = (float)t.r11; m.m12 = -(float)t.r12;
        m.m20 = (float)t.r20; m.m21 = -(float)t.r21; m.m22 = (float)t.r22;
        rotation = m.rotation;
    }

    bool SameTarget(Vector3 p, Quaternion q) =>
        p.x == lastTargetPosition.x && p.y == lastTargetPosition.y && p.z == lastTargetPosition.z &&
        q.x == lastTargetRotation.x && q.y == lastTargetRotation.y &&
        q.z == lastTargetRotation.z && q.w == lastTargetRotation.w;

    // Unity local pose (left-handed) -> DH base frame (right-handed), via the
    // Y-mirror conjugation M*T*M: flip the sign of the rotation elements with
    // exactly one index on the Y row/column, and of the Y translation.
    public static RigidTransform TargetToRobotFrame(Vector3 p, Quaternion q)
    {
        double x = q.x, y = q.y, z = q.z, w = q.w;
        return new RigidTransform
        {
            r00 = 1 - 2 * (y * y + z * z), r01 = -2 * (x * y - z * w), r02 = 2 * (x * z + y * w), px = p.x,
            r10 = -2 * (x * y + z * w), r11 = 1 - 2 * (x * x + z * z), r12 = -2 * (y * z - x * w), py = -p.y,
            r20 = 2 * (x * z - y * w), r21 = -2 * (y * z + x * w), r22 = 1 - 2 * (x * x + y * y), pz = p.z,
        };
    }

    // Maps a DH joint angle onto the rig: rotation about the local Z axis
    // (negated for the mirrored world), plus the fixed mounting offset of the
    // UR_16 FBX links (Shoulder/Wrist2 tilted -90 deg, Wrist3 +90 deg).
    public static Quaternion JointRotation(int jointIndex, double angleRad)
    {
        float angleDeg = -(float)(Mathf.Rad2Deg * angleRad);
        switch (jointIndex)
        {
            case 1:
            case 4:
                return Quaternion.Euler(-90, 0, angleDeg);
            case 5:
                return Quaternion.Euler(90, 0, angleDeg);
            default:
                return Quaternion.Euler(0, 0, angleDeg);
        }
    }

#if UNITY_EDITOR
    static readonly Color AppliedColor = new Color(0.2f, 1f, 0.5f, 0.9f);
    static readonly Color CandidateColor = new Color(1f, 1f, 1f, 0.22f);
    static readonly Color UnreachableColor = new Color(1f, 0.3f, 0.25f, 0.9f);

    void OnDrawGizmos()
    {
        // Unity calls this even when the component is unchecked, which would keep
        // drawing a stale skeleton from the last solve.
        if (!isActiveAndEnabled || ik == null)
            return;

        if (drawTargetFrame)
        {
            float s = 0.12f * ik.lossyScale.magnitude;
            Vector3 o = ik.position;
            if (hasSolved && validCount == 0)
            {
                Gizmos.color = UnreachableColor;
                Gizmos.DrawWireSphere(o, 0.35f * s);
            }
            Gizmos.color = Color.red;
            Gizmos.DrawLine(o, o + ik.right * s);
            Gizmos.color = Color.green;
            Gizmos.DrawLine(o, o + ik.up * s);
            Gizmos.color = Color.blue;
            Gizmos.DrawLine(o, o + ik.forward * s);
        }

        if (!drawSolutionSkeletons || !hasSolved)
            return;
        Transform baseFrame = ik.parent != null ? ik.parent : transform;
        for (int c = 0; c < URKinematics.SolutionCount; c++)
        {
            if (!valid[c] || c == appliedIndex)
                continue;
            DrawSkeleton(baseFrame, c, CandidateColor, 0.006f);
        }
        if (appliedIndex >= 0)
            DrawSkeleton(baseFrame, appliedIndex, AppliedColor, 0.012f);
    }

    void DrawSkeleton(Transform baseFrame, int column, Color color, float jointRadius)
    {
        for (int i = 0; i < 6; i++)
            fkJoints[i] = solutions[i, column];

        skeletonPoints[0] = baseFrame.TransformPoint(Vector3.zero);
        var t = RigidTransform.Identity;
        for (int i = 0; i < 6; i++)
        {
            t = t * URKinematics.LinkTransform(DH, i, fkJoints[i]);
            // robot frame -> Unity local frame: undo the Y-mirror
            skeletonPoints[i + 1] = baseFrame.TransformPoint(
                new Vector3((float)t.px, -(float)t.py, (float)t.pz));
        }

        Gizmos.color = color;
        for (int i = 0; i < 7; i++)
        {
            if (i < 6)
                Gizmos.DrawLine(skeletonPoints[i], skeletonPoints[i + 1]);
            Gizmos.DrawSphere(skeletonPoints[i], jointRadius * baseFrame.lossyScale.magnitude);
        }
    }
#endif
}
