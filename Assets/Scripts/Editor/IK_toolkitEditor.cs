using UnityEditor;
using UnityEngine;
using URRobot;

// Inspector for IK_toolkit: solver status plus a live table of the 8 analytic
// branches with per-joint angles and one-click branch selection.
[CustomEditor(typeof(IK_toolkit))]
public class IK_toolkitEditor : Editor
{
    static readonly string[] ColumnLabels =
        { "shoulder + / wrist + / elbow +", "shoulder + / wrist + / elbow -",
          "shoulder + / wrist - / elbow +", "shoulder + / wrist - / elbow -",
          "shoulder - / wrist + / elbow +", "shoulder - / wrist + / elbow -",
          "shoulder - / wrist - / elbow +", "shoulder - / wrist - / elbow -" };

    // The solver state is not serialized, so the table would freeze while the
    // target is dragged in the scene view unless the inspector keeps repainting.
    public override bool RequiresConstantRepaint() => ((IK_toolkit)target).HasSolved;

    public override void OnInspectorGUI()
    {
        DrawDefaultInspector();

        var toolkit = (IK_toolkit)target;
        EditorGUILayout.Space(10);
        EditorGUILayout.LabelField("Analytic solver", EditorStyles.boldLabel);

        if (!toolkit.HasSolved)
        {
            EditorGUILayout.HelpBox("Waiting for the first solve. Assign the IK target and the six joint transforms.", MessageType.Info);
            return;
        }
        if (toolkit.ValidCount == 0)
        {
            EditorGUILayout.HelpBox("Target out of reach: no solution. The robot holds its last pose.", MessageType.Warning);
            return;
        }
        if (toolkit.AppliedIndex < 0)
            EditorGUILayout.HelpBox($"Solution {toolkit.solutionID} is unavailable for this target. Pick a valid branch below, or switch to ClosestToCurrent.", MessageType.Info);
        else
            EditorGUILayout.LabelField($"{toolkit.ValidCount}/8 solutions valid - applied: {toolkit.AppliedIndex}");

        using (new EditorGUILayout.HorizontalScope())
        {
            GUILayout.Label("#", GUILayout.Width(18));
            for (int j = 1; j <= 6; j++)
                GUILayout.Label("θ" + j, EditorStyles.miniBoldLabel, GUILayout.Width(52));
            GUILayout.FlexibleSpace();
        }

        for (int c = 0; c < URKinematics.SolutionCount; c++)
        {
            using (new EditorGUILayout.HorizontalScope())
            {
                bool isApplied = c == toolkit.AppliedIndex;
                var rowStyle = isApplied ? EditorStyles.boldLabel : EditorStyles.label;
                GUILayout.Label(c.ToString(), rowStyle, GUILayout.Width(18));

                if (!toolkit.Valid[c])
                {
                    GUILayout.Label("unavailable", EditorStyles.centeredGreyMiniLabel, GUILayout.Width(6 * 52));
                }
                else
                {
                    for (int j = 0; j < 6; j++)
                    {
                        double deg = toolkit.Solutions[j, c] * Mathf.Rad2Deg;
                        GUILayout.Label(deg.ToString("F1") + "°", rowStyle, GUILayout.Width(52));
                    }
                }

                GUILayout.FlexibleSpace();
                using (new EditorGUI.DisabledScope(!toolkit.Valid[c] || isApplied))
                {
                    if (GUILayout.Button(isApplied ? "active" : "apply", EditorStyles.miniButton, GUILayout.Width(70)))
                    {
                        Undo.RecordObject(toolkit, "Select IK solution");
                        toolkit.solutionID = c;
                        toolkit.selectionMode = IK_toolkit.SolutionSelectionMode.Manual;
                        EditorUtility.SetDirty(toolkit);
                    }
                }
                if (toolkit.Valid[c])
                    GUILayout.Label(ColumnLabels[c], EditorStyles.centeredGreyMiniLabel, GUILayout.Width(190));
                else
                    GUILayout.Space(194);
            }
        }
    }
}
