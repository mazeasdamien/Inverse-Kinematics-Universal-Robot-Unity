using System;
using System.Collections.Generic;
using UnityEngine;

// IK_toolkit: Inverse Kinematics Toolkit for UR16e robot arm
[ExecuteInEditMode]
public class IK_toolkit : MonoBehaviour
{
    public Transform ik;
    public int solutionID;
    private List<string> IK_Solutions = new List<string>();
    public List<double> goodSolution = new List<double>();
    public List<Transform> robot = new List<Transform>();

    // UR16e robot arm Denavit-Hartenberg parameters matrix
    public static double[,] DH_matrix_UR16e = new double[6, 3] {
        { 0, Mathf.PI / 2.0, 0.1807 },
        { -0.4784, 0, 0 },
        { -0.36, 0, 0 },
        { 0, Mathf.PI / 2.0, 0.17415 },
        { 0, -Mathf.PI / 2.0, 0.11985},
        { 0, 0,0.11655}};

    // Update is called once per frame
    void Update()
    {
        // Calculate the transformation matrix for the IK
        Matrix4x4 transform_matrix = GetTransformMatrix(ik);

        // Reflect the matrix along the Y-axis
        Matrix4x4 mt = Matrix4x4.identity;
        mt.m11 = -1;
        Matrix4x4 mt_inverse = mt.inverse;
        Matrix4x4 result = mt * transform_matrix * mt_inverse;

        // Compute the inverse kinematics solutions
        double[,] solutions = Inverse_kinematic_solutions(result);
        IK_Solutions.Clear();
        IK_Solutions = DisplaySolutions(solutions);

        // Set the robot arm joints based on the selected solution
        ApplyJointSolution(IK_Solutions, solutions, solutionID, robot);
        goodSolution.Clear();
        goodSolution.Add(solutions[0, 5]);
        goodSolution.Add(solutions[1, 5]);
        goodSolution.Add(solutions[2, 5]);
        goodSolution.Add(solutions[3, 5]);
        goodSolution.Add(solutions[4, 5]);
        goodSolution.Add(solutions[5, 5]);
    }

    // Get the transformation matrix for the given transform
    public static Matrix4x4 GetTransformMatrix(Transform controller)
    {
        return Matrix4x4.TRS(new Vector3(controller.localPosition.x, controller.localPosition.y, controller.localPosition.z), Quaternion.Euler(controller.localEulerAngles.x, controller.localEulerAngles.y, controller.localEulerAngles.z), new Vector3(1, 1, 1));
    }

    // Compute the transformation matrix using the Denavit-Hartenberg parameters
    public static Matrix4x4 ComputeTransformMatrix(int jointIndex, double[,] jointAngles)
    {
        jointIndex--;

        // Rotation around the Z-axis
        var rotationZ = Matrix4x4.identity;
        rotationZ.m00 = Mathf.Cos((float)jointAngles[0, jointIndex]);
        rotationZ.m01 = -Mathf.Sin((float)jointAngles[0, jointIndex]);
        rotationZ.m10 = Mathf.Sin((float)jointAngles[0, jointIndex]);
        rotationZ.m11 = Mathf.Cos((float)jointAngles[0, jointIndex]);

        // Translation along the Z-axis
        var translationZ = Matrix4x4.identity;
        translationZ.m23 = (float)DH_matrix_UR16e[jointIndex, 2];

        // Translation along the X-axis
        var translationX = Matrix4x4.identity;
        translationX.m03 = (float)DH_matrix_UR16e[jointIndex, 0];

        // Rotation around the X-axis
        var rotationX = Matrix4x4.identity;
        rotationX.m11 = Mathf.Cos((float)DH_matrix_UR16e[jointIndex, 1]);
        rotationX.m12 = -Mathf.Sin((float)DH_matrix_UR16e[jointIndex, 1]);
        rotationX.m21 = Mathf.Sin((float)DH_matrix_UR16e[jointIndex, 1]);
        rotationX.m22 = Mathf.Cos((float)DH_matrix_UR16e[jointIndex, 1]);

        // Combine the transformations in the following order: rotationZ, translationZ, translationX, and rotationX
        return rotationZ * translationZ * translationX * rotationX;
    }

    // Apply the inverse kinematics solution to the robot arm joints
    public static void ApplyJointSolution(List<string> solutionStatus, double[,] jointSolutions, int solutionIndex, List<Transform> robotJoints)
    {
        // Check if the solution is available
        if (solutionStatus[solutionIndex] != "NON DISPONIBLE")
        {
            // Iterate through each joint in the robot and apply the joint angles
            for (int i = 0; i < robotJoints.Count; i++)
            {
                robotJoints[i].localEulerAngles = ConvertJointAngles(jointSolutions[i, solutionIndex], i);
            }
        }
        else
        {
            // If no solution is available, log an error message
            Debug.LogError("NO SOLUTION");
        }
    }

    // Convert joint angles from radians to degrees and apply the appropriate offset
    private static Vector3 ConvertJointAngles(double angleRad, int jointIndex)
    {
        float angleDeg = -(float)(Mathf.Rad2Deg * angleRad);

        switch (jointIndex)
        {
            case 1:
                return new Vector3(-90, 0, angleDeg);
            case 4:
                return new Vector3(-90, 0, angleDeg);
            case 5:
                return new Vector3(90, 0, angleDeg);
            default:
                return new Vector3(0, 0, angleDeg);
        }
    }

    // Calculate the inverse kinematics solutions
    public static double[,] Inverse_kinematic_solutions(Matrix4x4 transform_matrix_unity)
    {

        double[,] theta = new double[6, 8];

        Vector4 P05 = transform_matrix_unity * new Vector4()
        {
            x = 0,
            y = 0,
            z = -(float)DH_matrix_UR16e[5, 2],
            w = 1
        }; ;
        float psi = Mathf.Atan2(P05[1], P05[0]);
        float phi = Mathf.Acos((float)((DH_matrix_UR16e[1, 2] + DH_matrix_UR16e[3, 2] + DH_matrix_UR16e[2, 2]) / Mathf.Sqrt(Mathf.Pow(P05[0], 2) + Mathf.Pow(P05[1], 2))));

        theta[0, 0] = psi + phi + Mathf.PI / 2;
        theta[0, 1] = psi + phi + Mathf.PI / 2;
        theta[0, 2] = psi + phi + Mathf.PI / 2;
        theta[0, 3] = psi + phi + Mathf.PI / 2;
        theta[0, 4] = psi - phi + Mathf.PI / 2;
        theta[0, 5] = psi - phi + Mathf.PI / 2;
        theta[0, 6] = psi - phi + Mathf.PI / 2;
        theta[0, 7] = psi - phi + Mathf.PI / 2;

        for (int i = 0; i < 8; i += 4)
        {
            double t5 = (transform_matrix_unity[0, 3] * Mathf.Sin((float)theta[0, i]) - transform_matrix_unity[1, 3] * Mathf.Cos((float)theta[0, i]) - (DH_matrix_UR16e[1, 2] + DH_matrix_UR16e[3, 2] + DH_matrix_UR16e[2, 2])) / DH_matrix_UR16e[5, 2];
            float th5;
            if (1 >= t5 && t5 >= -1)
            {
                th5 = Mathf.Acos((float)t5);
            }
            else
            {
                th5 = 0;
            }

            if (i == 0)
            {
                theta[4, 0] = th5;
                theta[4, 1] = th5;
                theta[4, 2] = -th5;
                theta[4, 3] = -th5;
            }
            else
            {
                theta[4, 4] = th5;
                theta[4, 5] = th5;
                theta[4, 6] = -th5;
                theta[4, 7] = -th5;
            }
        }

        Matrix4x4 tmu_inverse = transform_matrix_unity.inverse;
        float th0 = Mathf.Atan2((-tmu_inverse[1, 0] * Mathf.Sin((float)theta[0, 0]) + tmu_inverse[1, 1] * Mathf.Cos((float)theta[0, 0])), (tmu_inverse[0, 0] * Mathf.Sin((float)theta[0, 0]) - tmu_inverse[0, 1] * Mathf.Cos((float)theta[0, 0])));
        float th2 = Mathf.Atan2((-tmu_inverse[1, 0] * Mathf.Sin((float)theta[0, 2]) + tmu_inverse[1, 1] * Mathf.Cos((float)theta[0, 2])), (tmu_inverse[0, 0] * Mathf.Sin((float)theta[0, 2]) - tmu_inverse[0, 1] * Mathf.Cos((float)theta[0, 2])));
        float th4 = Mathf.Atan2((-tmu_inverse[1, 0] * Mathf.Sin((float)theta[0, 4]) + tmu_inverse[1, 1] * Mathf.Cos((float)theta[0, 4])), (tmu_inverse[0, 0] * Mathf.Sin((float)theta[0, 4]) - tmu_inverse[0, 1] * Mathf.Cos((float)theta[0, 4])));
        float th6 = Mathf.Atan2((-tmu_inverse[1, 0] * Mathf.Sin((float)theta[0, 6]) + tmu_inverse[1, 1] * Mathf.Cos((float)theta[0, 6])), (tmu_inverse[0, 0] * Mathf.Sin((float)theta[0, 6]) - tmu_inverse[0, 1] * Mathf.Cos((float)theta[0, 6])));

        theta[5, 0] = th0;
        theta[5, 1] = th0;
        theta[5, 2] = th2;
        theta[5, 3] = th2;
        theta[5, 4] = th4;
        theta[5, 5] = th4;
        theta[5, 6] = th6;
        theta[5, 7] = th6;

        for (int i = 0; i <= 7; i += 2)
        {
            double[,] t1 = new double[1, 6];
            t1[0, 0] = theta[0, i];
            t1[0, 1] = theta[1, i];
            t1[0, 2] = theta[2, i];
            t1[0, 3] = theta[3, i];
            t1[0, 4] = theta[4, i];
            t1[0, 5] = theta[5, i];
            Matrix4x4 T01 = ComputeTransformMatrix(1, t1);
            Matrix4x4 T45 = ComputeTransformMatrix(5, t1);
            Matrix4x4 T56 = ComputeTransformMatrix(6, t1);
            Matrix4x4 T14 = T01.inverse * transform_matrix_unity * (T45 * T56).inverse;

            Vector4 P13 = T14 * new Vector4()
            {
                x = 0,
                y = (float)-DH_matrix_UR16e[3, 2],
                z = 0,
                w = 1
            };
            double t3 = (Mathf.Pow(P13[0], 2) + Mathf.Pow(P13[1], 2) - Mathf.Pow((float)DH_matrix_UR16e[1, 0], 2) - Mathf.Pow((float)DH_matrix_UR16e[2, 0], 2)) / (2 * DH_matrix_UR16e[1, 0] * DH_matrix_UR16e[2, 0]);
            double th3;
            if (1 >= t3 && t3 >= -1)
            {
                th3 = Mathf.Acos((float)t3);
            }
            else
            {
                th3 = 0;
            }
            theta[2, i] = th3;
            theta[2, i + 1] = -th3;
        }

        for (int i = 0; i < 8; i++)
        {
            double[,] t1 = new double[1, 6];
            t1[0, 0] = theta[0, i];
            t1[0, 1] = theta[1, i];
            t1[0, 2] = theta[2, i];
            t1[0, 3] = theta[3, i];
            t1[0, 4] = theta[4, i];
            t1[0, 5] = theta[5, i];
            Matrix4x4 T01 = ComputeTransformMatrix(1, t1);
            Matrix4x4 T45 = ComputeTransformMatrix(5, t1);
            Matrix4x4 T56 = ComputeTransformMatrix(6, t1);
            Matrix4x4 T14 = T01.inverse * transform_matrix_unity * (T45 * T56).inverse;

            Vector4 P13 = T14 * new Vector4()
            {
                x = 0,
                y = (float)-DH_matrix_UR16e[3, 2],
                z = 0,
                w = 1
            };

            theta[1, i] = Mathf.Atan2(-P13[1], -P13[0]) - Mathf.Asin((float)(-DH_matrix_UR16e[2, 0] * Mathf.Sin((float)theta[2, i]) / Mathf.Sqrt(Mathf.Pow(P13[0], 2) + Mathf.Pow(P13[1], 2))));

            double[,] t2 = new double[1, 6];
            t2[0, 0] = theta[0, i];
            t2[0, 1] = theta[1, i];
            t2[0, 2] = theta[2, i];
            t2[0, 3] = theta[3, i];
            t2[0, 4] = theta[4, i];
            t2[0, 5] = theta[5, i];
            Matrix4x4 T32 = ComputeTransformMatrix(3, t2).inverse;
            Matrix4x4 T21 = ComputeTransformMatrix(2, t2).inverse;
            Matrix4x4 T34 = T32 * T21 * T14;
            theta[3, i] = Mathf.Atan2(T34[1, 0], T34[0, 0]);
        }
        return theta;
    }

    public static List<string> DisplaySolutions(double[,] solutions)
    {
        List<string> info = new List<string>();

        // Iterate through the 8 possible solutions
        for (int column = 0; column < 8; column++)
        {
            // Check if all joint angles in the solution are valid (not NaN)
            bool isValidSolution = true;
            for (int row = 0; row < 6; row++)
            {
                if (double.IsNaN(solutions[row, column]))
                {
                    isValidSolution = false;
                    break;
                }
            }

            // If the solution is valid, format and add the joint angles to the info list
            if (isValidSolution)
            {
                string solutionInfo = "";
                for (int row = 0; row < 6; row++)
                {
                    double angleInDegrees = Math.Round(Mathf.Rad2Deg * solutions[row, column], 2);
                    solutionInfo += $"{angleInDegrees}";

                    if (row < 5)
                    {
                        solutionInfo += " | ";
                    }
                }
                info.Add(solutionInfo);
            }
            // If the solution is not valid, add "NON DISPONIBLE" to the info list
            else
            {
                info.Add("NON DISPONIBLE");
            }
        }

        return info;
    }

}