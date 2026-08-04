using System;

// Analytic (closed-form) inverse kinematics for Universal Robots e-Series arms,
// following the standard UR solution (Hawkins 2013, "Analytic Inverse Kinematics
// for the Universal Robots UR-5/UR-10 Arms").
//
// Pure C# / double precision, no UnityEngine dependency: the math can be unit-tested
// outside the editor. All poses are expressed in the robot's right-handed DH base
// frame; the Unity-side conversion (left-handed mirror) lives in IK_toolkit.cs.
namespace URRobot
{
    // Rigid transform (rotation r + translation p), the 3x4 upper part of a
    // homogeneous matrix. Row-major: rRC = row R, column C.
    public struct RigidTransform
    {
        public double r00, r01, r02, px;
        public double r10, r11, r12, py;
        public double r20, r21, r22, pz;

        public static RigidTransform Identity => new RigidTransform { r00 = 1, r11 = 1, r22 = 1 };

        // Denavit-Hartenberg link transform Rz(theta) * Tz(d) * Tx(a) * Rx(alpha).
        public static RigidTransform FromDH(double theta, double d, double a, double alpha)
        {
            double ct = Math.Cos(theta), st = Math.Sin(theta);
            double ca = Math.Cos(alpha), sa = Math.Sin(alpha);
            return new RigidTransform
            {
                r00 = ct, r01 = -st * ca, r02 = st * sa, px = a * ct,
                r10 = st, r11 = ct * ca, r12 = -ct * sa, py = a * st,
                r20 = 0, r21 = sa, r22 = ca, pz = d,
            };
        }

        public static readonly RigidTransform Zero = default;

        public static RigidTransform operator *(in RigidTransform a, in RigidTransform b)
        {
            return new RigidTransform
            {
                r00 = a.r00 * b.r00 + a.r01 * b.r10 + a.r02 * b.r20,
                r01 = a.r00 * b.r01 + a.r01 * b.r11 + a.r02 * b.r21,
                r02 = a.r00 * b.r02 + a.r01 * b.r12 + a.r02 * b.r22,
                px = a.r00 * b.px + a.r01 * b.py + a.r02 * b.pz + a.px,
                r10 = a.r10 * b.r00 + a.r11 * b.r10 + a.r12 * b.r20,
                r11 = a.r10 * b.r01 + a.r11 * b.r11 + a.r12 * b.r21,
                r12 = a.r10 * b.r02 + a.r11 * b.r12 + a.r12 * b.r22,
                py = a.r10 * b.px + a.r11 * b.py + a.r12 * b.pz + a.py,
                r20 = a.r20 * b.r00 + a.r21 * b.r10 + a.r22 * b.r20,
                r21 = a.r20 * b.r01 + a.r21 * b.r11 + a.r22 * b.r21,
                r22 = a.r20 * b.r02 + a.r21 * b.r12 + a.r22 * b.r22,
                pz = a.r20 * b.px + a.r21 * b.py + a.r22 * b.pz + a.pz,
            };
        }

        // Inverse of a rigid transform: (R^T, -R^T p). Exact, no Gaussian elimination.
        public readonly RigidTransform Inverse()
        {
            return new RigidTransform
            {
                r00 = r00, r01 = r10, r02 = r20, px = -(r00 * px + r10 * py + r20 * pz),
                r10 = r01, r11 = r11, r12 = r21, py = -(r01 * px + r11 * py + r21 * pz),
                r20 = r02, r21 = r12, r22 = r22, pz = -(r02 * px + r12 * py + r22 * pz),
            };
        }

        public readonly void TransformPoint(double x, double y, double z, out double ox, out double oy, out double oz)
        {
            ox = r00 * x + r01 * y + r02 * z + px;
            oy = r10 * x + r11 * y + r12 * z + py;
            oz = r20 * x + r21 * y + r22 * z + pz;
        }
    }

    public static class URKinematics
    {
        // UR e-Series DH constants (meters). d2 = d3 = 0 and a1 = a4 = a5 = a6 = 0
        // for every UR arm, so only the six non-zero parameters are stored.
        // alpha = { +90, 0, 0, +90, -90, 0 } degrees, identical across the series.
        public readonly struct DHParameters
        {
            public readonly double d1, a2, a3, d4, d5, d6;
            public DHParameters(double d1, double a2, double a3, double d4, double d5, double d6)
            {
                this.d1 = d1; this.a2 = a2; this.a3 = a3; this.d4 = d4; this.d5 = d5; this.d6 = d6;
            }
        }

        // Official UR16e values (Universal Robots DH parameter tables).
        // Other arms in the series differ only in these six numbers, so a preset
        // for another model is a one-line addition.
        public static readonly DHParameters UR16e = new DHParameters(0.1807, -0.4784, -0.36, 0.17415, 0.11985, 0.11655);

        // Nominal reach in metres (|a2| + |a3| + d5), matching the published specs.
        public static double Reach(in DHParameters dh) =>
            Math.Abs(dh.a2) + Math.Abs(dh.a3) + dh.d5;

        public const int SolutionCount = 8;

        // Tolerance when clamping acos/asin arguments: values overshooting [-1, 1]
        // by less than this are treated as boundary (the input pose comes from
        // float-precision Unity transforms), beyond it the branch is unreachable.
        const double ClampTolerance = 1e-6;
        // Wrist singularity (joints 4 and 6 collinear, sin(theta5) = 0): theta6 is a
        // free parameter and gets pinned by the caller. The test is applied to the
        // magnitude of the theta6 numerator/denominator pair, which equals
        // |sin(theta5)| analytically but is read straight off the target matrix --
        // testing sin(acos(t5)) instead would never fire, since acos cannot return
        // an angle whose sine is below ~2e-8.
        const double WristSingularEps = 1e-6;

        static readonly double[] DhAlpha = { Math.PI / 2, 0, 0, Math.PI / 2, -Math.PI / 2, 0 };

        static void LinkConstants(in DHParameters dh, int joint, out double d, out double a)
        {
            switch (joint)
            {
                case 0: d = dh.d1; a = 0; break;
                case 1: d = 0; a = dh.a2; break;
                case 2: d = 0; a = dh.a3; break;
                case 3: d = dh.d4; a = 0; break;
                case 4: d = dh.d5; a = 0; break;
                default: d = dh.d6; a = 0; break;
            }
        }

        public static RigidTransform LinkTransform(in DHParameters dh, int joint, double theta)
        {
            LinkConstants(dh, joint, out double d, out double a);
            return RigidTransform.FromDH(theta, d, a, DhAlpha[joint]);
        }

        // Forward kinematics: base -> flange transform for the given joint angles (radians).
        public static RigidTransform ForwardKinematics(in DHParameters dh, double[] joints)
        {
            var t = LinkTransform(dh, 0, joints[0]);
            for (int i = 1; i < 6; i++)
                t = t * LinkTransform(dh, i, joints[i]);
            return t;
        }

        // Computes the up-to-8 closed-form solutions for the target flange pose.
        //
        // solutions is a [6, 8] array (joint row, solution column); valid[c] tells
        // whether column c is a reachable configuration. Columns are ordered
        // shoulder(+/-) x wrist(+/-) x elbow(+/-):
        //   c = 4*shoulder + 2*wrist + elbow.
        // wristSingularTheta6 pins theta6 when the wrist is singular (theta5 ~ 0),
        // where theta6 is mathematically free — pass the current joint 6 angle to
        // avoid jumps. Returns the number of valid solutions.
        public static int Solve(in RigidTransform target, in DHParameters dh,
                                double[,] solutions, bool[] valid, double wristSingularTheta6 = 0)
        {
            if (solutions == null || solutions.GetLength(0) != 6 || solutions.GetLength(1) != SolutionCount)
                throw new ArgumentException("solutions must be a [6, 8] array");
            if (valid == null || valid.Length != SolutionCount)
                throw new ArgumentException("valid must have length 8");

            for (int c = 0; c < SolutionCount; c++) valid[c] = false;

            RigidTransform inv = target.Inverse();

            // ----- theta1 (shoulder): from P05, the wrist-2 center seen from the base.
            target.TransformPoint(0, 0, -dh.d6, out double p05x, out double p05y, out _);
            double radius = Math.Sqrt(p05x * p05x + p05y * p05y);
            if (!TryAcos(dh.d4 / radius, out double phi))
                return 0; // target closer to the base axis than d4: no shoulder solution
            double psi = Math.Atan2(p05y, p05x);

            int found = 0;
            for (int shoulder = 0; shoulder < 2; shoulder++)
            {
                double theta1 = psi + (shoulder == 0 ? phi : -phi) + Math.PI / 2;
                double s1 = Math.Sin(theta1), c1 = Math.Cos(theta1);

                // ----- theta5 (wrist): angle between joint-6 axis and the theta1 plane.
                double t5 = (target.px * s1 - target.py * c1 - dh.d4) / dh.d6;
                if (!TryAcos(t5, out double theta5abs))
                    continue;

                for (int wrist = 0; wrist < 2; wrist++)
                {
                    double theta5 = wrist == 0 ? theta5abs : -theta5abs;
                    double s5 = Math.Sin(theta5);

                    // ----- theta6: from the target orientation, sign of sin(theta5) included.
                    // Reference divides both atan2 arguments by sin(theta5); only the
                    // sign matters, so apply it directly and skip the division.
                    double num = -inv.r10 * s1 + inv.r11 * c1;
                    double den = inv.r00 * s1 - inv.r01 * c1;
                    double theta6;
                    if (Math.Sqrt(num * num + den * den) < WristSingularEps)
                    {
                        theta6 = wristSingularTheta6; // free parameter at the singularity
                    }
                    else
                    {
                        double sign = s5 < 0 ? -1.0 : 1.0;
                        theta6 = Math.Atan2(sign * num, sign * den);
                    }

                    // ----- theta3 (elbow): law of cosines on the joint-1 -> joint-4 chain.
                    RigidTransform t01 = LinkTransform(dh, 0, theta1);
                    RigidTransform t45 = LinkTransform(dh, 4, theta5);
                    RigidTransform t56 = LinkTransform(dh, 5, theta6);
                    RigidTransform t14 = t01.Inverse() * target * (t45 * t56).Inverse();
                    t14.TransformPoint(0, -dh.d4, 0, out double p13x, out double p13y, out _);

                    double lenSq = p13x * p13x + p13y * p13y;
                    double len = Math.Sqrt(lenSq);
                    double t3 = (lenSq - dh.a2 * dh.a2 - dh.a3 * dh.a3) / (2 * dh.a2 * dh.a3);
                    if (!TryAcos(t3, out double theta3abs))
                        continue;

                    for (int elbow = 0; elbow < 2; elbow++)
                    {
                        double theta3 = elbow == 0 ? theta3abs : -theta3abs;

                        // ----- theta2 (upper arm) from the P13 triangle.
                        if (!TryAsin(-dh.a3 * Math.Sin(theta3) / len, out double delta))
                            continue;
                        double theta2 = Math.Atan2(-p13y, -p13x) - delta;

                        // ----- theta4: residual rotation so the chain matches T14.
                        RigidTransform t12 = LinkTransform(dh, 1, theta2);
                        RigidTransform t23 = LinkTransform(dh, 2, theta3);
                        RigidTransform t34 = (t12 * t23).Inverse() * t14;
                        double theta4 = Math.Atan2(t34.r10, t34.r00);

                        int c = 4 * shoulder + 2 * wrist + elbow;
                        solutions[0, c] = NormalizeAngle(theta1);
                        solutions[1, c] = NormalizeAngle(theta2);
                        solutions[2, c] = NormalizeAngle(theta3);
                        solutions[3, c] = NormalizeAngle(theta4);
                        solutions[4, c] = NormalizeAngle(theta5);
                        solutions[5, c] = NormalizeAngle(theta6);
                        valid[c] = true;
                        found++;
                    }
                }
            }
            return found;
        }

        // Index of the valid solution closest to reference in joint space
        // (2*pi-wrapped, proximal joints weighted heavier), or -1 if none is valid.
        public static int ClosestSolution(double[,] solutions, bool[] valid, double[] reference)
        {
            double best = double.MaxValue;
            int bestIndex = -1;
            for (int c = 0; c < SolutionCount; c++)
            {
                if (!valid[c]) continue;
                double cost = 0;
                for (int j = 0; j < 6; j++)
                {
                    double d = NormalizeAngle(solutions[j, c] - reference[j]);
                    cost += (6 - j) * d * d; // moving the base is costlier than the wrist
                }
                if (cost < best) { best = cost; bestIndex = c; }
            }
            return bestIndex;
        }

        // Wraps an angle to (-pi, pi].
        public static double NormalizeAngle(double angle)
        {
            angle = Math.IEEERemainder(angle, 2 * Math.PI);
            return angle <= -Math.PI ? angle + 2 * Math.PI : angle;
        }

        static bool TryAcos(double t, out double angle)
        {
            if (t > 1 + ClampTolerance || t < -1 - ClampTolerance || double.IsNaN(t))
            {
                angle = 0;
                return false;
            }
            angle = Math.Acos(Math.Clamp(t, -1, 1));
            return true;
        }

        static bool TryAsin(double t, out double angle)
        {
            if (t > 1 + ClampTolerance || t < -1 - ClampTolerance || double.IsNaN(t))
            {
                angle = 0;
                return false;
            }
            angle = Math.Asin(Math.Clamp(t, -1, 1));
            return true;
        }
    }
}
