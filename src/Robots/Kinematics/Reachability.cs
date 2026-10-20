using System.Drawing;
using Rhino.Geometry;

namespace Robots;

public enum ReachStatus { Clear, FlipRisk, NearSingularity, Unreachable }

/// <summary>
/// How a robot reaches a TCP plane: the status, the absolute J5 angle in the
/// manufacturer's degrees, and the joints in radians, if reachable.
/// </summary>
public readonly record struct ReachResult(ReachStatus Status, double Wrist, double[]? Joints);

/// <summary>
/// A point's reachability score from 0 to 1 over the tested rotations, the rotation in
/// degrees with most wrist margin, and how many rotations reach at all.
/// </summary>
public readonly record struct ReachScore(double Score, double BestRotation, int Reachable);

/// <summary>
/// Checks where a 6-axis robot can reach with a tool in a given orientation, and
/// where its wrist is near the J5 = 0 flip boundary. Linear moves there can flip
/// the wrist or pass a singularity.
/// </summary>
public static class Reachability
{
    /// <summary>
    /// Smallest |J5| in degrees considered clear of wrist flips.
    /// </summary>
    public const double FlipMargin = 15;

    /// <summary>
    /// |J5| in degrees below which the wrist is at the flip boundary.
    /// </summary>
    public const double WristBoundary = 5;

    /// <summary>
    /// |J5| in degrees that scores a rotation fully.
    /// </summary>
    public const double ComfortableWrist = 30;

    static readonly Color ClearColor = Color.FromArgb(76, 175, 80);
    static readonly Color FlipRiskColor = Color.FromArgb(255, 160, 0);
    static readonly Color SingularColor = Color.FromArgb(220, 50, 47);
    static readonly Color UnreachableColor = Color.FromArgb(150, 150, 150);

    /// <summary>
    /// Colors of the score scale, from 0 to 1.
    /// </summary>
    public static IReadOnlyList<Color> ScoreColors { get; } = [SingularColor, FlipRiskColor, ClearColor];

    /// <summary>
    /// Scores each point by reaching it with the tool rotated evenly about the
    /// orientation's Z axis. A rotation scores 0 out of reach or near a singularity,
    /// and otherwise |J5| over <see cref="ComfortableWrist"/>, up to 1. The point's
    /// score is the mean, and its best rotation the angle in degrees with most wrist margin.
    /// </summary>
    public static ReachScore[] Score(RobotSystem system, IReadOnlyList<Point3d> points, Plane orientation, int rotations = 8, Tool? tool = null, double[]? reference = null)
    {
        ArgumentOutOfRangeException.ThrowIfLessThan(rotations, 1);

        var planes = new Plane[points.Count * rotations];

        for (int i = 0; i < points.Count; i++)
        {
            for (int r = 0; r < rotations; r++)
            {
                Plane plane = new(points[i], orientation.XAxis, orientation.YAxis);
                _ = plane.Rotate(2 * Math.PI * r / rotations, plane.ZAxis);
                planes[i * rotations + r] = plane;
            }
        }

        var results = Check(system, planes, tool, reference);
        var scores = new ReachScore[points.Count];

        for (int i = 0; i < points.Count; i++)
        {
            double total = 0;
            double best = -1;
            double bestRotation = 0;
            int reachable = 0;

            for (int r = 0; r < rotations; r++)
            {
                var result = results[i * rotations + r];

                if (result.Status is ReachStatus.Unreachable)
                    continue;

                reachable++;
                double value = result.Status is ReachStatus.NearSingularity ? 0 : Math.Min(1, result.Wrist / ComfortableWrist);
                total += value;

                if (result.Wrist > best)
                {
                    best = result.Wrist;
                    bestRotation = 360.0 * r / rotations;
                }
            }

            scores[i] = new(total / rotations, bestRotation, reachable);
        }

        return scores;
    }

    /// <summary>
    /// A square per point colored along the score scale, grey where no rotation reaches.
    /// </summary>
    public static Mesh Map(IReadOnlyList<Plane> planes, IReadOnlyList<ReachScore> scores, double size)
    {
        ArgumentOutOfRangeException.ThrowIfNotEqual(scores.Count, planes.Count, nameof(scores));
        return Squares(planes, size, i => scores[i].Reachable == 0 ? UnreachableColor : ScoreColor(scores[i].Score));
    }

    /// <summary>
    /// The score scale's color, blending red at 0, amber at 0.5 and green at 1.
    /// </summary>
    public static Color ScoreColor(double score)
    {
        double t = Math.Clamp(score, 0, 1) * (ScoreColors.Count - 1);
        int index = Math.Min((int)t, ScoreColors.Count - 2);
        double f = t - index;
        var (a, b) = (ScoreColors[index], ScoreColors[index + 1]);
        return Color.FromArgb(Blend(a.R, b.R), Blend(a.G, b.G), Blend(a.B, b.B));

        int Blend(int from, int to)
        {
            return (int)Math.Round(from + (to - from) * f);
        }
    }

    /// <param name="reference">Optional joints in radians the robot comes from, which choose the solution as the planner would.</param>
    public static ReachResult[] Check(RobotSystem system, IReadOnlyList<Plane> planes, Tool? tool = null, double[]? reference = null)
    {
        if (system is not IndustrialSystem { MechanicalGroups: [{ Robot: { Joints.Length: 6 } robot }] })
            throw new NotSupportedException("Reachability supports a single 6-axis robot.");

        var results = new ReachResult[planes.Count];
        double[][]? previous = reference is null ? null : [reference];

        _ = Parallel.For(0, planes.Count, i =>
        {
            CartesianTarget target = new(planes[i], tool: tool);
            var solution = system.Kinematics([target], previous)[0];
            double wrist = Math.Abs(robot.RadianToDegree(solution.Joints[4], 4));
            results[i] = new(Status(solution, wrist), wrist, solution.Errors.Count == 0 || IsNearSingular(solution) ? solution.Joints : null);
        });

        return results;

        static ReachStatus Status(KinematicSolution solution, double wrist)
        {
            if (IsNearSingular(solution))
                return ReachStatus.NearSingularity;

            if (solution.Errors.Count > 0)
                return ReachStatus.Unreachable;

            return wrist < WristBoundary
                ? ReachStatus.NearSingularity
                : wrist < FlipMargin ? ReachStatus.FlipRisk : ReachStatus.Clear;
        }

        static bool IsNearSingular(KinematicSolution solution)
        {
            return solution.Errors.Count > 0 && solution.Errors.All(error => error.Contains("singularity", StringComparison.OrdinalIgnoreCase));
        }
    }

    /// <summary>
    /// A square per plane in the plane's XY, sized to tile a grid, in each plane's color.
    /// </summary>
    static Mesh Squares(IReadOnlyList<Plane> planes, double size, Func<int, Color> color)
    {
        ArgumentOutOfRangeException.ThrowIfNegativeOrZero(size);

        Mesh mesh = new();
        double half = size / 2;

        for (int i = 0; i < planes.Count; i++)
        {
            var plane = planes[i];
            var squareColor = color(i);
            int first = mesh.Vertices.Count;

            foreach (var (x, y) in new[] { (-half, -half), (half, -half), (half, half), (-half, half) })
            {
                _ = mesh.Vertices.Add(plane.PointAt(x, y));
                _ = mesh.VertexColors.Add(squareColor);
            }

            _ = mesh.Faces.AddFace(first, first + 1, first + 2, first + 3);
        }

        _ = mesh.Normals.ComputeNormals();
        return mesh;
    }
}
