using NUnit.Framework;
using Rhino.Geometry;

namespace Robots.Tests;

/// <summary>
/// The planner solves samples along a path by continuation from the previous
/// sample. Each must be the solution the full inverse kinematics picks from the
/// same previous joints.
/// </summary>
public class ContinuationTests
{
    static readonly Tool Gripper = new(new Plane(new Point3d(0, 0, 122), Vector3d.XAxis, Vector3d.YAxis), "Gripper");

    [TestCase(0.0, 0, TestName = "Straight down")]
    [TestCase(30.0, 0, TestName = "Tilted")]
    [TestCase(30.0, 10, TestName = "Tilted with fly-by")]
    [TestCase(-1.0, 10, TestName = "Random orientations with fly-by")]
    public void PathSamplesMatchTheFullInverseKinematics(double tilt, double zone)
    {
        var system = TestRobots.FanucCrx10();
        Random random = new(7);
        int programs = 0;
        int samples = 0;

        for (int attempt = 0; attempt < 100 && programs < 6; attempt++)
        {
            var program = RandomPath(system, random, tilt, zone);

            if (program.Errors.Count > 0)
                continue;

            programs++;
            samples += AssertSamplesMatch(system, program);
        }

        Assert.Multiple(() =>
        {
            Assert.That(programs, Is.EqualTo(6), "Too few reachable random paths.");
            Assert.That(samples, Is.GreaterThan(150));
        });
    }

    [Test, NonParallelizable]
    public void ContinuationSpeedsUpStraightDownPaths()
    {
        // Straight down is where the full inverse kinematics is slowest.
        var system = TestRobots.FanucCrx10();
        Plane start = new(new Point3d(500, -300, -150), Vector3d.XAxis, -Vector3d.YAxis);
        Plane end = new(new Point3d(500, 300, -100), Vector3d.XAxis, -Vector3d.YAxis);
        CartesianTarget first = new(start, tool: Gripper);
        CartesianTarget second = new(end, motion: Motions.Linear, tool: Gripper);

        var (full, fullTime) = Plan(continuation: false);
        var (continued, continuedTime) = Plan(continuation: true);

        Assert.Multiple(() =>
        {
            Assert.That(continued.Errors, Is.Empty);
            Assert.That(TestRobots.FlattenCode(continued), Is.EqualTo(TestRobots.FlattenCode(full)));
            Assert.That(continued.Warnings, Is.EqualTo(full.Warnings));

            // J4 turns half a revolution on this path, and whether it ends at +pi or -pi is a
            // tie that rounding in the previous sample decides, so compare angles, not windings.
            Assert.That(continued.Targets[^1].Joints.Zip(full.Targets[^1].Joints, (a, b) => Math.Abs(Math.IEEERemainder(a - b, 2 * Math.PI))), Has.All.LessThan(1e-6));
            Assert.That(continuedTime, Is.LessThan(fullTime / 5), "Continuation was not used.");
        });

        (Robots.Program Program, double Milliseconds) Plan(bool continuation)
        {
            RobotKinematics.ContinuationEnabled = continuation;

            try
            {
                var stopwatch = System.Diagnostics.Stopwatch.StartNew();
                Robots.Program program = new("Straight", system, [TestRobots.Toolpath(first, second)]);
                return (program, stopwatch.Elapsed.TotalMilliseconds);
            }
            finally
            {
                RobotKinematics.ContinuationEnabled = true;
            }
        }
    }

    static Robots.Program RandomPath(RobotSystem system, Random random, double tilt, double zone)
    {
        // A reachable start from random joints, then linear moves from there.
        var robot = ((IndustrialSystem)system).MechanicalGroups[0].Robot;
        double[] degrees = [Next(-150, 150), Next(-40, 60), Next(-40, 60), Next(-150, 150), Next(-110, 110), Next(-150, 150)];
        JointTarget start = new([.. degrees.Select(robot.DegreeToRadian)], Gripper);
        var plane = system.Kinematics([start])[0].Planes[^1];

        if (tilt >= 0)
        {
            // Straight down, then tilted up to the given angle.
            plane = new(plane.Origin, Vector3d.XAxis, -Vector3d.YAxis);
            _ = plane.Rotate(tilt * (random.NextDouble() * 2 - 1) * Math.PI / 180, plane.XAxis);
            start = new([.. system.Kinematics([new CartesianTarget(plane, tool: Gripper)])[0].Joints], Gripper);
        }

        List<Target> targets = [start];

        for (int i = 0; i < 3; i++)
        {
            Vector3d move = new(Next(-1, 1), Next(-1, 1), Next(-1, 1));
            _ = move.Unitize();
            plane.Origin += move * Next(40, 150);
            _ = plane.Rotate(Next(-0.3, 0.3), Vector3d.ZAxis, plane.Origin);
            targets.Add(new CartesianTarget(plane, motion: Motions.Linear, tool: Gripper, zone: new(zone)));
        }

        return new("Continuation", system, [TestRobots.Toolpath([.. targets])], stepSize: 2);

        double Next(double min, double max)
        {
            return min + random.NextDouble() * (max - min);
        }
    }

    static int AssertSamplesMatch(RobotSystem system, Robots.Program program)
    {
        List<SystemTarget> path = [];

        foreach (var segment in program.MotionSegments)
        {
            foreach (var sample in segment.CheckedSamples ?? [segment.Start, segment.End])
            {
                if (path.Count == 0 || !ReferenceEquals(path[^1], sample))
                    path.Add(sample);
            }
        }

        int count = 0;

        for (int i = 1; i < path.Count; i++)
        {
            var sample = path[i].ProgramTargets[0];

            if (sample.IsJointMotion)
                continue;

            var previous = path[i - 1].ProgramTargets[0].Kinematics.Joints;
            CartesianTarget target = new(sample.Kinematics.Planes[^1], motion: Motions.Linear, tool: Gripper);
            var full = system.Kinematics([target], [previous])[0];

            Assert.That(sample.Kinematics.Joints, Is.EqualTo(full.Joints).Within(1e-6), $"Sample {i} of {path.Count}.");
            count++;
        }

        return count;
    }
}
