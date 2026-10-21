using NUnit.Framework;
using Rhino.Geometry;

namespace Robots.Tests;

public class ConfigurationSelectionTests
{
    static IEnumerable<TestCaseData> Robots()
    {
        yield return new(TestRobots.UR10()) { TestName = "UR10" };
        yield return new(TestRobots.AbbIrb120()) { TestName = "IRB120" };
        yield return new(TestRobots.FanucCrx10()) { TestName = "CRX-10iA" };
    }

    // Targets some configurations cannot reach have placeholder joints, which must not win for being closest to the previous joints.
    [TestCaseSource(nameof(Robots))]
    public void AutomaticConfigurationReachesWhatAnyConfigurationReaches(RobotSystem system)
    {
        Random random = new(7);
        var joints = new double[6];
        int reachable = 0;
        List<string> failures = [];

        for (int i = 0; i < 600; i++)
        {
            Plane plane = new(new Point3d(random.Next(-700, 700), random.Next(-700, 700), random.Next(-100, 900)), Vector3d.XAxis, -Vector3d.YAxis);
            _ = plane.Rotate(random.NextDouble() * Math.Tau, Vector3d.ZAxis);

            bool any = Enumerable.Range(0, 8).Any(configuration =>
                !system.Kinematics([new CartesianTarget(plane, (RobotConfigurations)configuration)])[0].Errors.Any(error => error.Contains("out of reach", StringComparison.Ordinal)));

            if (!any)
                continue;

            reachable++;
            var automatic = system.Kinematics([new CartesianTarget(plane)], [joints])[0];

            if (automatic.Errors.Any(error => error.Contains("out of reach", StringComparison.Ordinal)))
                failures.Add($"{plane.Origin}");
        }

        Assert.That(reachable, Is.GreaterThan(50));
        Assert.That(failures, Is.Empty, $"{failures.Count} of {reachable} targets reachable in some configuration were out of reach automatically.");
    }
}
