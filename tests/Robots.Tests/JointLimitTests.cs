using System.Xml.Linq;
using NUnit.Framework;
using Rhino.Geometry;

namespace Robots.Tests;

public class JointLimitTests
{
    static readonly Interval[] FullRanges = [new(-190, 190), new(-179.9, 179.9), new(-71, 251), new(-190, 190), new(-179.9, 179.9), new(-225, 225)];

    [Test]
    public void LimitedJointsRejectTargetsOutsideTheLimits()
    {
        Interval[] limits = [new(-10, 10), .. FullRanges[1..]];
        var system = Limited(limits);
        var robot = ((IndustrialSystem)system).MechanicalGroups[0].Robot;

        Assert.Multiple(() =>
        {
            Assert.That(robot.Joints[0].Range.Min, Is.EqualTo(-10 * Math.PI / 180).Within(1e-12));
            Assert.That(Errors(5), Is.Empty);
            Assert.That(Errors(20), Has.Some.Contains("Axis 1 is outside the permitted range"));
        });

        IReadOnlyList<string> Errors(double j1)
        {
            double[] degrees = [j1, 10, -20, 0, -30, 0];
            JointTarget target = new([.. degrees.Select(robot.DegreeToRadian)]);
            return new Program("Limits", system, [TestRobots.Toolpath(target, target)]).Errors;
        }
    }

    [Test]
    public void RejectsLimitsOutsideTheJointRange()
    {
        Interval[] limits = [new(-200, 10), .. FullRanges[1..]];

        var error = Assert.Throws<ArgumentException>(() => Limited(limits));
        Assert.That(error!.Message, Does.Contain("J1 limit of -200 to 10 deg must be an increasing range within the joint's range of -190 to 190 deg"));
    }

    [Test]
    public void RejectsMoreLimitsThanJoints()
    {
        var error = Assert.Throws<ArgumentException>(() => Limited([.. FullRanges, new(-10, 10)]));
        Assert.That(error!.Message, Does.Contain("Expected at most 6 joint limits"));
    }

    [Test]
    public void SingleValuesKeepTheJointRange()
    {
        var robot = ((IndustrialSystem)Limited([new(-1, -1), new(-1, -1), new(-1, -1), new(-45, 45)])).MechanicalGroups[0].Robot;

        Assert.Multiple(() =>
        {
            Assert.That(robot.Joints[0].Range.Max, Is.EqualTo(190 * Math.PI / 180).Within(1e-12));
            Assert.That(robot.Joints[3].Range.Length, Is.EqualTo(90 * Math.PI / 180).Within(1e-12));
        });
    }

    [Test]
    public void LimitsOnlyTheFirstJoints()
    {
        var robot = ((IndustrialSystem)Limited([new(-10, 10)])).MechanicalGroups[0].Robot;

        Assert.Multiple(() =>
        {
            Assert.That(robot.Joints[0].Range.Max, Is.EqualTo(10 * Math.PI / 180).Within(1e-12));
            Assert.That(robot.Joints[5].Range.Max, Is.EqualTo(225 * Math.PI / 180).Within(1e-12));
        });
    }

    static RobotSystem Limited(IReadOnlyList<Interval> limits)
    {
        var element = RobotSystemParser.LimitJoints(XElement.Parse(TestRobots.FanucCrx10Xml), limits);
        return FileIO.ParseRobotSystem(element.ToString(), Plane.WorldXY);
    }
}
