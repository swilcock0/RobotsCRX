using System.Xml.Linq;
using NUnit.Framework;
using Rhino.Geometry;

namespace Robots.Tests;

public class ReachabilityTests
{
    // A CRX-10iA/L on a pedestal, J2 713 mm above the floor, with a gripper pointing down.
    static readonly Tool Gripper = new(new Plane(new Point3d(0, 0, 122), Vector3d.XAxis, Vector3d.YAxis), "Gripper");
    static readonly Plane Down = new(Point3d.Origin, Vector3d.XAxis, -Vector3d.YAxis);

    [Test]
    public void FlagsTheWristFlipBandOfARaisedCrx()
    {
        var system = RaisedCrx();
        Plane[] planes = [At(600), At(950), At(3000)];

        var results = Reachability.Check(system, planes, Gripper);

        Assert.Multiple(() =>
        {
            Assert.That(results[0].Status, Is.EqualTo(ReachStatus.NearSingularity).Or.EqualTo(ReachStatus.FlipRisk), "In the J5 = 0 band.");
            Assert.That(results[0].Wrist, Is.LessThan(Reachability.FlipMargin));
            Assert.That(results[1].Status, Is.EqualTo(ReachStatus.Clear));
            Assert.That(results[1].Wrist, Is.GreaterThan(Reachability.FlipMargin));
            Assert.That(results[2].Status, Is.EqualTo(ReachStatus.Unreachable));
            Assert.That(results[2].Joints, Is.Null);
        });

        static Plane At(double x)
        {
            return new(new Point3d(x, 0, 15), Down.XAxis, Down.YAxis);
        }
    }

    [Test]
    public void ScoresPointsOverRotations()
    {
        var system = RaisedCrx();
        Point3d[] points = [new(600, 0, 15), new(950, 0, 15), new(3000, 0, 15)];

        var scores = Reachability.Score(system, points, Down, 8, Gripper);
        var map = Reachability.Map([.. points.Select(point => new Plane(point, Vector3d.XAxis, Vector3d.YAxis))], scores, 40);

        Assert.Multiple(() =>
        {
            Assert.That(scores[1].Score, Is.GreaterThan(scores[0].Score), "Clear of the band scores higher.");
            Assert.That(scores[1].Score, Is.InRange(0.5, 1.0));
            Assert.That(scores[1].Reachable, Is.EqualTo(8));
            Assert.That(scores[2].Score, Is.Zero);
            Assert.That(scores[2].Reachable, Is.Zero);
            Assert.That(scores.Select(score => score.BestRotation), Has.All.InRange(0.0, 315.0));
            Assert.That(map.Faces, Has.Count.EqualTo(3));
            Assert.That(map.VertexColors, Has.Count.EqualTo(map.Vertices.Count));
            Assert.That(Reachability.ScoreColor(0), Is.EqualTo(Reachability.ScoreColors[0]));
            Assert.That(Reachability.ScoreColor(1), Is.EqualTo(Reachability.ScoreColors[^1]));
        });
    }

    [Test]
    public void RejectsSystemsWithSeveralRobots()
    {
        var error = Assert.Throws<NotSupportedException>(() => Reachability.Check(TestRobots.AbbTwoGroupWithCustomExternal(), [Plane.WorldXY]));
        Assert.That(error!.Message, Does.Contain("single 6-axis robot"));
    }

    static RobotSystem RaisedCrx() =>
        FileIO.ParseRobotSystem(BundledCell("Fanuc-CRX-10iA/L"), new Plane(new Point3d(0, 0, 713), Vector3d.XAxis, Vector3d.YAxis));

    static string BundledCell(string name)
    {
        var root = new DirectoryInfo(TestContext.CurrentContext.TestDirectory);

        while (!File.Exists(Path.Combine(root.FullName, "Robots.slnx")))
            root = root.Parent ?? throw new DirectoryNotFoundException("Repository root not found.");

        var file = Path.Combine(root.FullName, "src", "Robots.Grasshopper", "Resources", "Package", "libraries", "Fanuc_CRX.xml");
        return XDocument.Load(file).Root!.Elements("RobotCell").Single(cell => cell.Attribute("name")!.Value == name).ToString();
    }
}
