using System.Diagnostics;
using System.Xml.Linq;
using NUnit.Framework;
using Rhino.FileIO;
using Rhino.Geometry;
using static System.Math;

namespace Robots.Tests;

public class CollisionCheckerTests
{
    static readonly Plane Down = new(Point3d.Origin, Vector3d.XAxis, -Vector3d.YAxis);

    [Test]
    public void GjkFindsTheGapBetweenTurnedBoxes()
    {
        var random = new Random(3);
        var box = ConvexHull.FromPoints(Box(new(-50, 50), new(-20, 20), new(-10, 10)).Vertices.ToPoint3dArray());

        for (int i = 0; i < 500; i++)
        {
            // Box B is beside box A along X with a known gap, then both are turned and moved together.
            double gap = (random.NextDouble() * 4) - 1;
            double margin = 1;
            var frame = RandomPlane(random);
            var frameB = frame;
            frameB.Origin += frame.XAxis * (100 + gap);

            bool within = Gjk.Within(box, frame, box, frameB, margin);
            Assert.That(within, Is.EqualTo(gap < margin), $"Gap {gap}.");
        }
    }

    [Test]
    public void GjkFindsTheDistanceFromARoundHullToAPoint()
    {
        var random = new Random(5);
        var sphere = ConvexHull.FromPoints(Enumerable.Range(0, 4000).Select(_ => (Point3d)(RandomDirection(random) * 100)));
        var point = ConvexHull.FromPoints([Point3d.Origin]);

        for (int i = 0; i < 200; i++)
        {
            double distance = 98 + (random.NextDouble() * 10);
            var frame = Plane.WorldXY;
            frame.Origin = (Point3d)(RandomDirection(random) * distance);

            bool within = Gjk.Within(sphere, Plane.WorldXY, point, frame, 3);

            // The hull of the points is just inside the sphere.
            if (distance < 102.5)
                Assert.That(within, Is.True, $"Distance {distance}.");
            else if (distance > 103.5)
                Assert.That(within, Is.False, $"Distance {distance}.");
        }
    }

    [Test]
    public void SplitsUnweldedMeshesIntoPieces()
    {
        var mesh = Box(new(0, 10), new(0, 10), new(0, 10));
        mesh.Append(Box(new(20, 30), new(0, 10), new(0, 10)));

        var pieces = ConvexHull.Pieces(mesh);

        Assert.That(pieces, Has.Length.EqualTo(2));
        Assert.That(pieces.Select(piece => piece.Points.Length), Is.All.EqualTo(8));
    }

    [Test]
    public void CleanCellHasNoCollisions()
    {
        var cell = Cell.Create();
        var program = cell.Program();

        var watch = Stopwatch.StartNew();
        var contacts = program.CheckCellCollisions(cell.Options);
        TestContext.Out.WriteLine($"Checked in {watch.ElapsedMilliseconds} ms.");

        Assert.That(contacts, Is.Empty, string.Join(" ", contacts.Select(c => $"{c.TargetIndex}: {c}")));
    }

    [Test]
    public void CreateProgramReportsCollisionsAsErrors()
    {
        var cell = Cell.Create();

        // A block over the stack, which no turn, lift or route avoids.
        var block = Box(new(700, 1000), new(-450, -150), new(150, 400));
        var options = cell.Options with { Environment = [.. cell.Options.Environment, block] };

        var program = cell.Program(options);

        Assert.Multiple(() =>
        {
            Assert.That(program.Collisions, Is.Not.Empty);
            Assert.That(program.Errors, Has.Some.Contains("environment mesh 2"));
            Assert.That(program.Code, Is.Null);
        });
    }

    [Test]
    public void RestingPartsCollideWithoutClearance()
    {
        var cell = Cell.Create();

        var contacts = cell.Program().CheckCellCollisions(cell.Options with { Clearance = 0 });

        Assert.That(contacts.Select(contact => contact.Second), Has.Some.Contains("(placed)"));
    }

    [Test]
    public void CreateProgramLiftsPickAndPlacesOverAWall()
    {
        var cell = Cell.Create(parts: 3);
        var options = cell.Options with { Environment = [.. cell.Options.Environment, Post(400)] };

        var kept = cell.Program(options with { Avoid = false });
        var avoided = cell.Program(options);

        Assert.Multiple(() =>
        {
            Assert.That(kept.Errors, Has.Some.Contains("environment mesh 2"));
            Assert.That(avoided.Errors, Is.Empty);
            Assert.That(avoided.Warnings, Has.Some.Contains("lifted"));
            Assert.That(avoided.Targets, Has.Count.GreaterThan(kept.Targets.Count), "Lifted pick and places have more targets.");
            Assert.That(avoided.Targets[0].ProgramTargets[0].Target, Is.SameAs(cell.Start), "Other targets are kept.");
        });
    }

    [Test]
    public void CreateProgramRoutesPickAndPlacesAroundATallPost()
    {
        var cell = Cell.Create(parts: 3);
        var options = cell.Options with { Environment = [.. cell.Options.Environment, Post(2000)] };

        var kept = cell.Program(options with { Avoid = false });
        var avoided = cell.Program(options);
        var again = cell.Program(options);
        var waypoints = avoided.Targets.Skip(1).Count(target => target.ProgramTargets[0].Target is JointTarget);

        Assert.Multiple(() =>
        {
            Assert.That(kept.Errors, Has.Some.Contains("environment mesh 2"));
            Assert.That(avoided.Errors, Is.Empty);
            Assert.That(avoided.Warnings, Has.Some.Contains("routed through"));
            Assert.That(waypoints, Is.InRange(1, 6 * cell.Picks.Length), "A few waypoints for each pick and place.");
            Assert.That(again.Code, Is.EqualTo(avoided.Code), "The same cell gets the same route.");
        });
    }

    [Test]
    public void CreateProgramRoutesOtherJointMoves()
    {
        var cell = Cell.Create(parts: 3);
        var options = cell.Options with { Environment = [.. cell.Options.Environment, Post(2000)] };

        // Plain joint moves between two points, with no pick and place.
        List<Target> targets = [cell.Start, cell.Above(cell.Picks[0]), cell.Above(cell.Places[0]), cell.Above(cell.Picks[1])];

        var kept = new Program("Cell", cell.System, [new SimpleToolpath(targets)], collisions: options with { Avoid = false });
        var avoided = new Program("Cell", cell.System, [new SimpleToolpath(targets)], collisions: options);

        Assert.Multiple(() =>
        {
            Assert.That(kept.Errors, Has.Some.Contains("environment mesh 2"));
            Assert.That(avoided.Errors, Is.Empty);
            Assert.That(avoided.Warnings, Has.Some.Contains("Joint move routed"));
            Assert.That(avoided.Targets, Has.Count.GreaterThan(targets.Count));
        });
    }

    [Test]
    public void CreateProgramLeavesCollidingLinearMovesAsErrors()
    {
        var cell = Cell.Create(parts: 3);
        var options = cell.Options with { Environment = [.. cell.Options.Environment, Post(2000)] };

        // Straight through the post, which routing cannot change.
        Plane across = new(new Point3d(700, -905, 215), Down.XAxis, Down.YAxis);
        CartesianTarget line = new(across, cell.Template, motion: Motions.Linear);
        var program = new Program("Cell", cell.System, [new SimpleToolpath([cell.Start, cell.Above(cell.Picks[0]), line])], collisions: options);

        Assert.That(program.Errors, Has.Some.Contains("Collision"));
    }

    [Test]
    public void CreateProgramLiftsNoMoreThanEachPickAndPlaceNeeds()
    {
        var cell = Cell.Create(parts: 4);
        var options = cell.Options with { Environment = [.. cell.Options.Environment, Post(400)] };

        var lifts = cell.Program(options).Warnings
            .Select(warning => System.Text.RegularExpressions.Regex.Match(warning, @"lifted (\d+) mm"))
            .Where(match => match.Success)
            .Select(match => int.Parse(match.Groups[1].Value, System.Globalization.CultureInfo.InvariantCulture))
            .ToArray();

        Assert.That(lifts, Is.Not.Empty);
        Assert.That(lifts, Has.All.LessThanOrEqualTo(500), "No higher than a 400 mm wall needs.");
    }

    [Test]
    public void CreateProgramReachesPickAndPlacesInAnotherArmConfiguration()
    {
        var cell = Cell.Create(parts: 2);

        // A block where the elbow is when the arm reaches the first pick in the configuration the solver chooses.
        var plain = new Program("Cell", cell.System, [new SimpleToolpath([cell.Start, cell.Above(cell.Picks[0])])]);
        var elbow = plain.Targets[^1].ProgramTargets[0].Kinematics.Planes[3].Origin;
        var block = Box(new(elbow.X - 120, elbow.X + 120), new(elbow.Y - 120, elbow.Y + 120), new(elbow.Z - 120, elbow.Z + 120));
        var options = cell.Options with { Environment = [.. cell.Options.Environment, block] };

        var kept = cell.Program(options with { Avoid = false });
        var avoided = cell.Program(options);

        Assert.Multiple(() =>
        {
            Assert.That(kept.Errors, Has.Some.Contains("environment mesh 2"));
            Assert.That(avoided.Errors, Is.Empty);
            Assert.That(avoided.Warnings, Has.Some.Contains("configuration"));
        });
    }

    [Test]
    public void CreateProgramReconfiguresOtherTargetsToClearLinearMoves()
    {
        var cell = Cell.Create(parts: 2);

        // A joint move above the pick, then straight down to it, with a block where the elbow is at the bottom in the configuration the solver chooses.
        CartesianTarget down = new(cell.Picks[0], cell.Template, motion: Motions.Linear);
        List<Target> targets = [cell.Start, cell.Above(cell.Picks[0]), down];
        var plain = new Program("Cell", cell.System, [new SimpleToolpath(targets)]);
        var elbow = plain.Targets[^1].ProgramTargets[0].Kinematics.Planes[3].Origin;
        var block = Box(new(elbow.X - 40, elbow.X + 40), new(elbow.Y - 40, elbow.Y + 40), new(elbow.Z - 40, elbow.Z + 40));
        var options = cell.Options with { Environment = [.. cell.Options.Environment, block] };

        var kept = new Program("Cell", cell.System, [new SimpleToolpath(targets)], collisions: options with { Avoid = false });
        var avoided = new Program("Cell", cell.System, [new SimpleToolpath(targets)], collisions: options);

        Assert.Multiple(() =>
        {
            Assert.That(kept.Errors, Has.Some.Contains("environment mesh 2"));
            Assert.That(avoided.Errors, Is.Empty);
            Assert.That(avoided.Warnings, Has.Some.Contains("configuration"));
        });
    }

    [Test]
    public void CheckFindsABlockOnThePathOfALongJointMove()
    {
        var cell = Cell.Create(parts: 1);

        // The whole way round, with only the tool passing through a small block on its path.
        JointTarget end = new([.. new double[] { 170, 20, 30, 0, -60, 0 }.Select(degrees => degrees * PI / 180)], cell.Template.Tool);
        Program program = new("Swing", cell.System, [new SimpleToolpath([cell.Start, end])]);
        Assert.That(program.Errors, Is.Empty);

        program.Animate(program.Duration * 0.4, false);
        var tip = program.CurrentSimulationPose.Kinematics[0].Planes[^1].Origin;
        var block = Box(new(tip.X - 20, tip.X + 20), new(tip.Y - 20, tip.Y + 20), new(tip.Z - 20, tip.Z + 20));

        var found = program.CheckCellCollisions(new CollisionOptions { Environment = [block], Parts = false });

        Assert.That(found, Has.Count.EqualTo(1), "Skipping poses far from everything must not step over it.");
    }

    [Test]
    public void ConvexHullMovedWithATransformMatchesTheHullOfTheMovedMesh()
    {
        var mesh = Box(new(-150, 150), new(-20, 20), new(-9, 19));
        Random random = new(3);
        Transform move = Transform.Rotation(0.7, new Vector3d(1, 2, 3), new Point3d(10, 20, 30)) * Transform.Translation(100, -200, 300);

        var moved = mesh.DuplicateMesh();
        _ = moved.Transform(move);
        var expected = ConvexHull.Pieces(moved);
        var actual = ConvexHull.Pieces(mesh).Select(hull => hull.Transformed(move)).ToArray();

        Assert.Multiple(() =>
        {
            Assert.That(actual, Has.Length.EqualTo(expected.Length));

            for (int i = 0; i < 50; i++)
            {
                Vector3d direction = new(random.NextDouble() - 0.5, random.NextDouble() - 0.5, random.NextDouble() - 0.5);
                Assert.That(actual[0].Support(direction).DistanceTo(expected[0].Support(direction)), Is.LessThan(1e-3), "Mesh vertices are stored as floats.");
            }
        });
    }

    [Test]
    public void JointRouterGoesAroundAnObstacle()
    {
        Interval[] ranges = [.. Enumerable.Repeat(new Interval(-PI, PI), 6)];
        double[] start = [-1, 0, 0, 0, 0, 0];
        double[] goal = [1, 0, 0, 0, 0, 0];

        JointRouter router = new(IsFree, ranges, 0.01, 1);
        var path = router.Find(start, goal, []);

        Assert.That(path, Is.Not.Null);
        Assert.Multiple(() =>
        {
            Assert.That(path![0], Is.EqualTo(start));
            Assert.That(path[^1], Is.EqualTo(goal));
            Assert.That(path, Has.Count.InRange(3, 5), "Shortened to a few waypoints.");

            for (int i = 1; i < path.Count; i++)
            {
                for (int k = 0; k <= 1000; k++)
                {
                    var joints = path[i - 1].Zip(path[i], (a, b) => a + ((b - a) * k / 1000)).ToArray();
                    Assert.That(IsFree(joints), Is.True, $"Motion {i} is clear.");
                }
            }
        });

        // A wall across the first joint, which the second joint has to lift over.
        static bool IsFree(double[] joints)
        {
            return Abs(joints[0]) > 0.3 || joints[1] > 1;
        }
    }

    [Test]
    public void JointRouterReusesEarlierRoutes()
    {
        Interval[] ranges = [.. Enumerable.Repeat(new Interval(-PI, PI), 6)];
        int checks = 0;

        bool IsFree(double[] joints)
        {
            checks++;
            return Abs(joints[0]) > 0.3 || joints[1] > 1;
        }

        var first = new JointRouter(IsFree, ranges, 0.01, 1).Find([-1, 0, 0, 0, 0, 0], [1, 0, 0, 0, 0, 0], [])!;

        // Back again from a little further along.
        double[] start = [1.2, 0.1, 0, 0, 0, 0];
        double[] goal = [-1.1, -0.1, 0, 0, 0, 0];

        checks = 0;
        var fresh = new JointRouter(IsFree, ranges, 0.01, 2).Find(start, goal, []);
        int freshChecks = checks;

        checks = 0;
        var reused = new JointRouter(IsFree, ranges, 0.01, 2).Find(start, goal, [first]);

        Assert.Multiple(() =>
        {
            Assert.That(fresh, Is.Not.Null);
            Assert.That(reused, Is.Not.Null);
            Assert.That(checks, Is.LessThan(freshChecks), "Joining an earlier route checks fewer poses than searching again.");
        });
    }

    [Test]
    public void CreateProgramKeepsClearPickAndPlaces()
    {
        var cell = Cell.Create(parts: 3);

        var kept = cell.Program(cell.Options with { Avoid = false });
        var avoided = cell.Program(cell.Options);

        Assert.Multiple(() =>
        {
            Assert.That(avoided.Errors, Is.Empty);
            Assert.That(avoided.Warnings, Has.None.Contains("to avoid collisions"));
            Assert.That(avoided.Code, Is.EqualTo(kept.Code));
        });
    }

    sealed record Cell(RobotSystem System, Target Start, CartesianTarget Template, Plane[] Picks, Plane[] Places, Mesh[] Parts, CollisionOptions Options)
    {
        // A CRX-10iA/L raised 713 mm on a pedestal over a table, with a WSG 50 picking struts from a row and stacking them.
        public static Cell Create(int parts = 10)
        {
            var root = new DirectoryInfo(TestContext.CurrentContext.TestDirectory);

            while (!File.Exists(Path.Combine(root.FullName, "Robots.slnx")))
                root = root.Parent ?? throw new DirectoryNotFoundException("Repository root not found.");

            var libraries = Path.Combine(root.FullName, "src", "Robots.Grasshopper", "Resources", "Package", "libraries");
            var crx = XDocument.Load(Path.Combine(libraries, "Fanuc_CRX.xml")).Root!
                .Elements("RobotCell").Single(e => e.Attribute("name")!.Value == "Fanuc-CRX-10iA/L").ToString();
            using var crxMeshes = File3dm.Read(Path.Combine(libraries, "Fanuc_CRX.3dm"));
            using var wsgMeshes = File3dm.Read(Path.Combine(libraries, "WSG50.3dm"));
            var tool = FileIO.ParseTool(XDocument.Load(Path.Combine(libraries, "WSG50.xml")).Root!.Element("Tool")!.ToString(), wsgMeshes);
            var system = FileIO.ParseRobotSystem(crx, new Plane(new Point3d(0, 0, 713), Vector3d.XAxis, Vector3d.YAxis), crxMeshes);
            var robot = ((IndustrialSystem)system).MechanicalGroups[0].Robot;

            CartesianTarget template = new(Plane.WorldXY, tool: tool, speed: new(translation: 300), zone: new(10));
            JointTarget start = new([.. new double[] { 0, 0, 0, 0, -90, 0 }.Select(robot.DegreeToRadian)], tool);
            Plane[] picks = [.. Enumerable.Range(0, parts).Select(i => new Plane(new Point3d(-708.9, -824.1 + (80 * i), 15), Down.XAxis, Down.YAxis))];
            Plane[] places = [.. Enumerable.Range(0, parts).Select(i => new Plane(new Point3d(850, -300, 15 + (28 * i)), Down.XAxis, Down.YAxis))];

            // 300 long along the gripper's X, 41.4 wide between the jaws and 28 high, gripped 5 mm above the middle.
            Mesh[] struts = [.. picks.Select(pick => Box(pick, new(-150, 150), new(-20.7, 20.7), new(-9, 19)))];
            Mesh pedestal = Box(new(-130, 130), new(-130, 130), new(0, 468));
            Mesh table = Box(new(-1200, 1200), new(-1200, 1200), new(-30, -10));

            return new(system, start, template, picks, places, struts, new() { Environment = [pedestal, table] });
        }

        // A joint move to a point 200 mm above the plane.
        public CartesianTarget Above(Plane plane)
        {
            return new(new Plane(plane.Origin + (Vector3d.ZAxis * 200), plane.XAxis, plane.YAxis), Template);
        }

        public Program Program(CollisionOptions? options = null)
        {
            List<Target> targets = [Start];

            for (int i = 0; i < Picks.Length; i++)
                targets.AddRange(PickAndPlace.Create(Picks[i], Places[i], Template, 41.4, 100, 100, Parts[i], null, 0.5, open: 60));

            return new("Cell", System, [new SimpleToolpath(targets)], collisions: options);
        }
    }

    // A post between the pick and place.
    static Mesh Post(double height) => Box(new(-150, 150), new(-1000, -850), new(0, height));

    static Mesh Box(Interval x, Interval y, Interval z) => Box(Plane.WorldXY, x, y, z);

    static Mesh Box(Plane plane, Interval x, Interval y, Interval z)
    {
        Mesh mesh = new();
        Point3d[] corners = [.. Enumerable.Range(0, 8).Select(i =>
            plane.PointAt(i % 2 == 0 ? x.T0 : x.T1, i / 2 % 2 == 0 ? y.T0 : y.T1) + (plane.ZAxis * (i < 4 ? z.T0 : z.T1)))];
        int[][] faces = [[0, 2, 3, 1], [4, 5, 7, 6], [0, 1, 5, 4], [2, 6, 7, 3], [0, 4, 6, 2], [1, 3, 7, 5]];

        // Unwelded, with four vertices per face, as boxes often are.
        foreach (var face in faces)
        {
            int first = mesh.Vertices.Count;

            foreach (int corner in face)
                _ = mesh.Vertices.Add(corners[corner]);

            _ = mesh.Faces.AddFace(first, first + 1, first + 2, first + 3);
        }

        return mesh;
    }

    static Plane RandomPlane(Random random)
    {
        var x = RandomDirection(random);
        var y = Vector3d.CrossProduct(x, RandomDirection(random));
        _ = y.Unitize();
        return new((Point3d)(RandomDirection(random) * 500), x, y);
    }

    static Vector3d RandomDirection(Random random)
    {
        double z = (random.NextDouble() * 2) - 1;
        double angle = random.NextDouble() * 2 * PI;
        double r = Sqrt(1 - (z * z));
        return new(r * Cos(angle), r * Sin(angle), z);
    }
}
