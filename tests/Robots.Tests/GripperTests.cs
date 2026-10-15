using NUnit.Framework;
using Rhino.DocObjects;
using Rhino.FileIO;
using Rhino.Geometry;
using Robots.Commands;

namespace Robots.Tests;

public class GripperTests
{
    const string GripperXml = """
        <Tool name="Gripper">
          <Tcp x="0" y="0" z="100" q1="1" q2="0" q3="0" q4="0"/>
          <Mass weight="1" x="0" y="0" z="50"/>
          <Jaws min="5" max="110">
            <Jaw x="1" y="0" z="0"/>
            <Jaw x="-1" y="0" z="0"/>
          </Jaws>
        </Tool>
        """;

    static readonly double[] Start = [0, 10, -20, 0, -30, 0];
    static readonly double[] End = [20, 25, -5, 15, -45, 30];
    static readonly double[] ExpectedOpenings = [110, 20];
    static readonly Motions[] ExpectedMotions = [Motions.Joint, Motions.Linear, Motions.Linear, Motions.Joint, Motions.Linear, Motions.Linear];

    [Test]
    public void LoadsJawsFromTheLibrary()
    {
        var tool = CreateGripper();

        Assert.Multiple(() =>
        {
            Assert.That(tool.Jaws, Has.Count.EqualTo(2));
            Assert.That(tool.Opening, Is.EqualTo(new Interval(5, 110)));
            Assert.That(tool.Jaws[1].Direction, Is.EqualTo(-Vector3d.XAxis));
            Assert.That(tool.Mesh.Vertices, Has.Count.EqualTo(9));

            // The jaws are modeled at zero opening and shown fully open.
            Assert.That(tool.Mesh.GetBoundingBox(true).Max.X, Is.EqualTo(56).Within(1e-9));
            Assert.That(tool.WithOpening(20).Mesh.GetBoundingBox(true).Max.X, Is.EqualTo(11).Within(1e-9));
        });
    }

    [Test]
    public void SimulationFollowsTheGripperCommands()
    {
        var tool = CreateGripper();
        var system = TestRobots.FanucCrx10();
        var program = CreateProgram(system, tool, new SetGripper(20));

        Assert.That(program.Errors, Is.Empty);

        var openings = program.Targets.Select(target => target.ProgramTargets[0].Gripper.Opening).ToArray();
        Assert.That(openings, Is.EqualTo(ExpectedOpenings));

        program.MeshPoser = new RhinoMeshPoser(system);
        program.Animate(1);
        var toolMesh = ((RhinoMeshPoser)program.MeshPoser).Meshes[^1];
        var flange = program.CurrentSimulationPose.Kinematics[0].Planes[^2];
        var width = toolMesh.Vertices.ToPoint3dArray().Max(point => flange.XAxis * (point - flange.Origin));

        Assert.That(width, Is.EqualTo(11).Within(1e-3));
    }

    [Test]
    public void CarriesPartsFromPickToRelease()
    {
        var tool = CreateGripper();
        var system = TestRobots.FanucCrx10();
        var robot = ((IndustrialSystem)system).MechanicalGroups[0].Robot;
        var part = Box(-5, 5);
        _ = part.Transform(Transform.Translation(500, 0, 300));
        double[] middle = [10, 20, -10, 5, -40, 10];

        JointTarget pick = new([.. Start.Select(robot.DegreeToRadian)], tool, command: new SetGripper(20, part));
        JointTarget place = new([.. middle.Select(robot.DegreeToRadian)], tool, command: new SetGripper(110));
        JointTarget end = new([.. End.Select(robot.DegreeToRadian)], tool);
        Program program = new("Carry", system, [TestRobots.Toolpath(pick, place, end)]);
        Assert.That(program.Errors, Is.Empty);

        var vertex = (Point3d)part.Vertices[0];
        var pickFlange = Flange(program.Targets[0]);
        var placeFlange = Flange(program.Targets[1]);

        program.Animate(0, isNormalized: false);
        Assert.That(First(program.CurrentParts()[0]).DistanceTo(vertex), Is.LessThan(1e-3), "Gripped where it lies.");

        double halfway = (program.Targets[0].TotalTime + program.Targets[1].TotalTime) / 2;
        program.Animate(halfway, isNormalized: false);
        var flange = program.CurrentSimulationPose.Kinematics[0].Planes[^2];
        Assert.That(First(program.CurrentParts()[0]).DistanceTo(Moved(vertex, pickFlange, flange)), Is.LessThan(1e-3), "Carried by the tool.");

        // Held parts count as part of the tool in collision checks.
        var poser = RhinoMeshPoser.CreateCollision(system);
        poser.Pose(program.CurrentSimulationPose.Kinematics, program.Targets[1]);
        Assert.That(poser.Meshes[^1].Vertices, Has.Count.EqualTo(tool.CollisionMesh.Vertices.Count + part.Vertices.Count));

        program.Animate(1);
        Assert.That(First(program.CurrentParts()[0]).DistanceTo(Moved(vertex, pickFlange, placeFlange)), Is.LessThan(1e-3), "Left at the release.");

        static Plane Flange(SystemTarget target)
        {
            return target.ProgramTargets[0].Kinematics.Planes[^2];
        }

        static Point3d First(Mesh mesh)
        {
            return mesh.Vertices[0];
        }

        static Point3d Moved(Point3d point, Plane from, Plane to)
        {
            point.Transform(Transform.PlaneToPlane(from, to));
            return point;
        }
    }

    [Test]
    public void PicksAndPlacesAPart()
    {
        var tool = CreateGripper();
        var system = TestRobots.FanucCrx10();
        var robot = ((IndustrialSystem)system).MechanicalGroups[0].Robot;
        var pick = Tcp(Start);
        var place = Tcp(End);
        var part = Box(-5, 5);
        _ = part.Transform(Transform.PlaneToPlane(Plane.WorldXY, pick));

        JointTarget template = new(new double[6], tool);
        var targets = PickAndPlace.Create(pick, place, template, 20, approach: 50, retreat: 30, part, wait: 0.5);
        Program program = new("PickAndPlace", system, [TestRobots.Toolpath(targets)]);

        Assert.That(program.Errors, Is.Empty);

        program.Animate(1);
        var placed = (Point3d)program.CurrentParts()[0].Vertices[0];
        var expected = (Point3d)part.Vertices[0];
        expected.Transform(Transform.PlaneToPlane(pick, place));

        Assert.Multiple(() =>
        {
            Assert.That(targets, Has.Length.EqualTo(6));
            Assert.That(((CartesianTarget)targets[0]).Plane.Origin.DistanceTo(pick.Origin - pick.ZAxis * 50), Is.LessThan(1e-9));
            Assert.That(((CartesianTarget)targets[5]).Plane.Origin.DistanceTo(place.Origin - place.ZAxis * 30), Is.LessThan(1e-9));
            Assert.That(targets.Select(target => ((CartesianTarget)target).Motion), Is.EqualTo(ExpectedMotions));
            Assert.That(placed.DistanceTo(expected), Is.LessThan(1e-3));
        });

        Plane Tcp(double[] degrees)
        {
            return system.Kinematics([new JointTarget([.. degrees.Select(robot.DegreeToRadian)], tool)])[0].Planes[^1];
        }
    }

    [Test]
    public void RejectsPickingWhileHolding()
    {
        var part = Box(0, 1);
        var system = TestRobots.FanucCrx10();
        var robot = ((IndustrialSystem)system).MechanicalGroups[0].Robot;
        var tool = CreateGripper();
        JointTarget first = new([.. Start.Select(robot.DegreeToRadian)], tool, command: new SetGripper(20, part));
        JointTarget second = new([.. End.Select(robot.DegreeToRadian)], tool, command: new SetGripper(20, part));
        Program program = new("Twice", system, [TestRobots.Toolpath(first, second)]);

        Assert.That(program.Errors, Has.Some.Contains("already holding one"));
    }

    [Test]
    public void GripperChangesBeforeTheWait()
    {
        var tool = CreateGripper();
        var system = TestRobots.FanucCrx10();
        var robot = ((IndustrialSystem)system).MechanicalGroups[0].Robot;
        JointTarget start = new([.. Start.Select(robot.DegreeToRadian)], tool);
        JointTarget grasp = new([.. End.Select(robot.DegreeToRadian)], tool, command: new Robots.Commands.Group([new SetGripper(20), new Wait(1)]));
        Program program = new("Wait", system, [TestRobots.Toolpath(start, grasp)])
        {
            MeshPoser = new RhinoMeshPoser(system)
        };
        double arrival = program.Targets[1].TotalTime - 1;

        Assert.Multiple(() =>
        {
            Assert.That(Width(arrival - 0.1), Is.EqualTo(56).Within(1e-3), "Open while moving.");
            Assert.That(Width(arrival + 0.5), Is.EqualTo(11).Within(1e-3), "Closed during the wait.");
        });

        double Width(double time)
        {
            program.Animate(time, isNormalized: false);
            var toolMesh = ((RhinoMeshPoser)program.MeshPoser).Meshes[^1];
            var flange = program.CurrentSimulationPose.Kinematics[0].Planes[^2];
            return toolMesh.Vertices.ToPoint3dArray().Max(point => flange.XAxis * (point - flange.Origin));
        }
    }

    [Test]
    public void GripperCommandsAddNoCode()
    {
        var tool = CreateGripper();
        var system = TestRobots.FanucCrx10();
        var withCommand = CreateProgram(system, tool, new SetGripper(20));
        var withoutCommand = CreateProgram(system, tool, null);

        Assert.That(TestRobots.FlattenCode(withCommand), Is.EqualTo(TestRobots.FlattenCode(withoutCommand)));
    }

    [TestCase(120, "outside the 5 to 110 mm range")]
    [TestCase(2, "outside the 5 to 110 mm range")]
    public void RejectsOpeningsOutsideTheRange(double opening, string message)
    {
        var program = CreateProgram(TestRobots.FanucCrx10(), CreateGripper(), new SetGripper(opening));

        Assert.That(program.Errors, Has.Some.Contains(message));
    }

    [Test]
    public void RejectsGripperCommandsWithoutJaws()
    {
        var program = CreateProgram(TestRobots.FanucCrx10(), Tool.Default, new SetGripper(20));

        Assert.That(program.Errors, Has.Some.Contains("needs a tool with jaws"));
    }

    static Program CreateProgram(RobotSystem system, Tool tool, Command? command)
    {
        var robot = ((IndustrialSystem)system).MechanicalGroups[0].Robot;
        JointTarget start = new([.. Start.Select(robot.DegreeToRadian)], tool, command: command);
        JointTarget end = new([.. End.Select(robot.DegreeToRadian)], tool);
        return new("Gripper", system, [TestRobots.Toolpath(start, end)]);
    }

    static Tool CreateGripper()
    {
        using File3dm doc = new();
        AddMesh(doc, "Tool.Gripper", Box(-5, 5));
        AddMesh(doc, "Tool.Gripper.Jaw1", Box(0, 1));
        AddMesh(doc, "Tool.Gripper.Jaw2", Box(-1, 0));
        return FileIO.ParseTool(GripperXml, doc);
    }

    static Mesh Box(double minX, double maxX)
    {
        Mesh mesh = new();
        _ = mesh.Vertices.Add(minX, 0, 0);
        _ = mesh.Vertices.Add(maxX, 0, 0);
        _ = mesh.Vertices.Add(minX, 1, 0);
        _ = mesh.Faces.AddFace(0, 1, 2);
        return mesh;
    }

    static void AddMesh(File3dm doc, string layerName, Mesh mesh)
    {
        doc.AllLayers.Add(new Layer { Name = layerName });
        _ = doc.Objects.AddMesh(mesh, new ObjectAttributes { LayerIndex = doc.AllLayers.Count - 1 });
    }
}
