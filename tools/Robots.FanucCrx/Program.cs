using System.Xml.Linq;
using Rhino.DocObjects;
using Rhino.FileIO;
using Rhino.Geometry;
using Robots;
using Robots.FanucCrx;

// Converts fanuc_description's fanuc_crx_description package into a Robots
// library (Fanuc_CRX.xml and Fanuc_CRX.3dm), then reloads the library and checks
// it against the URDF joint chain.
// Usage: Robots.FanucCrx <fanuc_crx_description folder> [output folder]

CrxDefinition[] definitions =
[
    new("crx3ia", "CRX-3iA", 3),
    new("crx5ia", "CRX-5iA", 5),
    new("crx10ia", "CRX-10iA", 10),
    new("crx10ia_l", "CRX-10iA/L", 10),
    new("crx10ia_lp", "CRX-10iA/L Paint", 10),
    new("crx20ia_l", "CRX-20iA/L", 20),
    new("crx30ia", "CRX-30iA", 30),
];

try
{
    if (args.Length is < 1 or > 2)
        throw new ArgumentException("Usage: Robots.FanucCrx <fanuc_crx_description folder> [output folder]");

    string descriptionPath = Path.GetFullPath(args[0]);
    string outputPath = Path.GetFullPath(args.Length > 1 ? args[1] : Path.Combine("src", "Robots.Grasshopper", "Resources", "Package", "libraries"));
    _ = Directory.CreateDirectory(outputPath);

    var models = definitions.Select(definition => new CrxModel(definition, descriptionPath)).ToArray();
    string xmlPath = Path.Combine(outputPath, "Fanuc_CRX.xml");
    string meshPath = Path.ChangeExtension(xmlPath, ".3dm");

    new XElement("RobotSystems", models.Select(model => model.ToXml())).Save(xmlPath);
    WriteMeshes(models, descriptionPath, meshPath);
    Console.WriteLine($"Wrote {xmlPath}");
    Console.WriteLine($"Wrote {meshPath}");

    Validate(models, xmlPath, meshPath);
    return 0;
}
catch (Exception exception)
{
    Console.Error.WriteLine(exception);
    return 1;
}

static void WriteMeshes(CrxModel[] models, string descriptionPath, string path)
{
    using File3dm doc = new();
    doc.Settings.ModelUnitSystem = Rhino.UnitSystem.Millimeters;

    foreach (var model in models)
    {
        string name = $"RobotArm.Fanuc.{model.Definition.Model}";
        AddLayers(name, model.Visual);
        AddLayers($"{name}.Collision", model.Collision);

        void AddLayers(string parentName, UrdfMesh[] meshes)
        {
            var parent = AddLayer(parentName, Guid.Empty);

            for (int i = 0; i < meshes.Length; i++)
            {
                var layer = AddLayer(i.ToString(System.Globalization.CultureInfo.InvariantCulture), parent.Id);
                var mesh = model.PlaceMesh(meshes[i], i, descriptionPath);
                _ = doc.Objects.AddMesh(mesh, new ObjectAttributes { LayerIndex = layer.Index });
            }
        }
    }

    if (!doc.Write(path, 8))
        throw new IOException($"Could not write '{path}'.");

    Layer AddLayer(string name, Guid parent)
    {
        doc.AllLayers.Add(new Layer { Name = name, ParentLayerId = parent });
        return doc.AllLayers.FindIndex(doc.AllLayers.Count - 1);
    }
}

static void Validate(CrxModel[] models, string xmlPath, string meshPath)
{
    var library = XElement.Load(xmlPath);
    using var meshDoc = File3dm.Read(meshPath);
    Random random = new(1);
    const int samples = 200;

    foreach (var model in models)
    {
        var cell = library.Elements("RobotCell").Single(e => e.Attribute("name")!.Value == model.CellName);
        var system = (IndustrialSystem)FileIO.ParseRobotSystem(cell.ToString(), Plane.WorldXY, meshDoc);
        var robot = system.MechanicalGroups[0].Robot;
        double maxPosition = 0, maxAxis = 0, maxIkPosition = 0;
        Dictionary<string, int> ikErrors = [];

        for (int sample = 0; sample < samples; sample++)
        {
            var joints = robot.Joints.Select(joint => joint.Range.T0 + random.NextDouble() * joint.Range.Length).ToArray();
            // URDF joint values are controller degrees.
            var degrees = joints.Select(robot.RadianToDegree).ToArray();
            var urdf = model.Forward([.. degrees.Select(d => d * Math.PI / 180)]);
            var flange = robot.Kinematics(new JointTarget(joints)).Planes[^1];

            var origin = Point3d.Origin;
            origin.Transform(urdf);
            origin *= 1000;
            var normal = Vector3d.XAxis;
            normal.Transform(urdf);
            var xAxis = -Vector3d.ZAxis;
            xAxis.Transform(urdf);

            maxPosition = Math.Max(maxPosition, origin.DistanceTo(flange.Origin));
            maxAxis = Math.Max(maxAxis, Math.Max((normal - flange.Normal).Length, (xAxis - flange.XAxis).Length));

            var inverse = robot.Kinematics(new CartesianTarget(flange, motion: Motions.Joint), joints);

            if (inverse.Errors.Count > 0)
            {
                foreach (var error in inverse.Errors)
                    ikErrors[error] = ikErrors.GetValueOrDefault(error) + 1;

                continue;
            }

            var check = robot.Kinematics(new JointTarget(inverse.Joints)).Planes[^1];
            maxIkPosition = Math.Max(maxIkPosition, check.Origin.DistanceTo(flange.Origin));
        }

        Console.WriteLine(
            $"{model.Definition.Model,-12} DH a2={model.UpperArm * 1000} d4={model.Forearm * 1000} d5={model.WristOffset * 1000} d6={model.FlangeOffset * 1000}"
            + $" | FK vs URDF: {maxPosition:0.0e0} mm, {maxAxis:0.0e0} axis | IK: {maxIkPosition:0.0e0} mm, {ikErrors.Values.Sum()}/{samples} with errors"
            + string.Concat(ikErrors.Select(pair => $" [{pair.Value}x {pair.Key}]")));

        if (maxPosition > 1e-6 || maxAxis > 1e-9)
            throw new InvalidOperationException($"{model.Definition.Model} forward kinematics does not match the URDF.");

        var display = robot.Joints.Select(joint => joint.Mesh).Prepend(robot.BaseMesh).ToArray();
        var collision = robot.Joints.Select(joint => joint.CollisionMesh).Prepend(robot.BaseCollisionMesh).ToArray();

        for (int i = 0; i < display.Length; i++)
        {
            if (display[i].Faces.Count == 0 || collision[i].Faces.Count == 0)
                throw new InvalidOperationException($"{model.Definition.Model} mesh {i} is empty.");

            var a = display[i].GetBoundingBox(true);
            var b = collision[i].GetBoundingBox(true);
            double offset = a.Center.DistanceTo(b.Center);
            Console.WriteLine($"    mesh {i}: {display[i].Faces.Count,6} faces, display/collision box centers {offset,6:0.0} mm apart, display box {a.Diagonal.Length:0} mm");
        }
    }
}
