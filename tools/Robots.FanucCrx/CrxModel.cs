using System.Globalization;
using System.Xml.Linq;
using Rhino.Geometry;

namespace Robots.FanucCrx;

record CrxDefinition(string Urdf, string Model, double Payload);

/// <summary>
/// A CRX arm read from its urdf macro. Lengths are meters and angles radians,
/// as in the URDF; <see cref="ToXml"/> converts to the library units.
/// </summary>
class CrxModel
{
    const double Tolerance = 1e-12;
    static readonly string[] Links = ["base_link", "J1_link", "J2_link", "J3_link", "J4_link", "J5_link", "J6_link"];

    // CRX joint axes at the zero pose, from J1 to J6.
    static readonly Vector3d[] Axes =
    [
        Vector3d.ZAxis, Vector3d.YAxis, -Vector3d.YAxis, -Vector3d.XAxis, -Vector3d.YAxis, -Vector3d.XAxis
    ];

    public CrxDefinition Definition { get; }
    public UrdfJoint[] Joints { get; }
    public Transform Flange { get; }
    public Transform[] LinkFrames { get; }
    public double BaseHeight { get; }
    public double UpperArm { get; }
    public double Forearm { get; }
    public double WristOffset { get; }
    public double FlangeOffset { get; }
    public UrdfMesh[] Visual { get; }
    public UrdfMesh[] Collision { get; }

    public CrxModel(CrxDefinition definition, string descriptionPath)
    {
        Definition = definition;
        var xacro = new Xacro(Path.Combine(descriptionPath, "urdf", $"{definition.Urdf}_urdf_macro.xacro"));
        Joints = [.. Enumerable.Range(1, 6).Select(i => xacro.Joint($"J{i}"))];
        Flange = xacro.Joint("J6-flange").Origin;

        for (int i = 0; i < 6; i++)
        {
            var joint = Joints[i];

            if (joint.Parent != Links[i] || joint.Child != Links[i + 1])
                throw new InvalidDataException($"{definition.Urdf} J{i + 1} must connect {Links[i]} to {Links[i + 1]}.");

            if (!joint.Axis.EpsilonEquals(Axes[i], Tolerance))
                throw new InvalidDataException($"{definition.Urdf} J{i + 1} axis is {joint.Axis}, expected {Axes[i]}.");
        }

        // The CRX chain is a sequence of pure translations at the zero pose.
        BaseHeight = Offset(0, Vector3d.ZAxis);
        _ = Offset(1, Vector3d.Unset);
        UpperArm = Offset(2, Vector3d.ZAxis);
        _ = Offset(3, Vector3d.Unset);
        Forearm = Offset(4, Vector3d.XAxis);
        WristOffset = -Offset(5, Vector3d.YAxis);
        FlangeOffset = Translation(Flange, "J6-flange", Vector3d.XAxis);

        LinkFrames = new Transform[Links.Length];
        LinkFrames[0] = Transform.Identity;

        for (int i = 0; i < 6; i++)
            LinkFrames[i + 1] = LinkFrames[i] * Joints[i].Origin;

        Visual = [.. Links.Select(link => xacro.Mesh(link, "visual"))];
        Collision = [.. Links.Select(link => xacro.Mesh(link, "collision"))];

        double Offset(int index, Vector3d direction)
        {
            return Translation(Joints[index].Origin, Joints[index].Name, direction);
        }
    }

    public string CellName => $"Fanuc-{Definition.Model}";

    /// <summary>
    /// Flange frame of the URDF chain with its origin at the J2 height,
    /// for joint values in radians.
    /// </summary>
    public Transform Forward(double[] joints)
    {
        var transform = Transform.Translation(0, 0, -BaseHeight);

        for (int i = 0; i < 6; i++)
            transform *= Joints[i].Origin * Transform.Rotation(joints[i], Joints[i].Axis, Point3d.Origin);

        return transform * Flange;
    }

    public XElement ToXml()
    {
        double[][] dh =
        [
            [0, 0], [UpperArm, 0], [0, 0], [0, Forearm], [0, WristOffset], [0, FlangeOffset]
        ];

        return new XElement("RobotCell",
            new XAttribute("name", CellName),
            new XAttribute("manufacturer", "Fanuc"),
            new XElement("Mechanisms",
                new XElement("RobotArm",
                    new XAttribute("model", Definition.Model),
                    new XAttribute("manufacturer", "Fanuc"),
                    new XAttribute("payload", Format(Definition.Payload)),
                    new XElement("Base",
                        new XAttribute("x", "0"), new XAttribute("y", "0"), new XAttribute("z", "0"),
                        new XAttribute("q1", "1"), new XAttribute("q2", "0"), new XAttribute("q3", "0"), new XAttribute("q4", "0")),
                    new XElement("Joints", Joints.Select((joint, i) => new XElement("Revolute",
                        new XAttribute("number", i + 1),
                        new XAttribute("a", Format(dh[i][0] * 1000)),
                        new XAttribute("d", Format(dh[i][1] * 1000)),
                        new XAttribute("minrange", Format(Degrees(joint.Lower))),
                        new XAttribute("maxrange", Format(Degrees(joint.Upper))),
                        new XAttribute("maxspeed", Format(Degrees(joint.Velocity)))))))));
    }

    /// <summary>
    /// Mesh placed at the zero pose in millimeters, relative to the J2 height.
    /// </summary>
    public Mesh PlaceMesh(UrdfMesh mesh, int link, string descriptionPath)
    {
        var result = MeshReader.Read(Path.Combine(descriptionPath, mesh.File));
        var placement = Transform.Scale(Point3d.Origin, 1000)
            * Transform.Translation(0, 0, -BaseHeight)
            * LinkFrames[link]
            * mesh.Origin
            * Transform.Scale(Point3d.Origin, mesh.Scale);

        _ = result.Transform(placement);
        _ = result.Normals.ComputeNormals();
        _ = result.Compact();
        return result;
    }

    static double Translation(Transform transform, string name, Vector3d direction)
    {
        var rotation = transform;
        rotation.M03 = rotation.M13 = rotation.M23 = 0;

        if (!rotation.Equals(Transform.Identity))
            throw new InvalidDataException($"Joint '{name}' must not be rotated at the zero pose.");

        Vector3d translation = new(transform.M03, transform.M13, transform.M23);

        if (direction == Vector3d.Unset)
        {
            return translation.Length < Tolerance
                ? 0
                : throw new InvalidDataException($"Joint '{name}' must have no offset, found {translation}.");
        }

        double length = translation * direction;

        if ((translation - length * direction).Length > Tolerance)
            throw new InvalidDataException($"Joint '{name}' offset {translation} must be along {direction}.");

        return length;
    }

    static double Degrees(double radians) => radians * 180 / Math.PI;

    static string Format(double value) =>
        Math.Round(value, 6).ToString("0.######", CultureInfo.InvariantCulture);
}
