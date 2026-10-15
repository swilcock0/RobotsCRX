using System.Xml.Linq;
using Rhino.FileIO;
using Rhino.Geometry;

namespace Robots;

static class TargetAttributeParser
{
    public static Tool ParseTool(string xml, File3dm document) => ParseTool(XElement.Parse(xml), document);

    public static Tool ParseTool(XElement element, File3dm document)
    {
        string type = element.Name.LocalName;

        if (type != "Tool")
            throw new ArgumentException($"Element '{type}' should be 'Tool'.");

        string name = element.GetString("name");
        var plane = element.GetElement("Tcp").ToPlane();
        var mass = element.GetElement("Mass");
        double weight = mass.GetDoubleAttribute("weight");
        var centroid = mass.ToPointOrNull();
        bool useController = element.GetBoolOrDefault("useController");
        int? number = element.GetIntOrNull("number");
        Mesh mesh = MeshIO.GetToolMesh(document, name);
        Mesh collisionMesh = MeshIO.GetToolCollisionMesh(document, name, mesh);
        var (jaws, opening) = ParseJaws(element, name, document);
        return new(plane, name, weight, centroid, mesh, useController: useController, number: number, collisionMesh: collisionMesh, jaws: jaws, opening: opening);
    }

    /// <summary>
    /// Jaws slide along their direction, with meshes on the Tool.name.Jaw1,
    /// Tool.name.Jaw2... layers, modeled at zero opening.
    /// </summary>
    static (ToolJaw[]? Jaws, Interval? Opening) ParseJaws(XElement element, string name, File3dm document)
    {
        if (element.GetElementOrDefault("Jaws") is not { } jawsElement)
            return (null, null);

        Interval opening = new(jawsElement.GetDoubleAttribute("min"), jawsElement.GetDoubleAttribute("max"));
        var jaws = jawsElement.Elements("Jaw").Select((jaw, i) =>
        {
            var direction = (Vector3d)(jaw.ToPointOrNull() ?? throw new ArgumentException($"Jaw {i + 1} of tool '{name}' needs a direction."));
            var jawName = $"{name}.Jaw{i + 1}";
            var mesh = MeshIO.GetToolMesh(document, jawName);
            return new ToolJaw(mesh, direction, MeshIO.GetToolCollisionMesh(document, jawName, mesh));
        }).ToArray();

        if (jaws.Length == 0)
            throw new ArgumentException($"The Jaws element of tool '{name}' has no Jaw elements.");

        return (jaws, opening);
    }

    public static Frame ParseFrame(XElement element)
    {
        string type = element.Name.LocalName;

        if (type != "Frame")
            throw new ArgumentException($"Element '{type}' should be 'Frame'.");

        string name = element.GetString("name");
        bool useController = element.GetBoolOrDefault("useController");
        int? number = element.GetIntOrNull("number");
        var plane = element.GetElement("Base").ToPlane();
        var coupling = element.GetElementOrDefault("Coupling");
        int mechanism = coupling?.GetIntOrNull("mechanism") ?? -1;
        int group = coupling?.GetIntOrNull("group") ?? -1;
        return new(plane, mechanism, group, name, useController, number);
    }
}
