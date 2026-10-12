using System.Globalization;
using System.Xml.Linq;
using Rhino.Geometry;

namespace Robots.FanucCrx;

/// <summary>
/// Reads the triangle meshes used by fanuc_crx_description: COLLADA visual
/// meshes and binary STL collision meshes. Vertices are returned in meters.
/// </summary>
static class MeshReader
{
    static readonly XNamespace Collada = "http://www.collada.org/2005/11/COLLADASchema";

    public static Mesh Read(string file) => Path.GetExtension(file).ToLowerInvariant() switch
    {
        ".dae" => ReadCollada(file),
        ".stl" => ReadBinaryStl(file),
        _ => throw new InvalidDataException($"Unsupported mesh format '{file}'.")
    };

    static Mesh ReadCollada(string file)
    {
        var root = XElement.Load(file);
        var asset = root.Element(Collada + "asset");
        double meters = double.Parse(
            asset?.Element(Collada + "unit")?.Attribute("meter")?.Value ?? "1",
            CultureInfo.InvariantCulture);

        string upAxis = asset?.Element(Collada + "up_axis")?.Value ?? "Y_UP";

        if (upAxis != "Z_UP")
            throw new InvalidDataException($"'{file}' must be Z up.");

        var scene = root.Element(Collada + "library_visual_scenes")?.Elements(Collada + "visual_scene").Single()
            ?? throw new InvalidDataException($"'{file}' has no visual scene.");

        Mesh mesh = new();

        foreach (var node in scene.Elements(Collada + "node"))
            AddNode(node, Transform.Scale(Point3d.Origin, meters));

        if (mesh.Faces.Count == 0)
            throw new InvalidDataException($"'{file}' has no triangles.");

        return mesh;

        void AddNode(XElement node, Transform parent)
        {
            var transform = parent;

            foreach (var element in node.Elements())
            {
                if (element.Name == Collada + "matrix")
                {
                    transform *= Matrix(element.Value);
                }
                else if (element.Name.LocalName is "translate" or "rotate" or "scale" or "lookat" or "skew")
                {
                    throw new InvalidDataException($"'{file}' uses an unsupported node transform '{element.Name.LocalName}'.");
                }
            }

            foreach (var instance in node.Elements(Collada + "instance_geometry"))
            {
                var part = Geometry(root, instance.Attribute("url")!.Value);
                _ = part.Transform(transform);
                mesh.Append(part);
            }

            foreach (var child in node.Elements(Collada + "node"))
                AddNode(child, transform);
        }
    }

    static Mesh Geometry(XElement root, string url)
    {
        var geometry = root.Descendants(Collada + "geometry").Single(g => "#" + g.Attribute("id")!.Value == url);
        var meshElement = geometry.Element(Collada + "mesh")!;

        if (meshElement.Elements().Any(e => e.Name.LocalName is "polylist" or "polygons" or "tristrips" or "trifans"))
            throw new InvalidDataException($"Geometry '{url}' contains non-triangle primitives.");

        var vertices = meshElement.Element(Collada + "vertices")!;
        string positionSource = vertices.Elements(Collada + "input")
            .Single(input => input.Attribute("semantic")!.Value == "POSITION")
            .Attribute("source")!.Value;

        var positions = Floats(meshElement.Elements(Collada + "source")
            .Single(source => "#" + source.Attribute("id")!.Value == positionSource)
            .Element(Collada + "float_array")!.Value);

        Mesh mesh = new();

        for (int i = 0; i < positions.Length; i += 3)
            _ = mesh.Vertices.Add(positions[i], positions[i + 1], positions[i + 2]);

        foreach (var triangles in meshElement.Elements(Collada + "triangles"))
        {
            var inputs = triangles.Elements(Collada + "input").ToArray();
            int stride = inputs.Max(input => int.Parse(input.Attribute("offset")!.Value, CultureInfo.InvariantCulture)) + 1;
            int vertexOffset = int.Parse(
                inputs.Single(input => input.Attribute("semantic")!.Value == "VERTEX").Attribute("offset")!.Value,
                CultureInfo.InvariantCulture);

            var indices = triangles.Element(Collada + "p")!.Value
                .Split([' ', '\n', '\r', '\t'], StringSplitOptions.RemoveEmptyEntries)
                .Select(text => int.Parse(text, CultureInfo.InvariantCulture))
                .ToArray();

            int count = int.Parse(triangles.Attribute("count")!.Value, CultureInfo.InvariantCulture);

            if (indices.Length != count * 3 * stride)
                throw new InvalidDataException($"Geometry '{url}' has an inconsistent triangle index count.");

            for (int i = 0; i < indices.Length; i += 3 * stride)
            {
                _ = mesh.Faces.AddFace(
                    indices[i + vertexOffset],
                    indices[i + stride + vertexOffset],
                    indices[i + 2 * stride + vertexOffset]);
            }
        }

        return mesh;
    }

    static Mesh ReadBinaryStl(string file)
    {
        using var reader = new BinaryReader(File.OpenRead(file));
        _ = reader.ReadBytes(80);
        uint count = reader.ReadUInt32();

        if (reader.BaseStream.Length != 84 + 50L * count)
            throw new InvalidDataException($"'{file}' is not a binary STL file.");

        Mesh mesh = new();
        Dictionary<(float, float, float), int> indices = [];

        for (uint i = 0; i < count; i++)
        {
            _ = reader.ReadBytes(12);
            int a = Vertex();
            int b = Vertex();
            int c = Vertex();
            _ = reader.ReadUInt16();

            if (a != b && b != c && a != c)
                _ = mesh.Faces.AddFace(a, b, c);
        }

        return mesh;

        int Vertex()
        {
            var key = (reader.ReadSingle(), reader.ReadSingle(), reader.ReadSingle());

            if (!indices.TryGetValue(key, out int index))
            {
                index = mesh.Vertices.Add(key.Item1, key.Item2, key.Item3);
                indices.Add(key, index);
            }

            return index;
        }
    }

    static Transform Matrix(string text)
    {
        var values = Floats(text);

        if (values.Length != 16)
            throw new InvalidDataException("A COLLADA matrix must have 16 values.");

        Transform transform = default;

        for (int row = 0; row < 4; row++)
        {
            for (int column = 0; column < 4; column++)
                transform[row, column] = values[row * 4 + column];
        }

        return transform;
    }

    static double[] Floats(string text) =>
        [.. text.Split([' ', '\n', '\r', '\t'], StringSplitOptions.RemoveEmptyEntries)
            .Select(value => double.Parse(value, CultureInfo.InvariantCulture))];
}
