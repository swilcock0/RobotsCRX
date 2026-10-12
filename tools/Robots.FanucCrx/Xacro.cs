using System.Globalization;
using System.Text.RegularExpressions;
using System.Xml.Linq;
using Rhino.Geometry;

namespace Robots.FanucCrx;

record UrdfJoint(string Name, string Parent, string Child, Transform Origin, Vector3d Axis, double Lower, double Upper, double Velocity);

record UrdfMesh(string File, Transform Origin, double Scale);

/// <summary>
/// Expands the subset of xacro used by fanuc_crx_description's urdf macros:
/// properties, <c>${prefix}</c>, and arithmetic expressions with <c>radians()</c>.
/// </summary>
partial class Xacro
{
    static readonly XNamespace XacroNamespace = "http://wiki.ros.org/xacro";

    readonly Dictionary<string, string> _properties = [];
    readonly Dictionary<string, double> _values = [];
    readonly XElement _macro;

    public Xacro(string file)
    {
        var root = XElement.Load(file);
        var macros = root.Elements(XacroNamespace + "macro").ToArray();

        if (macros.Length != 1)
            throw new InvalidDataException($"'{file}' must contain one xacro macro.");

        _macro = macros[0];

        foreach (var property in _macro.Elements(XacroNamespace + "property"))
            _properties.Add(Attribute(property, "name"), Attribute(property, "value"));
    }

    public UrdfJoint Joint(string name) =>
        ReadJoint(_macro.Elements("joint").SingleOrDefault(e => Name(e) == name)
            ?? throw new InvalidDataException($"Joint '{name}' was not found."));

    public UrdfMesh Mesh(string link, string kind)
    {
        var element = _macro.Elements("link").SingleOrDefault(e => Name(e) == link)?.Element(kind)
            ?? throw new InvalidDataException($"Link '{link}' has no {kind} element.");

        var mesh = element.Element("geometry")?.Element("mesh")
            ?? throw new InvalidDataException($"Link '{link}' {kind} geometry is not a mesh.");

        var scale = Vector(mesh.Attribute("scale")?.Value ?? "1 1 1");

        if (scale.X != scale.Y || scale.Y != scale.Z)
            throw new InvalidDataException($"Link '{link}' {kind} mesh has a non-uniform scale.");

        const string package = "package://fanuc_crx_description/";
        string file = Attribute(mesh, "filename");

        if (!file.StartsWith(package, StringComparison.Ordinal))
            throw new InvalidDataException($"Unexpected mesh location '{file}'.");

        return new(file[package.Length..], Origin(element), scale.X);
    }

    UrdfJoint ReadJoint(XElement joint)
    {
        var limit = joint.Element("limit");
        var axis = joint.Element("axis");

        return new(
            Name(joint),
            Expand(Attribute(joint.Element("parent")!, "link")),
            Expand(Attribute(joint.Element("child")!, "link")),
            Origin(joint),
            axis is null ? Vector3d.Unset : Vector(Attribute(axis, "xyz")),
            limit is null ? double.NaN : Number(Attribute(limit, "lower")),
            limit is null ? double.NaN : Number(Attribute(limit, "upper")),
            limit is null ? double.NaN : Number(Attribute(limit, "velocity")));
    }

    string Name(XElement element) => Expand(Attribute(element, "name"));

    Transform Origin(XElement element)
    {
        var origin = element.Element("origin")
            ?? throw new InvalidDataException($"'{element.Name}' has no origin.");

        var xyz = Vector(Attribute(origin, "xyz"));
        var rpy = Vector(Attribute(origin, "rpy"));

        // URDF rpy is extrinsic X, Y, Z.
        return Transform.Translation(xyz)
            * Transform.Rotation(rpy.Z, Vector3d.ZAxis, Point3d.Origin)
            * Transform.Rotation(rpy.Y, Vector3d.YAxis, Point3d.Origin)
            * Transform.Rotation(rpy.X, Vector3d.XAxis, Point3d.Origin);
    }

    Vector3d Vector(string text)
    {
        var values = Expand(text).Split(' ', StringSplitOptions.RemoveEmptyEntries)
            .Select(ParseNumber)
            .ToArray();

        return values.Length == 3
            ? new(values[0], values[1], values[2])
            : throw new InvalidDataException($"'{text}' is not a 3D vector.");
    }

    double Number(string text) => ParseNumber(Expand(text));

    string Expand(string text) => SubstitutionRegex().Replace(text, match =>
    {
        string expression = match.Groups[1].Value.Trim();
        return expression == "prefix"
            ? ""
            : Evaluate(expression).ToString("R", CultureInfo.InvariantCulture);
    });

    double Value(string name)
    {
        if (_values.TryGetValue(name, out double value))
            return value;

        if (!_properties.TryGetValue(name, out var text))
            throw new InvalidDataException($"Unknown xacro property '{name}'.");

        value = Number(text);
        _values.Add(name, value);
        return value;
    }

    double Evaluate(string expression)
    {
        var tokens = TokenRegex().Matches(expression).Select(m => m.Value).ToArray();

        if (string.Concat(tokens) != expression.Replace(" ", "", StringComparison.Ordinal))
            throw new InvalidDataException($"Unsupported xacro expression '{expression}'.");

        int position = 0;
        double result = Sum();

        if (position != tokens.Length)
            throw new InvalidDataException($"Unsupported xacro expression '{expression}'.");

        return result;

        double Sum()
        {
            double value = Product();

            while (position < tokens.Length && tokens[position] is "+" or "-")
                value = tokens[position++] == "+" ? value + Product() : value - Product();

            return value;
        }

        double Product()
        {
            double value = Unary();

            while (position < tokens.Length && tokens[position] is "*" or "/")
                value = tokens[position++] == "*" ? value * Unary() : value / Unary();

            return value;
        }

        double Unary()
        {
            if (position < tokens.Length && tokens[position] == "-")
            {
                position++;
                return -Unary();
            }

            return Primary();
        }

        double Primary()
        {
            string token = tokens[position++];

            if (token == "(")
                return Close(Sum());

            if (token == "radians")
            {
                Expect("(");
                return Close(Sum()) * Math.PI / 180;
            }

            if (token == "pi")
                return Math.PI;

            return char.IsLetter(token[0]) || token[0] == '_'
                ? Value(token)
                : ParseNumber(token);
        }

        double Close(double value)
        {
            Expect(")");
            return value;
        }

        void Expect(string token)
        {
            if (position >= tokens.Length || tokens[position++] != token)
                throw new InvalidDataException($"Expected '{token}' in xacro expression '{expression}'.");
        }
    }

    static double ParseNumber(string text) =>
        double.Parse(text, NumberStyles.Float, CultureInfo.InvariantCulture);

    static string Attribute(XElement element, string name) =>
        element.Attribute(name)?.Value
            ?? throw new InvalidDataException($"'{element.Name}' has no '{name}' attribute.");

    [GeneratedRegex(@"\$\{([^}]*)\}")]
    private static partial Regex SubstitutionRegex();

    [GeneratedRegex(@"\d*\.?\d+(?:[eE][-+]?\d+)?|[A-Za-z_]\w*|[-+*/()]")]
    private static partial Regex TokenRegex();
}
