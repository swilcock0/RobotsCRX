using System.Runtime.CompilerServices;
using Rhino.Geometry;
using static System.Math;

namespace Robots;

/// <summary>
/// The points of a convex shape, for fast collision checks. Only the points on
/// its hull are kept, found as the points furthest out in many directions.
/// </summary>
sealed class ConvexHull
{
    static readonly ConditionalWeakTable<Mesh, ConvexHull[]> _pieces = [];
    static readonly Vector3d[] _directions = Directions(2048);

    public Point3d[] Points { get; }
    public Point3d Center { get; }
    public double Radius { get; }

    ConvexHull(Point3d[] points)
    {
        Points = points;
        var box = new BoundingBox(points);
        Center = box.Center;
        Radius = points.Max(point => point.DistanceTo(Center));
    }

    /// <summary>
    /// The hulls of the separate pieces of a mesh, in its own coordinates. Faces
    /// that share a vertex position belong to the same piece, so unwelded meshes
    /// still form whole pieces.
    /// </summary>
    public static ConvexHull[] Pieces(Mesh mesh) => _pieces.GetValue(mesh, CreatePieces);

    public static ConvexHull FromPoints(IEnumerable<Point3d> points) => new(Extremes([.. points]));

    /// <summary>
    /// The hull moved by a transform, which is much quicker than finding the hull of a moved mesh.
    /// </summary>
    public ConvexHull Transformed(Transform transform) => new([.. Points.Select(point =>
    {
        var moved = point;
        moved.Transform(transform);
        return moved;
    })]);

    /// <summary>
    /// The hull with its points in the coordinates of a plane.
    /// </summary>
    public ConvexHull InPlane(Plane plane) => new([.. Points.Select(point =>
    {
        var v = point - plane.Origin;
        return new Point3d(v * plane.XAxis, v * plane.YAxis, v * plane.ZAxis);
    })]);

    /// <summary>
    /// The hull shrunk by a distance along the axes of a plane, about the middle of
    /// its extent. A box with faces along the axes moves each face in by the distance.
    /// </summary>
    public ConvexHull Shrunk(Plane plane, double distance)
    {
        if (distance == 0)
            return this;

        var local = InPlane(plane).Points;
        var box = new BoundingBox(local);
        var middle = box.Center;
        var size = box.Diagonal;

        return new([.. local.Select(point =>
        {
            double x = middle.X + ((point.X - middle.X) * Scale(size.X));
            double y = middle.Y + ((point.Y - middle.Y) * Scale(size.Y));
            double z = middle.Z + ((point.Z - middle.Z) * Scale(size.Z));
            return plane.Origin + (plane.XAxis * x) + (plane.YAxis * y) + (plane.ZAxis * z);
        })]);

        double Scale(double extent)
        {
            return extent <= 2 * distance ? 0 : (extent - (2 * distance)) / extent;
        }
    }

    /// <summary>
    /// The point furthest in a direction given in the hull's coordinates.
    /// </summary>
    public Point3d Support(Vector3d direction)
    {
        var points = Points;
        int best = 0;
        double max = double.NegativeInfinity;

        for (int i = 0; i < points.Length; i++)
        {
            var p = points[i];
            double dot = (p.X * direction.X) + (p.Y * direction.Y) + (p.Z * direction.Z);

            if (dot > max)
            {
                max = dot;
                best = i;
            }
        }

        return points[best];
    }

    static ConvexHull[] CreatePieces(Mesh mesh)
    {
        int vertexCount = mesh.Vertices.Count;

        if (vertexCount == 0)
            return [];

        // Vertices at the same position are one, so pieces are found by position.
        Dictionary<(long, long, long), int> positions = [];
        var ids = new int[vertexCount];

        for (int i = 0; i < vertexCount; i++)
        {
            var v = mesh.Vertices[i];
            var key = (Key(v.X), Key(v.Y), Key(v.Z));

            if (!positions.TryGetValue(key, out int id))
            {
                id = positions.Count;
                positions.Add(key, id);
            }

            ids[i] = id;
        }

        var parents = Enumerable.Range(0, positions.Count).ToArray();

        for (int i = 0; i < mesh.Faces.Count; i++)
        {
            var face = mesh.Faces[i];
            Union(ids[face.A], ids[face.B]);
            Union(ids[face.A], ids[face.C]);

            if (face.IsQuad)
                Union(ids[face.A], ids[face.D]);
        }

        Dictionary<int, List<Point3d>> pieces = [];
        HashSet<int> added = [];

        for (int i = 0; i < vertexCount; i++)
        {
            if (!added.Add(ids[i]))
                continue;

            int root = Find(ids[i]);

            if (!pieces.TryGetValue(root, out var points))
            {
                points = [];
                pieces.Add(root, points);
            }

            points.Add(mesh.Vertices[i]);
        }

        return [.. pieces.Values.Select(points => new ConvexHull(Extremes([.. points])))];

        int Find(int i)
        {
            while (parents[i] != i)
                i = parents[i] = parents[parents[i]];

            return i;
        }

        void Union(int a, int b)
        {
            a = Find(a);
            b = Find(b);

            if (a != b)
                parents[Max(a, b)] = Min(a, b);
        }

        static long Key(float value)
        {
            return (long)Math.Round(value * 1e4);
        }
    }

    /// <summary>
    /// The points furthest out in many directions. A hull point that is missed
    /// lies within about 0.1% of the hull's size of the hull of the points kept.
    /// </summary>
    static Point3d[] Extremes(Point3d[] points)
    {
        if (points.Length <= 32)
            return points;

        HashSet<int> kept = [];

        foreach (var direction in _directions)
        {
            int best = 0;
            double max = double.NegativeInfinity;

            for (int i = 0; i < points.Length; i++)
            {
                double dot = (Vector3d)points[i] * direction;

                if (dot > max)
                {
                    max = dot;
                    best = i;
                }
            }

            _ = kept.Add(best);
        }

        return [.. kept.Order().Select(i => points[i])];
    }

    /// <summary>
    /// Directions spread evenly over the sphere, on a Fibonacci spiral.
    /// </summary>
    static Vector3d[] Directions(int count)
    {
        var directions = new Vector3d[count];
        double golden = PI * (3 - Sqrt(5));

        for (int i = 0; i < count; i++)
        {
            double z = 1 - ((2.0 * i + 1) / count);
            double r = Sqrt(1 - (z * z));
            double angle = golden * i;
            directions[i] = new(r * Cos(angle), r * Sin(angle), z);
        }

        return directions;
    }
}

/// <summary>
/// Gilbert-Johnson-Keerthi checks of whether two convex hulls come within a distance.
/// </summary>
static class Gjk
{
    const int MaxIterations = 64;

    /// <summary>
    /// True if the hulls, with their points in the coordinates of their planes,
    /// come closer than the margin.
    /// </summary>
    public static bool Within(ConvexHull a, in Plane frameA, ConvexHull b, in Plane frameB, double margin)
    {
        double margin2 = margin * margin;
        var v = ToWorld(frameA, a.Center) - ToWorld(frameB, b.Center);

        if (v.SquareLength < 1e-18)
            v = Vector3d.XAxis;

        Span<Vector3d> simplex = stackalloc Vector3d[4];
        int count = 0;

        for (int iteration = 0; iteration < MaxIterations; iteration++)
        {
            // The point of the difference of the hulls furthest towards the origin.
            var w = Support(a, frameA, -v) - Support(b, frameB, v);
            double vv = v.SquareLength;
            double vw = v * w;

            // The plane through w normal to v keeps the origin at least vw / |v| away.
            if (vw > 0 && vw * vw > margin2 * vv)
                return false;

            // No progress, so v is the closest point.
            if (vv - vw <= 1e-12 * Max(vv, 1))
                return vv < margin2;

            simplex[count++] = w;
            v = Closest(simplex, ref count);

            // The origin is inside the difference, so the hulls overlap.
            if (count == 4)
                return true;

            // The difference holds v, so the hulls are at most |v| apart.
            if (v.SquareLength < margin2)
                return true;
        }

        return v.SquareLength < margin2;
    }

    static Point3d ToWorld(in Plane frame, Point3d point) =>
        frame.Origin + (frame.XAxis * point.X) + (frame.YAxis * point.Y) + (frame.ZAxis * point.Z);

    static Vector3d Support(ConvexHull hull, in Plane frame, Vector3d direction)
    {
        var local = new Vector3d(direction * frame.XAxis, direction * frame.YAxis, direction * frame.ZAxis);
        return (Vector3d)ToWorld(frame, hull.Support(local));
    }

    /// <summary>
    /// The point of the simplex closest to the origin. The simplex is reduced to
    /// the fewest points whose hull holds that point, and keeps all four if the
    /// origin is inside it.
    /// </summary>
    static Vector3d Closest(Span<Vector3d> s, ref int count)
    {
        switch (count)
        {
            case 1:
                return s[0];
            case 2:
                {
                    var v = Segment(s[0], s[1], out int mask);
                    Keep(s, ref count, mask);
                    return v;
                }
            case 3:
                {
                    var v = Triangle(s[0], s[1], s[2], out int mask);
                    Keep(s, ref count, mask);
                    return v;
                }
            default:
                return Tetrahedron(s, ref count);
        }
    }

    static Vector3d Tetrahedron(Span<Vector3d> s, ref int count)
    {
        ReadOnlySpan<int> faces = [0, 1, 2, 3, 0, 1, 3, 2, 0, 2, 3, 1, 1, 2, 3, 0];
        var best = Vector3d.Zero;
        double bestLength = double.PositiveInfinity;
        int bestMask = 0;
        bool inside = true;

        for (int f = 0; f < 16; f += 4)
        {
            Vector3d a = s[faces[f]], b = s[faces[f + 1]], c = s[faces[f + 2]], d = s[faces[f + 3]];
            var normal = Vector3d.CrossProduct(b - a, c - a);
            double origin = -(a * normal);
            double opposite = (d - a) * normal;

            // The origin is on the other side of the face from the fourth point.
            if (origin * opposite >= 0 && opposite != 0)
                continue;

            inside = false;
            var v = Triangle(a, b, c, out int mask);
            double length = v.SquareLength;

            if (length < bestLength)
            {
                bestLength = length;
                best = v;
                bestMask = 0;

                for (int k = 0; k < 3; k++)
                {
                    if ((mask & (1 << k)) != 0)
                        bestMask |= 1 << faces[f + k];
                }
            }
        }

        if (inside)
            return Vector3d.Zero;

        Keep(s, ref count, bestMask);
        return best;
    }

    static Vector3d Segment(Vector3d a, Vector3d b, out int mask)
    {
        var ab = b - a;
        double length = ab.SquareLength;
        double t = length == 0 ? 0 : -(a * ab) / length;

        if (t <= 0)
        {
            mask = 1;
            return a;
        }

        if (t >= 1)
        {
            mask = 2;
            return b;
        }

        mask = 3;
        return a + (ab * t);
    }

    /// <summary>
    /// The point of a triangle closest to the origin, from Ericson's
    /// Real-Time Collision Detection, section 5.1.5.
    /// </summary>
    static Vector3d Triangle(Vector3d a, Vector3d b, Vector3d c, out int mask)
    {
        var ab = b - a;
        var ac = c - a;
        double d1 = -(ab * a);
        double d2 = -(ac * a);

        if (d1 <= 0 && d2 <= 0)
        {
            mask = 1;
            return a;
        }

        double d3 = -(ab * b);
        double d4 = -(ac * b);

        if (d3 >= 0 && d4 <= d3)
        {
            mask = 2;
            return b;
        }

        double vc = (d1 * d4) - (d3 * d2);

        if (vc <= 0 && d1 >= 0 && d3 <= 0)
        {
            mask = 3;
            return a + (ab * (d1 / (d1 - d3)));
        }

        double d5 = -(ab * c);
        double d6 = -(ac * c);

        if (d6 >= 0 && d5 <= d6)
        {
            mask = 4;
            return c;
        }

        double vb = (d5 * d2) - (d1 * d6);

        if (vb <= 0 && d2 >= 0 && d6 <= 0)
        {
            mask = 5;
            return a + (ac * (d2 / (d2 - d6)));
        }

        double va = (d3 * d6) - (d5 * d4);

        if (va <= 0 && (d4 - d3) >= 0 && (d5 - d6) >= 0)
        {
            mask = 6;
            return b + ((c - b) * ((d4 - d3) / (d4 - d3 + (d5 - d6))));
        }

        double sum = va + vb + vc;

        // A degenerate triangle is closest along one of its edges.
        if (sum <= 1e-300)
        {
            var p = Segment(a, b, out int m1);
            var q = Segment(b, c, out int m2);
            var r = Segment(a, c, out int m3);
            m2 <<= 1;
            m3 = (m3 & 1) | ((m3 & 2) << 1);

            if (p.SquareLength <= q.SquareLength && p.SquareLength <= r.SquareLength)
            {
                mask = m1;
                return p;
            }

            if (q.SquareLength <= r.SquareLength)
            {
                mask = m2;
                return q;
            }

            mask = m3;
            return r;
        }

        mask = 7;
        double denominator = 1 / sum;
        return a + (ab * (vb * denominator)) + (ac * (vc * denominator));
    }

    static void Keep(Span<Vector3d> s, ref int count, int mask)
    {
        int kept = 0;

        for (int i = 0; i < count; i++)
        {
            if ((mask & (1 << i)) != 0)
                s[kept++] = s[i];
        }

        count = kept;
    }
}
