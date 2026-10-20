using Rhino.Geometry;

namespace Robots;

/// <summary>
/// Finds a joint motion path around collisions with RRT-Connect, and shortens it so
/// it goes straight between as few waypoints as it can instead of wandering. Routes
/// found before are tried first, joined to the new start and goal, as later moves in
/// a cell usually go around the same obstacles.
/// </summary>
/// <param name="isFree">Whether the robot is clear of collisions with these joints.</param>
/// <param name="ranges">The joints' ranges, in radians.</param>
/// <param name="resolution">The largest sum of joint changes, in radians, between the poses checked along a straight joint motion.</param>
/// <param name="seed">Seed of the random samples, so the same cell gets the same path.</param>
sealed class JointRouter(Func<double[], bool> isFree, Interval[] ranges, double resolution, int seed)
{
    const double Step = 0.3;
    const int MaxIterations = 3000;
    const int ShortcutIterations = 150;

    readonly record struct Node(double[] Joints, int Parent);

    readonly Random _random = new(seed);

    /// <summary>
    /// A path from the start to the goal, including both, or null if none was found.
    /// </summary>
    /// <param name="routes">Paths found before, tried first in either direction.</param>
    public List<double[]>? Find(double[] start, double[] goal, IReadOnlyList<IReadOnlyList<double[]>> routes)
    {
        if (IsFree(start, goal))
            return [start, goal];

        if (!isFree(start) || !isFree(goal))
            return null;

        var path = Reuse(start, goal, routes) ?? Connect(start, goal);
        return path is null ? null : Shorten(path);
    }

    /// <summary>
    /// The start, a stretch of a route found before, and the goal, if every motion between them is clear.
    /// </summary>
    List<double[]>? Reuse(double[] start, double[] goal, IReadOnlyList<IReadOnlyList<double[]>> routes)
    {
        for (int r = routes.Count - 1; r >= 0; r--)
        {
            foreach (var route in new IReadOnlyList<double[]>[] { routes[r], [.. routes[r].Reverse()] })
            {
                for (int i = 0; i < route.Count; i++)
                {
                    if (!isFree(route[i]) || !IsFree(start, route[i]))
                        continue;

                    for (int j = route.Count - 1; j >= i; j--)
                    {
                        if (!IsFree(route[j], goal))
                            continue;

                        bool clear = true;

                        for (int k = i; k < j && clear; k++)
                            clear = IsFree(route[k], route[k + 1]);

                        if (clear)
                            return [start, .. route.Skip(i).Take(j - i + 1), goal];
                    }
                }
            }
        }

        return null;
    }

    List<double[]>? Connect(double[] start, double[] goal)
    {
        List<Node> a = [new(start, -1)];
        List<Node> b = [new(goal, -1)];
        bool aIsStart = true;

        for (int iteration = 0; iteration < MaxIterations; iteration++)
        {
            if (Extend(a, Sample()) is int added)
            {
                int? reached = null;
                var target = a[added].Joints;

                while (Extend(b, target) is int next)
                {
                    if (Distance(b[next].Joints, target) < 1e-9)
                    {
                        reached = next;
                        break;
                    }
                }

                if (reached is int joined)
                {
                    var (fromStart, fromGoal) = aIsStart ? (Trace(a, added), Trace(b, joined)) : (Trace(b, joined), Trace(a, added));
                    fromGoal.Reverse();
                    return [.. fromStart, .. fromGoal.Skip(1)];
                }
            }

            (a, b) = (b, a);
            aIsStart = !aIsStart;
        }

        return null;
    }

    int? Extend(List<Node> tree, double[] target)
    {
        int nearest = 0;
        double best = double.MaxValue;

        for (int i = 0; i < tree.Count; i++)
        {
            double distance = Distance(tree[i].Joints, target);

            if (distance < best)
            {
                best = distance;
                nearest = i;
            }
        }

        var from = tree[nearest].Joints;
        var to = best <= Step ? target : Lerp(from, target, Step / best);

        if (!isFree(to) || !IsFree(from, to))
            return null;

        tree.Add(new(to, nearest));
        return tree.Count - 1;
    }

    static List<double[]> Trace(List<Node> tree, int index)
    {
        List<double[]> path = [];

        for (int i = index; i >= 0; i = tree[i].Parent)
            path.Add(tree[i].Joints);

        path.Reverse();
        return path;
    }

    /// <summary>
    /// Skips the waypoints between random pairs of waypoints it can go straight
    /// between, then from each waypoint goes straight to the furthest it can, so
    /// the path keeps few waypoints rather than wandering through many.
    /// </summary>
    List<double[]> Shorten(List<double[]> path)
    {
        for (int iteration = 0; iteration < ShortcutIterations && path.Count > 2; iteration++)
        {
            int a = _random.Next(path.Count);
            int b = _random.Next(path.Count);

            if (a > b)
                (a, b) = (b, a);

            if (b - a > 1 && IsFree(path[a], path[b]))
                path = [.. path.Take(a + 1), .. path.Skip(b)];
        }

        List<double[]> result = [path[0]];

        for (int i = 0; i < path.Count - 1;)
        {
            int j = path.Count - 1;

            while (j > i + 1 && !IsFree(path[i], path[j]))
                j--;

            result.Add(path[j]);
            i = j;
        }

        return result;
    }

    /// <summary>
    /// Whether a straight joint motion is clear, checking the middle first.
    /// </summary>
    bool IsFree(double[] from, double[] to)
    {
        double sum = 0;

        for (int i = 0; i < from.Length; i++)
            sum += Math.Abs(to[i] - from[i]);

        int divisions = (int)Math.Ceiling(sum / resolution);
        int span = 1;

        while (span < divisions)
            span *= 2;

        for (int s = span / 2; s >= 1; s /= 2)
        {
            for (int i = s; i < divisions; i += 2 * s)
            {
                if (!isFree(Lerp(from, to, (double)i / divisions)))
                    return false;
            }
        }

        return true;
    }

    double[] Sample()
    {
        var joints = new double[ranges.Length];

        for (int i = 0; i < joints.Length; i++)
            joints[i] = ranges[i].ParameterAt(_random.NextDouble());

        return joints;
    }

    static double[] Lerp(double[] from, double[] to, double t)
    {
        var joints = new double[from.Length];

        for (int i = 0; i < joints.Length; i++)
            joints[i] = from[i] + ((to[i] - from[i]) * t);

        return joints;
    }

    static double Distance(double[] a, double[] b)
    {
        double sum = 0;

        for (int i = 0; i < a.Length; i++)
            sum += (a[i] - b[i]) * (a[i] - b[i]);

        return Math.Sqrt(sum);
    }
}
