using Rhino.Geometry;
using Robots.Commands;

namespace Robots;

/// <summary>
/// Makes the pick and places of a single-robot program again to avoid collisions.
/// For each in turn, it tries the tool turned 180 degrees about its Z axis and
/// moving higher between the pick and place, until the targets from the end of the
/// previous pick and place to the end of this one have no errors. Parts placed
/// before are obstacles where they were placed, and parts after where they wait.
/// Of the options without errors, it takes one whose linear motions keep their
/// configuration if there is one, as a changed configuration flips the arm. If
/// none clears, it routes the joint motions that collide around the obstacles
/// through joint waypoints, reusing the routes of earlier pick and places.
/// </summary>
static class PickAndPlacePlanner
{
    const double LiftStep = 100;
    const double TrialStepSize = 5;

    sealed record Run(int Start, PickAndPlaceJob Job, int Count);

    sealed record Option(Target[] Targets, Program Program, double PickRotation, double PlaceRotation, double Lift, int Waypoints = 0);

    public static List<Target> Plan(Program program, IReadOnlyList<Target> targets, Group? initCommands, double stepSize, CollisionOptions options)
    {
        var runs = Runs(targets);

        if (runs.Count == 0)
            return [.. targets];

        double[] lifts = [.. Enumerable.Range(0, (int)Math.Floor((options.MaxLift / LiftStep) + 1e-9) + 1).Select(i => i * LiftStep)];
        var turns = new (double Pick, double Place)[runs.Count];
        List<Target> result = [];
        List<Target> start = [];
        List<IReadOnlyList<double[]>> routes = [];
        Option? previous = null;
        int next = 0;

        // Sampled more coarsely than the program, which checks the motions it takes at its own step size.
        double trialStepSize = Math.Max(stepSize, TrialStepSize);

        for (int r = 0; r < runs.Count; r++)
        {
            var run = runs[r];
            var job = run.Job;
            Target[] gap = [.. targets.Skip(next).Take(run.Start - next)];
            (double, double)[] rotations = job.Symmetric
                ? [(0, 0), (180, 180), (0, 180), (180, 0)]
                : [(0, 0), (180, 180)];
            var others = options.Parts ? OtherParts(r) : [];
            Option? chosen = null;
            Option? fallback = null;
            Option? first = null;
            Dictionary<(double, double, double), Option> tried = [];

            // Unchanged or turned first, then what cleared the previous pick and place, then higher.
            foreach (var (pickRotation, placeRotation) in rotations)
            {
                if (Consider(Try(pickRotation, placeRotation, 0)))
                    break;
            }

            if (chosen is null && previous is { } before)
            {
                if (before.Waypoints == 0)
                    _ = Consider(Try(before.PickRotation, before.PlaceRotation, before.Lift));
                else
                    chosen = Route(Try(before.PickRotation, before.PlaceRotation, 0), r, gap, others);
            }

            foreach (double lift in lifts.Skip(1))
            {
                if (chosen is not null)
                    break;

                bool reached = false;

                foreach (var (pickRotation, placeRotation) in rotations)
                {
                    var option = Try(pickRotation, placeRotation, lift);

                    // Errors other than collisions, such as targets out of reach.
                    reached |= option.Program.Errors.Count == option.Program.Collisions.Count;

                    if (Consider(option))
                        break;
                }

                // Higher is only further out of reach.
                if (!reached)
                    break;
            }

            if (chosen is null && fallback is null)
            {
                foreach (var (pickRotation, placeRotation) in rotations)
                {
                    chosen = Route(Try(pickRotation, placeRotation, 0), r, gap, others);

                    if (chosen is not null)
                        break;
                }
            }

            if (chosen is not null)
                previous = chosen;

            Option Try(double pickRotation, double placeRotation, double lift)
            {
                if (tried.TryGetValue((pickRotation, placeRotation, lift), out var option))
                    return option;

                var created = job.Targets(pickRotation, placeRotation, lift);
                option = new(created, Trial(created, gap, others), pickRotation, placeRotation, lift);
                tried[(pickRotation, placeRotation, lift)] = option;
                first ??= option;
                return option;
            }

            // Whether the option has no errors and keeps its configuration on linear motions, as a changed configuration flips the arm.
            bool Consider(Option option)
            {
                var trial = option.Program;

                if (trial.Errors.Count > 0)
                    return false;

                if (!trial.Targets.Any(target => target.ProgramTargets.Any(p => p.ChangesConfiguration && !p.IsJointMotion)))
                {
                    chosen = option;
                    return true;
                }

                fallback ??= option;
                return false;
            }

            var picked = chosen ?? fallback ?? first!;
            result.AddRange(gap);
            int index = result.Count;
            result.AddRange(picked.Targets);
            next = run.Start + run.Count;
            turns[r] = (picked.PickRotation, picked.PlaceRotation);

            if (chosen is null && fallback is null)
                program.AddWarning($"No turn, lift up to {options.MaxLift:0} mm or route around obstacles clears this pick and place.", index);
            else if (picked.PickRotation != 0 || picked.PlaceRotation != 0 || picked.Lift != 0 || picked.Waypoints != 0)
                program.AddWarning($"Pick and place {Describe(picked)} to avoid collisions.", index);

            if (chosen is null && fallback is not null)
                program.AddWarning("Pick and place clear of collisions only with a linear motion that changes configuration.", index);

            // The next pick and place starts where this one ends, with the jaws as they are.
            var end = picked.Program.Targets[^1].ProgramTargets[0];

            if (picked.Program.Targets.Count != start.Count + gap.Length + picked.Targets.Length || end.Kinematics.Joints is not { Length: > 0 } joints)
                break;

            start = [new JointTarget(joints, end.Target.Tool, frame: end.Target.Frame)];
            initCommands = new([new SetGripper(end.GripperAtTarget.Opening)]);
        }

        result.AddRange(targets.Skip(next));
        return result;

        Program Trial(IEnumerable<Target> created, Target[] gap, List<(Mesh, Plane, string)> others)
        {
            return new("Plan", program.RobotSystem, [new SimpleToolpath([.. start, .. gap, .. created])], initCommands, null, trialStepSize, options with { Avoid = false, FixedParts = others }, false);
        }

        // The option with its colliding joint motions routed around the obstacles, if that clears it.
        Option? Route(Option option, int run, Target[] gap, List<(Mesh, Plane, string)> others)
        {
            var trial = option.Program;
            int offset = start.Count + gap.Length;

            if (trial.Collisions.Count == 0 || trial.Errors.Count > trial.Collisions.Count)
                return null;

            var checker = new CollisionChecker(trial, options with { Avoid = false, FixedParts = others });
            var ranges = checker.Ranges;

            // Planned further from obstacles than the checks, so the poses between those checked are clear too.
            double margin = options.Distance + options.LinearStep;
            List<(int Index, List<double[]> Path, RobotConfigurations Configuration)> found = [];

            foreach (int index in trial.Collisions.Select(contact => contact.TargetIndex).Distinct())
            {
                var to = trial.Targets[index];

                // Only the pick and place's own joint motions can be routed.
                if (index < offset || index == 0 || !to.ProgramTargets[0].IsJointMotion)
                    return null;

                var from = trial.Targets[index - 1].ProgramTargets[0].Kinematics.Joints;
                double resolution = 2 * options.LinearStep / checker.Reach(to);
                JointRouter router = new(joints => !checker.Collides(joints, to, margin), ranges, resolution, run + 1);

                if (router.Find(from, to.ProgramTargets[0].Kinematics.Joints, routes) is not { } path)
                    return null;

                found.Add((index - offset, path, to.ProgramTargets[0].Kinematics.Configuration));
            }

            // Flying by the waypoints if that stays clear, otherwise stopping at them.
            foreach (var zone in new Zone?[] { null, Zone.Default })
            {
                List<Target> routed = [.. option.Targets];
                int waypoints = 0;

                foreach (var (index, path, configuration) in found.OrderByDescending(f => f.Index))
                {
                    var target = option.Targets[index];

                    // Keeping the arm as the route arrives, rather than the solution closest to the last waypoint.
                    if (target is CartesianTarget cartesian)
                        routed[index] = new CartesianTarget(cartesian.Plane, cartesian, configuration, cartesian.Motion) { Step = cartesian.Step };

                    routed.InsertRange(index, path.Skip(1).SkipLast(1).Select(joints => new JointTarget(joints, target.Tool, target.Speed, zone ?? target.Zone, null, target.Frame)));
                    waypoints += path.Count - 2;
                }

                var routedTrial = Trial(routed, gap, others);

                if (routedTrial.Errors.Count == 0)
                {
                    routes.AddRange(found.Select(f => f.Path));
                    return new([.. routed], routedTrial, option.PickRotation, option.PlaceRotation, 0, waypoints);
                }
            }

            return null;
        }

        // Parts placed before this pick and place where they were placed, and parts after it where they wait.
        List<(Mesh Mesh, Plane Frame, string Name)> OtherParts(int current)
        {
            List<(Mesh, Plane, string)> parts = [];

            for (int r = 0; r < runs.Count; r++)
            {
                var job = runs[r].Job;

                if (r < current && job.Placed(turns[r].Pick, turns[r].Place) is { } placed)
                    parts.Add((placed, job.Place, $"part {r} (placed)"));
                else if (r > current && job.Part is { } waiting)
                    parts.Add((waiting, job.Pick, $"part {r} (waiting)"));
            }

            return parts;
        }
    }

    /// <summary>
    /// The pick and places whose targets are all in the program, in order and together.
    /// </summary>
    static List<Run> Runs(IReadOnlyList<Target> targets)
    {
        List<Run> runs = [];

        for (int i = 0; i < targets.Count; i++)
        {
            if (targets[i].Step is not { Index: 0 } step || i + step.Count > targets.Count)
                continue;

            bool whole = true;

            for (int k = 1; k < step.Count; k++)
            {
                if (targets[i + k].Step is not { } other || other.Job != step.Job || other.Index != k)
                {
                    whole = false;
                    break;
                }
            }

            if (!whole)
                continue;

            runs.Add(new(i, step.Job, step.Count));
            i += step.Count - 1;
        }

        return runs;
    }

    static string Describe(Option option)
    {
        List<string> changes = [];

        if (option.PickRotation == option.PlaceRotation && option.PickRotation != 0)
            changes.Add($"turned {option.PickRotation:0}°");
        else if (option.PickRotation != option.PlaceRotation)
            changes.Add($"turned {option.PickRotation:0}° at the pick and {option.PlaceRotation:0}° at the place");

        if (option.Lift != 0)
            changes.Add($"lifted {option.Lift:0} mm");

        if (option.Waypoints != 0)
            changes.Add($"routed through {option.Waypoints} joint waypoint{(option.Waypoints == 1 ? "" : "s")}");

        return string.Join(" and ", changes);
    }
}
