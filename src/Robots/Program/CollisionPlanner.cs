using Rhino.Geometry;
using Robots.Commands;
using Key = (double Pick, double Place, double Lift, Robots.RobotConfigurations? PickArm, Robots.RobotConfigurations? PlaceArm);

namespace Robots;

/// <summary>
/// Changes a single-robot program to avoid collisions. Each pick and place is made
/// again in turn, trying the tool turned 180 degrees about its Z axis and moving
/// higher between the pick and place, lowest first, until the targets from the end
/// of the previous pick and place to the end of this one have no errors. Parts placed
/// before are obstacles where they were placed, and parts after where they wait.
/// Of the options without errors, it takes one whose linear motions keep their
/// configuration if there is one, as a changed configuration flips the arm. If
/// none clears, it routes the joint motions that collide around the obstacles
/// through joint waypoints, reusing the routes of earlier moves. Finally, any
/// other joint moves of the program that collide are routed the same way, while
/// linear moves are kept as they are.
/// </summary>
static class CollisionPlanner
{
    /// <summary>
    /// The planned targets, and the notes on what was changed, by target index, which the
    /// program warns of if the targets made at its step size have no errors besides collisions.
    /// </summary>
    public sealed record Planned(List<Target> Targets, List<(int Index, string Message)> Notes);

    const double LiftStep = 100;
    const double TrialStepSize = 5;

    // The lifts, from none, tried with the arm in another configuration, which rarely needs a lift too: up to 300 mm.
    const int AlternativeLifts = 4;

    // The arm configurations a six-axis robot can reach a pose with, such as shoulder, elbow and wrist up or down.
    static readonly RobotConfigurations[] Arms = [.. Enumerable.Range(0, 8).Select(i => (RobotConfigurations)i)];

    sealed record Run(int Start, PickAndPlaceJob Job, int Count);

    sealed record Routed(List<Target> Targets, Program Program, List<(int Original, int Index, int Count, RobotConfigurations? Arm)> Inserted);

    sealed record Option(Target[] Targets, Program Program, double PickRotation, double PlaceRotation, double Lift, RobotConfigurations? PickArm, RobotConfigurations? PlaceArm, int Waypoints = 0);

    public static Planned Plan(Program program, IReadOnlyList<Target> targets, Group? initCommands, double stepSize, CollisionOptions options)
    {
        var runs = Runs(targets);
        var initial = initCommands;

        double[] lifts = [.. Enumerable.Range(0, (int)Math.Floor((options.MaxLift / LiftStep) + 1e-9) + 1).Select(i => i * LiftStep)];
        var turns = new (double Pick, double Place)[runs.Count];
        List<Target> result = [];
        List<Target> start = [];
        List<IReadOnlyList<double[]>> routes = [];
        List<(int Index, string Message)> notes = [];
        int failed = 0;
        bool liftsFailed = false;
        double liftHint = 0;
        int next = 0;

        // Sampled more coarsely than the program, which checks the motions it takes at its own step size.
        double trialStepSize = Math.Max(stepSize, TrialStepSize);

        // The program as given, checked once: most of the time it is clear, and where it is not, only the pick and places it collides in need changing.
        var baseline = new Program("Plan", program.RobotSystem, [new SimpleToolpath(targets)], initial, null, trialStepSize, options with { Avoid = false }, false);

        if (baseline.Errors.Count == 0)
            return new([.. targets], []);

        // Its results hold for a pick and place as long as the targets before it end as they did, which they do while nothing before it is changed.
        bool baselineHolds = baseline.Errors.Count == baseline.Collisions.Count && baseline.Targets.Count == targets.Count;
        bool planned = false;
        bool unresolved = false;

        for (int r = 0; r < runs.Count; r++)
        {
            var run = runs[r];
            var job = run.Job;
            Target[] gap = [.. targets.Skip(next).Take(run.Start - next)];

            if (baselineHolds && !baseline.Collisions.Any(contact => contact.TargetIndex >= run.Start && contact.TargetIndex < run.Start + run.Count))
            {
                result.AddRange(gap);
                result.AddRange(targets.Skip(run.Start).Take(run.Count));
                next = run.Start + run.Count;
                turns[r] = (0, 0);
                (start, initCommands) = Continue(baseline.Targets[next - 1].ProgramTargets[0]);
                continue;
            }

            planned = true;
            (double, double)[] rotations = job.Symmetric
                ? [(0, 0), (180, 180), (0, 180), (180, 0)]
                : [(0, 0), (180, 180)];
            var others = options.Parts ? OtherParts(r) : [];
            Option? chosen = null;
            Option? fallback = null;
            Option? first = null;
            Dictionary<Key, Option> tried = [];
            var given = job.Configuration;
            int offset = start.Count + gap.Length;
            List<(RobotConfigurations? Pick, RobotConfigurations? Place)> arms = [(given, given)];

            // The lowest lift that clears, with the arm as given if it can, then routing around the obstacles with it. If no lift cleared the pick and place
            // before, which the next one probably is like, the lifts are left until after routing, which finds a way quicker than they can all be tried.
            Search(arms, liftsFailed ? 1 : int.MaxValue, liftHint);

            if (chosen is null && fallback is null)
                RouteArms(arms);

            if (chosen is null && fallback is null && liftsFailed)
                Search(arms, hint: liftHint);

            // Then with the arm in another configuration, the same at the pick and place. After two pick and places in a row that nothing clears, which the others
            // probably are too, the rest only try the arm as given, as the other configurations take much longer to try than they are likely to help.
            bool deep = failed < 2;

            if (chosen is null && fallback is null && deep)
            {
                var alternatives = Arms.Where(arm => arm != given).Select(arm => ((RobotConfigurations?)arm, (RobotConfigurations?)arm)).ToArray();
                arms.AddRange(alternatives);
                Search(alternatives, AlternativeLifts);
            }

            // Then a different one at the pick and place, as the arm that clears the pick may not clear the place.
            if (chosen is null && fallback is null && deep)
            {
                var singles = rotations.SelectMany(rotation => arms.Select(arm => Try(rotation.Item1, rotation.Item2, 0, arm.Pick, arm.Place))).ToArray();

                // The pick is the targets before the joint move to the place, and the place the ones after it, which that move is not part of.
                int swing = PickAndPlaceJob.PlaceIndex(0);
                var picks = singles.Where(o => Clear(o, 0, swing)).Select(o => o.PickArm).Distinct().ToArray();
                var places = singles.Where(o => Clear(o, swing + 1, int.MaxValue)).Select(o => o.PlaceArm).Distinct().ToArray();
                var pairs = picks.SelectMany(pick => places.Where(place => place != pick).Select(place => (pick, place))).ToArray();

                arms.AddRange(pairs);

                if (pairs.Length > 0)
                    Search(pairs, AlternativeLifts);
            }

            // Then routing around the obstacles with those.
            if (chosen is null && fallback is null && deep)
                RouteArms(arms.Skip(1));

            failed = chosen is null && fallback is null ? failed + 1 : 0;
            unresolved |= chosen is null && fallback is null;
            liftsFailed = chosen is null ? fallback is null : chosen.Waypoints > 0 || chosen.PickArm != given || chosen.PlaceArm != given;

            if (chosen is { Waypoints: 0 } && !liftsFailed)
                liftHint = chosen.Lift;

            // Routes the joint moves of each option around the obstacles, until one clears.
            void RouteArms(IEnumerable<(RobotConfigurations? Pick, RobotConfigurations? Place)> candidates)
            {
                foreach (var (pickArm, placeArm) in candidates)
                {
                    foreach (var (pickRotation, placeRotation) in rotations)
                    {
                        chosen = Route(Try(pickRotation, placeRotation, 0, pickArm, placeArm), r, gap, others);

                        if (chosen is not null)
                            return;
                    }
                }
            }

            // Whether the option has no collisions or other errors from the first target to the last, counted from the pick and place's first.
            bool Clear(Option option, int from, int to)
            {
                var trial = option.Program;
                return trial.Errors.Count == trial.Collisions.Count
                    && !trial.Collisions.Any(contact => contact.TargetIndex - offset >= from && contact.TargetIndex - offset < to);
            }

            // Lifts from the lowest, unchanged or turned first. Several are tried at once, as the trials are independent. If a pick and place before cleared with a lift,
            // the lifts around it are tried first, and the ones below only if the lowest of those clears, as lifting more clears more.
            void Search(IReadOnlyList<(RobotConfigurations? Pick, RobotConfigurations? Place)> candidates, int liftCount = int.MaxValue, double hint = 0)
            {
                int width = Math.Clamp(Environment.ProcessorCount / (rotations.Length * candidates.Count), 1, 8);

                // The turns and arms that collide on the moves in and out of the pick and place, which no lift changes, so they are only tried unlifted.
                HashSet<(int Rotation, int Candidate)> blocked = [];
                var considered = lifts.Take(liftCount).ToArray();

                double[] window = hint > 0 ? [.. considered.Where(lift => lift > 0 && Math.Abs(lift - hint) <= LiftStep + 1e-9)] : [];
                var rest = considered.Skip(1).ToArray();

                // The unlifted options alone first, as most of the time they clear, unless the one before needed a lift, as this probably does too.
                if (window.Length == 0)
                {
                    Evaluate(KeysFor([considered[0]]));

                    if (BestAt(considered[0], out bool reachable) is { } unlifted)
                    {
                        chosen = unlifted;
                        return;
                    }

                    // Higher is only further out of reach, and nothing is left to lift once every option collides in or out of the pick and place.
                    if (!reachable || blocked.Count == rotations.Length * candidates.Count)
                        return;
                }
                else
                {
                    Evaluate(KeysFor(window));
                    Option? found = null;
                    double foundLift = 0;
                    bool windowReached = false;

                    foreach (double lift in window)
                    {
                        found = BestAt(lift, out bool reached);
                        windowReached |= reached;

                        if (found is not null)
                        {
                            foundLift = lift;
                            break;
                        }
                    }

                    if (found is not null)
                    {
                        if (foundLift == window[0])
                        {
                            double[] lower = [.. considered.Where(lift => lift < window[0])];
                            Evaluate(KeysFor(lower));

                            foreach (double lift in lower)
                            {
                                if (BestAt(lift, out _) is { } below)
                                {
                                    found = below;
                                    break;
                                }
                            }
                        }

                        chosen = found;
                        return;
                    }

                    if (!windowReached)
                        return;

                    rest = [.. rest.Where(lift => lift > window[^1])];
                }

                foreach (var batch in rest.Chunk(width))
                {
                    Evaluate(KeysFor(batch));

                    foreach (double lift in batch)
                    {
                        if (BestAt(lift, out bool reached) is { } best)
                        {
                            chosen = best;
                            return;
                        }

                        if (!reached)
                            return;
                    }
                }

                Key[] KeysFor(IEnumerable<double> batch)
                {
                    return [.. batch.SelectMany(lift => candidates.SelectMany((arm, c) => rotations.Select((rotation, ro) => (ro, c, rotation, arm, lift)))
                        .Where(key => !blocked.Contains((key.ro, key.c)))
                        .Select(key => (key.rotation.Item1, key.rotation.Item2, key.lift, key.arm.Pick, key.arm.Place)))];
                }

                // The option at a lift, unturned first, then of the arms that clear the one whose joints move the least.
                Option? BestAt(double lift, out bool reached)
                {
                    reached = false;

                    for (int ro = 0; ro < rotations.Length; ro++)
                    {
                        Option? best = null;

                        for (int c = 0; c < candidates.Count; c++)
                        {
                            if (blocked.Contains((ro, c)))
                                continue;

                            var option = Try(rotations[ro].Item1, rotations[ro].Item2, lift, candidates[c].Pick, candidates[c].Place);

                            // Errors other than collisions, such as targets out of reach.
                            reached |= option.Program.Errors.Count == option.Program.Collisions.Count;

                            if (lift == lifts[0] && Blocked(option))
                                _ = blocked.Add((ro, c));

                            if (Consider(option) && (best is null || Travel(option) < Travel(best)))
                                best = option;
                        }

                        if (best is not null)
                            return best;
                    }

                    return null;
                }
            }

            // Whether the unlifted option collides on the linear moves to and from the pick and place, which are the same at every lift.
            bool Blocked(Option option)
            {
                return option.Program.Collisions.Any(contact => contact.TargetIndex - offset is 1 or 2 or 4 or 5);
            }

            Option Try(double pickRotation, double placeRotation, double lift, RobotConfigurations? pickArm, RobotConfigurations? placeArm)
            {
                Evaluate([(pickRotation, placeRotation, lift, pickArm, placeArm)]);
                return tried[(pickRotation, placeRotation, lift, pickArm, placeArm)];
            }

            // Makes and checks the options not tried yet, at the same time.
            void Evaluate(Key[] keys)
            {
                var missing = keys.Where(key => !tried.ContainsKey(key)).ToArray();
                var made = new Option[missing.Length];

                _ = Parallel.For(0, missing.Length, i =>
                {
                    var (pickRotation, placeRotation, lift, pickArm, placeArm) = missing[i];
                    var created = job.Targets(pickRotation, placeRotation, lift, pickArm, placeArm);
                    made[i] = new(created, Trial(created, gap, others, lift > 0), pickRotation, placeRotation, lift, pickArm, placeArm);
                });

                for (int i = 0; i < missing.Length; i++)
                {
                    tried[missing[i]] = made[i];
                    first ??= made[i];
                }
            }

            // Whether the option has no errors and keeps its configuration on linear motions, as a changed configuration flips the arm.
            bool Consider(Option option)
            {
                var trial = option.Program;

                if (trial.Errors.Count > 0)
                    return false;

                if (!trial.Targets.Any(target => target.ProgramTargets.Any(p => p.ChangesConfiguration && !p.IsJointMotion)))
                    return true;

                fallback ??= option;
                return false;
            }

            // How far the joints move from the target before the pick and place to its last.
            double Travel(Option option)
            {
                return JointTravel(option.Program, offset, option.Program.Targets.Count);
            }

            var picked = chosen ?? fallback ?? first!;
            result.AddRange(gap);
            int index = result.Count;
            result.AddRange(picked.Targets);
            next = run.Start + run.Count;
            turns[r] = (picked.PickRotation, picked.PlaceRotation);

            if (chosen is null && fallback is null)
                notes.Add((index, $"No turn, lift up to {options.MaxLift:0} mm, other configuration or route around obstacles clears this pick and place."));
            else if (picked.PickRotation != 0 || picked.PlaceRotation != 0 || picked.Lift != 0 || picked.Waypoints != 0 || picked.PickArm != given || picked.PlaceArm != given)
                notes.Add((index, $"Pick and place {Describe(picked, given)} to avoid collisions."));

            if (chosen is null && fallback is not null)
                notes.Add((index, "Pick and place clear of collisions only with a linear motion that changes configuration."));

            // The next pick and place starts where this one ends, with the jaws as they are.
            var end = picked.Program.Targets[^1].ProgramTargets[0];

            if (picked.Program.Targets.Count != start.Count + gap.Length + picked.Targets.Length || end.Kinematics.Joints is not { Length: > 0 })
                break;

            // The program as given holds again if it ends this pick and place with the arm as it was and the part where it was.
            baselineHolds = baselineHolds
                && picked.PickRotation == 0
                && picked.PlaceRotation == 0
                && baseline.Targets[run.Start + run.Count - 1].ProgramTargets[0].Kinematics.Joints.Zip(end.Kinematics.Joints, (a, b) => Math.Abs(a - b)).All(difference => difference < 1e-9);

            (start, initCommands) = Continue(end);
        }

        result.AddRange(targets.Skip(next));

        // Pick and places one after another have each been checked from where the one before ended, so there is nothing else to check or route.
        bool contiguous = runs.Count > 0 && runs.Zip(runs.Skip(1), (a, b) => a.Start + a.Count == b.Start).All(joined => joined) && runs[^1].Start + runs[^1].Count == targets.Count;

        return new(planned && contiguous && !unresolved ? result : RouteOthers(result), notes);

        // The rest of the program: its colliding joint moves routed around the obstacles, then the arm turned to another configuration where that clears the rest.
        List<Target> RouteOthers(List<Target> list)
        {
            var trialOptions = options with { Avoid = false };
            Program Make(IReadOnlyList<Target> all)
            {
                return new("Plan", program.RobotSystem, [new SimpleToolpath(all)], initial, null, trialStepSize, trialOptions, false);
            }

            var trial = planned ? Make(list) : baseline;

            if (trial.Collisions.Count > 0 && RouteAll(trial, list) is { } first)
            {
                list = first.Targets;
                trial = first.Program;
            }

            if (trial.Collisions.Count > 0 && trial.Errors.Count == trial.Collisions.Count && Reconfigure(trial, list, Make) is { } second)
            {
                list = second.Targets;
                trial = second.Program;

                // The joint moves to the changed configurations may need routing too.
                if (trial.Collisions.Count > 0 && RouteAll(trial, list) is { } third)
                    list = third.Targets;
            }

            return list;

            Routed? RouteAll(Program current, List<Target> all)
            {
                if (RouteMoves(current, all, 0, Make, trialOptions, runs.Count, partial: true) is not { } routed)
                    return null;

                Shift(routed.Inserted);

                foreach (var (_, index, count, arm) in routed.Inserted)
                {
                    string configuration = arm is { } changed ? $" in configuration \"{changed}\"" : "";
                    notes.Add((index, count == 0
                        ? $"Joint move{configuration} to avoid collisions."
                        : $"Joint move{configuration} routed through {count} joint waypoint{(count == 1 ? "" : "s")} to avoid collisions."));
                }

                return routed;
            }
        }

        // How far the joints move, in radians, from the target before the first to the last before the end.
        static double JointTravel(Program program, int first, int end)
        {
            var all = program.Targets;
            double travel = 0;

            for (int i = Math.Max(first, 1); i < Math.Min(end, all.Count); i++)
                travel += Distance(all[i - 1].ProgramTargets[0].Kinematics.Joints, all[i].ProgramTargets[0].Kinematics.Joints);

            return travel;
        }

        static double Distance(double[] a, double[] b)
        {
            return Math.Sqrt(a.Zip(b, (x, y) => (x - y) * (x - y)).Sum());
        }

        // Moves the notes after the targets inserted before them.
        void Shift(List<(int Original, int Index, int Count, RobotConfigurations? Arm)> inserted)
        {
            for (int n = 0; n < notes.Count; n++)
                notes[n] = (notes[n].Index + inserted.Where(i => i.Original <= notes[n].Index).Sum(i => i.Count), notes[n].Message);
        }

        // Where a collision is on a linear move, the arm may need to be in another configuration from the joint move before it, which fixes it for the linear moves after.
        Routed? Reconfigure(Program trial, List<Target> list, Func<IReadOnlyList<Target>, Program> make)
        {
            List<Target> changed = [.. list];
            var current = trial;
            bool any = false;

            // The joint move each collision follows from, in order.
            var heads = trial.Collisions.Select(contact => HeadOf(list, contact.TargetIndex)).Where(head => head >= 0).Distinct().Order().ToArray();

            foreach (int head in heads)
            {
                // The targets of a pick and place have tried every configuration already.
                if (changed[head] is not CartesianTarget { Step: null } cartesian || current.Targets.Count != changed.Count)
                    continue;

                int end = EndOf(changed, head);
                var configuration = current.Targets[head].ProgramTargets[0].Kinematics.Configuration;
                var arms = Arms.Where(arm => arm != configuration).ToArray();
                var made = new (List<Target> Targets, Program Program)?[arms.Length];

                _ = Parallel.For(0, arms.Length, i =>
                {
                    List<Target> candidate = [.. changed];
                    candidate[head] = new CartesianTarget(cartesian.Plane, cartesian, arms[i], cartesian.Motion) { Step = cartesian.Step };
                    made[i] = (candidate, make(candidate));
                });

                // Clear after the joint move to it, which routing can deal with, and no worse elsewhere. Of those, the least joint travel.
                (List<Target> Targets, Program Program)? best = null;
                double bestTravel = double.MaxValue;

                foreach (var option in made.OfType<(List<Target> Targets, Program Program)>())
                {
                    var after = option.Program;

                    if (after.Errors.Count != after.Collisions.Count || after.Collisions.Count >= current.Collisions.Count || after.Targets.Count != changed.Count)
                        continue;

                    if (after.Collisions.Any(contact => contact.TargetIndex > head && contact.TargetIndex < end))
                        continue;

                    double travel = JointTravel(after, head, end);

                    if (travel < bestTravel)
                    {
                        best = option;
                        bestTravel = travel;
                    }
                }

                if (best is not { } chosenOption)
                    continue;

                changed = chosenOption.Targets;
                current = chosenOption.Program;
                any = true;
                notes.Add((head, $"Joint move in configuration \"{current.Targets[head].ProgramTargets[0].Kinematics.Configuration}\" to avoid collisions."));
            }

            return any ? new(changed, current, []) : null;
        }

        // The joint move a target follows, which fixes the configuration of the linear moves after it.
        static int HeadOf(List<Target> list, int index)
        {
            for (int i = index; i >= 0; i--)
            {
                if (list[i] is CartesianTarget { Motion: Motions.Joint })
                    return i;

                if (list[i] is JointTarget)
                    return -1;
            }

            return -1;
        }

        // The next joint move after a target.
        static int EndOf(List<Target> list, int head)
        {
            for (int i = head + 1; i < list.Count; i++)
            {
                if (list[i] is JointTarget or CartesianTarget { Motion: Motions.Joint })
                    return i;
            }

            return list.Count;
        }

        // The start of the next pick and place, from where the targets before it end.
        static (List<Target> Start, Group Commands) Continue(ProgramTarget end)
        {
            return ([new JointTarget(end.Kinematics.Joints, end.Target.Tool, frame: end.Target.Frame)], new([new SetGripper(end.GripperAtTarget.Opening)]));
        }

        // Only whether a lifted option collides matters, not where, so its check stops at the first collision.
        Program Trial(IEnumerable<Target> created, Target[] gap, List<FixedPart> others, bool lifted)
        {
            return new("Plan", program.RobotSystem, [new SimpleToolpath([.. start, .. gap, .. created])], initCommands, null, trialStepSize, options with { Avoid = false, FixedParts = others, Any = lifted }, false);
        }

        // The option with its colliding joint motions routed around the obstacles, if that clears it.
        Option? Route(Option option, int run, Target[] gap, List<FixedPart> others)
        {
            int offset = start.Count + gap.Length;
            Program Make(IReadOnlyList<Target> all)
            {
                return new("Plan", program.RobotSystem, [new SimpleToolpath(all)], initCommands, null, trialStepSize, options with { Avoid = false, FixedParts = others }, false);
            }

            if (RouteMoves(option.Program, [.. start, .. gap, .. option.Targets], offset, Make, options with { Avoid = false, FixedParts = others }, run, partial: false) is not { } routed)
                return null;

            return new([.. routed.Targets.Skip(offset)], routed.Program, option.PickRotation, option.PlaceRotation, 0, option.PickArm, option.PlaceArm, routed.Inserted.Sum(i => i.Count));
        }

        // The targets with the colliding joint moves from the first index on routed around the obstacles, if that clears them (or with some left, if partial).
        Routed? RouteMoves(Program trial, IReadOnlyList<Target> list, int first, Func<IReadOnlyList<Target>, Program> make, CollisionOptions trialOptions, int seed, bool partial)
        {
            if (trial.Collisions.Count == 0 || trial.Errors.Count > trial.Collisions.Count || trial.Targets.Count != list.Count)
                return null;

            // Routing cannot help if a collision is on a move that cannot be routed, so do not try, unless the ones that can be are all that is wanted.
            if (!partial && trial.Collisions.Any(contact => contact.TargetIndex < first || contact.TargetIndex == 0 || !trial.Targets[contact.TargetIndex].ProgramTargets[0].IsJointMotion))
                return null;

            var checker = new CollisionChecker(trial, trialOptions);
            var ranges = checker.Ranges;

            // Planned further from obstacles than the checks, so the poses between those checked are clear too.
            double margin = options.Distance + options.LinearStep;
            List<(int Index, List<double[]> Path, RobotConfigurations Configuration, RobotConfigurations? Changed)> found = [];

            foreach (int index in trial.Collisions.Select(contact => contact.TargetIndex).Distinct())
            {
                var to = trial.Targets[index];
                var goal = to.ProgramTargets[0].Kinematics;

                // Only joint moves can be routed, as a linear move has to stay straight.
                if (index < first || index == 0 || !to.ProgramTargets[0].IsJointMotion || goal.Joints is not { Length: > 0 })
                {
                    if (partial)
                        continue;

                    return null;
                }

                var from = trial.Targets[index - 1].ProgramTargets[0].Kinematics.Joints;
                double resolution = 2 * options.LinearStep / checker.Reach(to);
                JointRouter router = new((joints, origin, destination) => checker.Clearance(joints, origin, destination, to, margin), ranges, resolution, seed + 1);

                var path = router.Find(from, goal.Joints, routes);
                var configuration = goal.Configuration;

                // The arm may have to reach the target in another configuration, such as the other way round a wall. Of those that can, the least joint travel.
                if (path is null && list[index] is CartesianTarget cartesian)
                {
                    double shortest = double.MaxValue;

                    foreach (var arm in Arms.Where(arm => arm != goal.Configuration))
                    {
                        var other = program.RobotSystem.Kinematics([new CartesianTarget(cartesian.Plane, cartesian, arm, cartesian.Motion)], [from])[0];

                        if (other.Errors.Count > 0 || other.Joints is not { Length: > 0 } || router.Find(from, other.Joints, routes) is not { } alternative)
                            continue;

                        double length = alternative.Zip(alternative.Skip(1), Distance).Sum();

                        if (length < shortest)
                        {
                            shortest = length;
                            path = alternative;
                            configuration = arm;
                        }
                    }
                }

                if (path is null)
                {
                    if (partial)
                        continue;

                    return null;
                }

                found.Add((index, path, configuration, configuration == goal.Configuration ? null : configuration));
            }

            if (found.Count == 0)
                return null;

            // Flying by the waypoints if that stays clear, otherwise stopping at them.
            foreach (var zone in new Zone?[] { null, Zone.Default })
            {
                List<Target> routed = [.. list];
                List<(int Original, int Index, int Count, RobotConfigurations? Arm)> inserted = [];
                int added = 0;

                foreach (var (index, path, configuration, changedArm) in found)
                {
                    var target = list[index];
                    int target0 = index + added;

                    // Keeping the arm as the route arrives, rather than the solution closest to the last waypoint.
                    if (target is CartesianTarget cartesian)
                        routed[target0] = new CartesianTarget(cartesian.Plane, cartesian, configuration, cartesian.Motion) { Step = cartesian.Step };

                    routed.InsertRange(target0, path.Skip(1).SkipLast(1).Select(joints => new JointTarget(joints, target.Tool, target.Speed, zone ?? target.Zone, null, target.Frame)));
                    added += path.Count - 2;
                    inserted.Add((index, target0 + path.Count - 2, path.Count - 2, changedArm));
                }

                var routedTrial = make(routed);

                if (routedTrial.Errors.Count == 0 || (partial && routedTrial.Collisions.Count < trial.Collisions.Count && routedTrial.Errors.Count <= routedTrial.Collisions.Count))
                {
                    routes.AddRange(found.Select(f => f.Path));
                    return new(routed, routedTrial, inserted);
                }
            }

            return null;
        }

        // Parts placed before this pick and place where they were placed, and parts after it where they wait.
        List<FixedPart> OtherParts(int current)
        {
            List<FixedPart> parts = [];

            for (int r = 0; r < runs.Count; r++)
            {
                var job = runs[r].Job;

                if (r < current && job.Placed(turns[r].Pick, turns[r].Place) is { } placed)
                    parts.Add(new(placed, job.Place, $"part {r} (placed)", job.Part, job.Placement(turns[r].Pick, turns[r].Place)));
                else if (r > current && job.Part is { } waiting)
                    parts.Add(new(waiting, job.Pick, $"part {r} (waiting)"));
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

    static string Name(RobotConfigurations? arm) => arm is { } named ? $"\"{named}\"" : "chosen by the solver";

    static string Describe(Option option, RobotConfigurations? given)
    {
        List<string> changes = [];

        if (option.PickArm == option.PlaceArm && option.PickArm != given)
            changes.Add($"with the arm in configuration {Name(option.PickArm)}");
        else if (option.PickArm != option.PlaceArm)
            changes.Add($"with the arm in configuration {Name(option.PickArm)} at the pick and {Name(option.PlaceArm)} at the place");

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
