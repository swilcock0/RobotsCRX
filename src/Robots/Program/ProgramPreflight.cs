using System.Globalization;

namespace Robots;

class ProgramPreflight(Program program)
{
    readonly Program _program = program;
    readonly RobotSystem _robotSystem = program.RobotSystem;

    public void Run(List<SystemTarget> systemTargets)
    {
        var targets = GetProgramTargets(systemTargets);
        ValidateTargetAxes(targets);
        _robotSystem.PostProcessor.Validate(_program, targets);
        WarnCommandFlybys(targets);
        WarnDefaults(targets);
        WarnPayloads(targets);
        SetGripperOpenings(systemTargets);

        var attributes = CollectAttributes(targets);
        NameAttributes(attributes, targets);

        foreach (var target in targets)
            ValidateFrame(target);

        _program.AddAttributes(attributes);
    }

    static ProgramTarget[] GetProgramTargets(List<SystemTarget> systemTargets) =>
        systemTargets.FlattenToArray(x => x.ProgramTargets);

    /// <summary>
    /// Follows the gripper commands of each robot in program order. The jaws start
    /// fully open, or at the opening set by the init commands, and an opening
    /// carries over to the following targets. A command with a part picks it up,
    /// and the robot's next gripper command lets it go.
    /// </summary>
    void SetGripperOpenings(List<SystemTarget> systemTargets)
    {
        var initCommands = _program.InitCommands.OfType<Commands.SetGripper>().ToArray();

        if (initCommands.Any(command => command.Part is not null))
            _program.AddError("Set Gripper in the init commands cannot pick up a part.");

        var initial = initCommands.LastOrDefault()?.Opening;
        var openings = Enumerable.Repeat(initial, _robotSystem.RobotCount).ToArray();
        var held = new GrippedPart?[_robotSystem.RobotCount];
        List<GrippedPart> parts = [];
        int events = 0;
        SystemTarget? previous = null;

        foreach (var systemTarget in systemTargets)
        {
            foreach (var target in systemTarget.ProgramTargets)
            {
                // Run-before commands happen at the previous target's pose.
                Apply(target, runBefore: true, previous?.ProgramTargets[target.Group] ?? target);

                target.Gripper = State(target);
                Apply(target, runBefore: false, target);
                target.GripperAtTarget = State(target);
            }

            previous = systemTarget;
        }

        _program.SetGrippedParts(parts);

        GripperState State(ProgramTarget target)
        {
            // An opening from a previous tool is kept within this tool's range.
            var range = target.Target.Tool.Opening;
            double opening = Math.Clamp(openings[target.Group] ?? range.T1, range.T0, range.T1);
            return new(opening, events, held[target.Group]);
        }

        void Apply(ProgramTarget target, bool runBefore, ProgramTarget pose)
        {
            var tool = target.Target.Tool;

            foreach (var command in target.Commands.OfType<Commands.SetGripper>().Where(command => command.RunBefore == runBefore))
            {
                if (tool.Jaws.Count == 0)
                    _program.AddError($"Set Gripper needs a tool with jaws, but tool {tool.Name} has none.", target.Index, target.Group);
                else if (!tool.Opening.IncludesParameter(command.Opening))
                    _program.AddError($"Gripper opening {command.Opening:0.##} mm is outside the {tool.Opening.T0:0.##} to {tool.Opening.T1:0.##} mm range of tool {tool.Name}.", target.Index, target.Group);

                openings[target.Group] = command.Opening;

                if (held[target.Group] is { } release)
                {
                    if (command.Part is not null)
                        _program.AddError("Set Gripper picks up a part while the gripper is already holding one.", target.Index, target.Group);

                    release.Release = pose;
                    release.ReleaseEvent = events;
                    held[target.Group] = null;
                }
                else if (command.Part is { } part)
                {
                    GrippedPart grasp = new(part, target.Group, pose, events);
                    parts.Add(grasp);
                    held[target.Group] = grasp;
                }

                events++;
            }
        }
    }

    void ValidateTargetAxes(IReadOnlyList<ProgramTarget> targets)
    {
        foreach (var target in targets)
        {
            if (_robotSystem.ValidateTargetAxes(target.Group, target.Target) is string error)
            {
                _program.AddError(
                    $"{error}.",
                    target.Index,
                    target.Group);
            }
        }
    }

    void WarnCommandFlybys(IReadOnlyList<ProgramTarget> targets)
    {
        int count = 0;
        ProgramTarget? first = null;

        foreach (var target in targets)
        {
            if (!target.Target.Zone.IsFlyBy
                || !target.Commands.Any(command => command is not Commands.IMotionCommand))
            {
                continue;
            }

            first ??= target;
            count++;
        }

        if (first is null)
            return;

        _program.AddWarning(
            count,
            first.Index,
            first.Group,
            () => "Commands on a fly-by target may run before or after the exact target position.",
            total => $"{total} targets have commands on fly-by targets; commands may run before or after the exact target positions.");
    }

    void WarnDefaults(IReadOnlyList<ProgramTarget> targets)
    {
        WarnMatching(
            targets,
            t => t.Tool.Equals(Tool.Default),
            "Tool is set to default.",
            count => $"{count} targets have their tool set to default.");

        WarnMatching(
            targets,
            t => t.Speed.Equals(Speed.Default),
            "Speed is set to default.",
            count => $"{count} targets have their speed set to default.");

        WarnMatching(
            targets,
            t => t is CartesianTarget { Motion: not Motions.Joint, Configuration: not null },
            "Forced configuration is ignored for non-joint motions.",
            count => $"{count} targets are set to non-joint motion with a forced configuration; configuration is ignored for non-joint motions.");
    }

    void WarnMatching(IReadOnlyList<ProgramTarget> targets, Func<Target, bool> condition, string singular, Func<int, string> plural)
    {
        int count = 0;
        ProgramTarget? first = null;

        foreach (var target in targets)
        {
            if (!condition(target.Target))
                continue;

            first ??= target;
            count++;
        }

        if (first is not null)
        {
            _program.AddWarning(
                count,
                first.Index,
                first.Group,
                () => singular,
                plural);
        }
    }

    void WarnPayloads(IReadOnlyList<ProgramTarget> targets)
    {
        var tools = targets.Select(y => (y.Target.Tool, y.Group)).Distinct();

        foreach (var (tool, group) in tools)
        {
            double payload = _robotSystem.Payload(group);

            if (tool.Weight > payload)
            {
                _program.AddWarning(
                    $"Tool {tool.Name} exceeds rated payload: {payload} kg.",
                    robotGroup: group);
            }
        }
    }

    HashSet<TargetProperty> CollectAttributes(IReadOnlyList<ProgramTarget> targets)
    {
        HashSet<TargetProperty> attributes = [];

        foreach (var programTarget in targets)
        {
            var target = programTarget.Target;
            _ = attributes.Add(target.Tool);
            _ = attributes.Add(target.Frame);
            _ = attributes.Add(target.Speed);
            _ = attributes.Add(target.Zone);
        }

        foreach (var command in _program.InitCommands)
            _ = attributes.Add(command);

        foreach (var programTarget in targets)
        {
            foreach (var command in programTarget.Commands)
                _ = attributes.Add(command);
        }

        return attributes;
    }

    void NameAttributes(HashSet<TargetProperty> attributes, IReadOnlyList<ProgramTarget> targets)
    {
        var comparer = _robotSystem.Manufacturer is Manufacturers.ABB or Manufacturers.KUKA
            ? StringComparer.OrdinalIgnoreCase
            : StringComparer.Ordinal;
        HashSet<string> names = new(attributes.Where(a => a.HasName).Select(a => a.Name), comparer);
        Dictionary<TargetProperty, string> renames = [];

        foreach (var named in attributes.GroupBy(a => a.HasName))
        {
            foreach (var group in named.GroupBy(a => named.Key ? a.Name : a.GetType().Name, comparer))
            {
                bool hasName = named.Key;
                string stem = group.Key;

                if (hasName && !group.Skip(1).Any())
                    continue;

                if (hasName)
                    _program.AddWarning($"Multiple target attributes named \"{stem}\" were found.");

                int i = 0;

                foreach (var attribute in group)
                {
                    string name;

                    do
                    {
                        string suffix = i++.ToString("000", CultureInfo.InvariantCulture);
                        name = stem[..Math.Min(stem.Length, 32 - suffix.Length)] + suffix;
                    }
                    while (!names.Add(name));

                    renames.Add(attribute, name);
                }
            }
        }

        ApplyNames(attributes, targets, renames);
    }

    void ValidateFrame(ProgramTarget programTarget)
    {
        var frame = programTarget.Target.Frame;

        string context = $"Frame {frame.Name}";

        if (!frame.IsCoupled)
        {
            if (frame.CoupledMechanism != -1)
                AddFrameError($"{context} has a coupled mechanism set but no mechanical group.", programTarget);

            return;
        }

        if (_robotSystem is not IndustrialSystem industrialSystem)
        {
            AddFrameError($"{context} is coupled, but this robot system does not support frame coupling.", programTarget);
            return;
        }

        if (frame.CoupledMechanicalGroup < 0 || frame.CoupledMechanicalGroup >= industrialSystem.MechanicalGroups.Count)
        {
            AddFrameError($"{context} is set to couple a nonexistent mechanical group.", programTarget);
            return;
        }

        if (frame.CoupledMechanism == -1)
        {
            if (frame.CoupledMechanicalGroup == programTarget.Group)
            {
                AddFrameError($"{context} is set to couple the robot to itself.", programTarget);
                return;
            }
        }
        else if (frame.CoupledMechanism < 0 || frame.CoupledMechanism >= industrialSystem.MechanicalGroups[frame.CoupledMechanicalGroup].Externals.Length)
        {
            AddFrameError($"{context} is set to couple a nonexistent mechanism.", programTarget);
            return;
        }

        programTarget.CoupledPlaneIndex = industrialSystem.GetPlaneIndex(frame);
    }

    void AddFrameError(string message, ProgramTarget programTarget) =>
        _program.AddError(message, programTarget.Index, programTarget.Group);

    void ApplyNames(HashSet<TargetProperty> attributes, IReadOnlyList<ProgramTarget> targets, Dictionary<TargetProperty, string> renames)
    {
        if (renames.Count == 0)
            return;

        Dictionary<TargetProperty, TargetProperty> replacements = [];

        foreach (var (attribute, name) in renames)
        {
            var namedAttribute = attribute.CloneWithName<TargetProperty>(name);

            _ = attributes.Remove(attribute);
            _ = attributes.Add(namedAttribute);
            replacements.Add(attribute, namedAttribute);
        }

        _program.ReplaceInitCommands(replacements);
        ReplaceProperties(targets, replacements);
    }

    static void ReplaceProperties(IReadOnlyList<ProgramTarget> targets, IReadOnlyDictionary<TargetProperty, TargetProperty> replacements)
    {
        foreach (var programTarget in targets)
        {
            var target = programTarget.Target;

            if (TryReplace(replacements, target.Tool, out Tool tool))
                target = target.WithTool(tool);

            if (TryReplace(replacements, target.Frame, out Frame frame))
                target = target.WithFrame(frame);

            if (TryReplace(replacements, target.Speed, out Speed speed))
                target = target.WithSpeed(speed);

            if (TryReplace(replacements, target.Zone, out Zone zone))
                target = target.WithZone(zone);

            programTarget.Target = target;
            programTarget.ReplaceCommands(replacements);
        }
    }

    static bool TryReplace<T>(IReadOnlyDictionary<TargetProperty, TargetProperty> replacements, T attribute, out T replacement)
        where T : TargetProperty
    {
        if (replacements.TryGetValue(attribute, out var value))
        {
            replacement = (T)value;
            return true;
        }

        replacement = attribute;
        return false;
    }
}
