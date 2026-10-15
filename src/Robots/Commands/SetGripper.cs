using Rhino.Geometry;

namespace Robots.Commands;

/// <summary>
/// Sets the opening between the jaws of the target's tool, in millimeters. With a
/// part, the gripper takes the part where it is and carries it with the tool; the
/// next Set Gripper lets it go. It only drives the simulation and collision checks,
/// and adds no code to the program.
/// </summary>
public class SetGripper : Command
{
    public double Opening { get; }

    /// <summary>
    /// Part to pick up, in its position before the gripper takes it.
    /// </summary>
    public Mesh? Part { get; }

    public SetGripper(double opening, Mesh? part = null, bool runBefore = false)
        : base(runBefore: runBefore)
    {
        Opening = CheckNonNegative(opening, nameof(opening));

        if (part is { IsValid: false })
            throw new ArgumentException("Part mesh is invalid.", nameof(part));

        Part = part;
    }

    public override string ToString() => Part is null
        ? $"Command (Gripper opening {Opening:0.##} mm)"
        : $"Command (Gripper opening {Opening:0.##} mm, picking a part)";
}
