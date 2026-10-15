using System.Collections.Concurrent;
using Rhino.Geometry;

namespace Robots;

/// <summary>
/// A part picked up by a <see cref="Commands.SetGripper"/> command. Gripper events
/// are numbered in program order, and each target records how many happened before
/// the robot moves to it, which places the part as waiting, held or placed.
/// </summary>
sealed class GrippedPart(Mesh part, int group, ProgramTarget grasp, int graspEvent)
{
    readonly ConcurrentDictionary<Tool, Tool> _tools = new();
    Mesh? _placed;

    public Mesh Part { get; } = part;
    public int Group { get; } = group;

    // The target whose pose the robot has when the gripper closes.
    public ProgramTarget Grasp { get; } = grasp;
    public int GraspEvent { get; } = graspEvent;
    public ProgramTarget? Release { get; set; }
    public int ReleaseEvent { get; set; } = int.MaxValue;

    /// <summary>
    /// The part relative to the flange, like a tool mesh.
    /// </summary>
    Mesh Local => field ??= Moved(Part, Transform.PlaneToPlane(Flange(Grasp.Kinematics), Plane.WorldXY));

    public bool IsHeld(int events) => events > GraspEvent && events <= ReleaseEvent;

    /// <summary>
    /// The part where it is before the robot moves to a target with the given
    /// number of gripper events, held by a robot at the given pose.
    /// </summary>
    public Mesh Pose(int events, KinematicSolution kinematics)
    {
        if (events <= GraspEvent)
            return Part;

        if (events <= ReleaseEvent)
            return Moved(Local, Transform.PlaneToPlane(Plane.WorldXY, Flange(kinematics)));

        return _placed ??= Moved(Local, Transform.PlaneToPlane(Plane.WorldXY, Flange(Release!.Kinematics)));
    }

    /// <summary>
    /// The tool carrying the part, for collision checks.
    /// </summary>
    public Tool Carried(Tool tool) => _tools.GetOrAdd(tool, value => value.WithAttached(Local));

    static Plane Flange(KinematicSolution kinematics) => kinematics.Planes[^2];

    static Mesh Moved(Mesh mesh, Transform transform)
    {
        var moved = mesh.DuplicateMesh();
        _ = moved.Transform(transform);
        return moved;
    }
}
