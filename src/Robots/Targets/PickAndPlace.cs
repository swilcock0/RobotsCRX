using Rhino.Geometry;
using Robots.Commands;

namespace Robots;

/// <summary>
/// The targets of a pick and place with a gripper tool. The robot moves with joint
/// motions to points above the pick and place, and with linear motions in and out
/// along the tool's Z axis. The jaws open above the pick, grip the part at the pick,
/// and let it go at the place. A program that avoids collisions may turn the tool
/// 180 degrees about its Z axis, or move higher between the pick and place.
/// </summary>
public static class PickAndPlace
{
    /// <param name="pick">TCP plane gripping the part, with Z pointing into it.</param>
    /// <param name="place">TCP plane releasing the part.</param>
    /// <param name="template">Tool, speed, zone, frame and joint motion configuration of the targets. The tool must have jaws.</param>
    /// <param name="grip">Opening that grips the part, in millimeters.</param>
    /// <param name="approach">Distance along the tool's Z axis to come in from, in millimeters.</param>
    /// <param name="retreat">Distance along the tool's Z axis to leave to, in millimeters.</param>
    /// <param name="part">Optional part, where it is before the pick.</param>
    /// <param name="linearSpeed">Speed of the linear motions, the template's speed if omitted.</param>
    /// <param name="wait">Seconds to wait after the jaws close and after they open.</param>
    /// <param name="open">Opening the jaws open to, the tool's largest if omitted. Keep it small between closely spaced parts.</param>
    /// <param name="symmetric">The part looks the same turned 180 degrees about the tool's Z axis, so a program avoiding collisions may place it turned from how it was picked.</param>
    public static Target[] Create(Plane pick, Plane place, Target template, double grip, double approach, double retreat, Mesh? part = null, Speed? linearSpeed = null, double wait = 0, double? open = null, bool symmetric = false)
    {
        var tool = template.Tool;

        if (tool.Jaws.Count == 0)
            throw new ArgumentException($"Pick and place needs a tool with jaws, but tool {tool.Name} has none.", nameof(template));

        ArgumentOutOfRangeException.ThrowIfNegative(Util.CheckFinite(approach, nameof(approach)), nameof(approach));
        ArgumentOutOfRangeException.ThrowIfNegative(Util.CheckFinite(retreat, nameof(retreat)), nameof(retreat));
        ArgumentOutOfRangeException.ThrowIfNegative(Util.CheckFinite(wait, nameof(wait)), nameof(wait));

        return new PickAndPlaceJob(pick, place, template, grip, approach, retreat, part, linearSpeed, wait, open ?? tool.Opening.T1, symmetric).Targets(0, 0, 0);
    }
}

/// <summary>
/// A pick and place, kept by its targets so a program can make them again turned
/// or lifted to avoid collisions.
/// </summary>
sealed class PickAndPlaceJob(Plane pick, Plane place, Target template, double grip, double approach, double retreat, Mesh? part, Speed? linearSpeed, double wait, double opening, bool symmetric)
{
    public Plane Pick { get; } = pick;
    public Plane Place { get; } = place;
    public Mesh? Part { get; } = part;
    public bool Symmetric { get; } = symmetric;

    /// <param name="pickRotation">Degrees the tool is turned about its Z axis at the pick.</param>
    /// <param name="placeRotation">Degrees the tool is turned about its Z axis at the place.</param>
    /// <param name="lift">Millimeters along the tool's Z axis above the approach and retreat to move between the pick and place at.</param>
    public Target[] Targets(double pickRotation, double placeRotation, double lift)
    {
        var pick = Turned(Pick, pickRotation);
        var place = Turned(Place, placeRotation);
        var linear = linearSpeed ?? template.Speed;
        var configuration = (template as CartesianTarget)?.Configuration;

        Target[] targets = lift == 0
            ?
            [
                Move(Offset(pick, approach), Motions.Joint, template.Speed, Zone.Default, new SetGripper(opening)),
                Move(pick, Motions.Linear, linear, Zone.Default, Gripper(grip, Part)),
                Move(Offset(pick, retreat), Motions.Linear, linear, template.Zone),
                Move(Offset(place, approach), Motions.Joint, template.Speed, template.Zone),
                Move(place, Motions.Linear, linear, Zone.Default, Gripper(opening, null)),
                Move(Offset(place, retreat), Motions.Linear, linear, template.Zone),
            ]
            :
            [
                // Straight up and down between the lifted points and the approach and retreat.
                Move(Offset(pick, approach + lift), Motions.Joint, template.Speed, template.Zone),
                Move(Offset(pick, approach), Motions.Linear, template.Speed, Zone.Default, new SetGripper(opening)),
                Move(pick, Motions.Linear, linear, Zone.Default, Gripper(grip, Part)),
                Move(Offset(pick, retreat), Motions.Linear, linear, template.Zone),
                Move(Offset(pick, retreat + lift), Motions.Linear, template.Speed, template.Zone),
                Move(Offset(place, approach + lift), Motions.Joint, template.Speed, template.Zone),
                Move(Offset(place, approach), Motions.Linear, template.Speed, template.Zone),
                Move(place, Motions.Linear, linear, Zone.Default, Gripper(opening, null)),
                Move(Offset(place, retreat), Motions.Linear, linear, template.Zone),
                Move(Offset(place, retreat + lift), Motions.Linear, template.Speed, template.Zone),
            ];

        for (int i = 0; i < targets.Length; i++)
            targets[i].Step = new(this, i, targets.Length);

        return targets;

        CartesianTarget Move(Plane plane, Motions motion, Speed speed, Zone zone, Command? command = null)
        {
            // A configuration only applies to joint motions.
            return new(plane, motion == Motions.Joint ? configuration : null, motion, template.Tool, speed, zone, command, template.Frame, template.External, template.ExternalCustom);
        }

        Command Gripper(double opening, Mesh? held)
        {
            return wait > 0
                ? new Group([new SetGripper(opening, held), new Wait(wait)])
                : new SetGripper(opening, held);
        }
    }

    /// <summary>
    /// The part where it is let go, picked and placed with the tool turned by the rotations.
    /// </summary>
    public Mesh? Placed(double pickRotation, double placeRotation)
    {
        if (Part is null)
            return null;

        var placed = Part.DuplicateMesh();
        _ = placed.Transform(Transform.PlaneToPlane(Turned(Pick, pickRotation), Turned(Place, placeRotation)));
        return placed;
    }

    static Plane Turned(Plane plane, double degrees)
    {
        if (degrees != 0)
            _ = plane.Rotate(degrees * Math.PI / 180, plane.ZAxis);

        return plane;
    }

    static Plane Offset(Plane plane, double distance)
    {
        plane.Origin -= plane.ZAxis * distance;
        return plane;
    }
}

/// <summary>
/// A target's place among the targets of a pick and place.
/// </summary>
sealed record PickAndPlaceStep(PickAndPlaceJob Job, int Index, int Count);
