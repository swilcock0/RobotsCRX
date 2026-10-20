using Rhino.Geometry;
using Robots.Commands;

namespace Robots;

/// <summary>
/// The targets of a pick and place with a gripper tool. The robot moves with joint
/// motions to points above the pick and place, and with linear motions in and out
/// along the tool's Z axis. The jaws open above the pick, grip the part at the pick,
/// and let it go at the place.
/// </summary>
public static class PickAndPlace
{
    /// <param name="pick">TCP plane gripping the part, with Z pointing into it.</param>
    /// <param name="place">TCP plane releasing the part.</param>
    /// <param name="template">Tool, speed, zone and frame of the targets. The tool must have jaws.</param>
    /// <param name="grip">Opening that grips the part, in millimeters.</param>
    /// <param name="approach">Distance along the tool's Z axis to come in from, in millimeters.</param>
    /// <param name="retreat">Distance along the tool's Z axis to leave to, in millimeters.</param>
    /// <param name="part">Optional part, where it is before the pick.</param>
    /// <param name="linearSpeed">Speed of the linear motions, the template's speed if omitted.</param>
    /// <param name="wait">Seconds to wait after the jaws close and after they open.</param>
    /// <param name="open">Opening the jaws open to, the tool's largest if omitted. Keep it small between closely spaced parts.</param>
    public static Target[] Create(Plane pick, Plane place, Target template, double grip, double approach, double retreat, Mesh? part = null, Speed? linearSpeed = null, double wait = 0, double? open = null)
    {
        var tool = template.Tool;

        if (tool.Jaws.Count == 0)
            throw new ArgumentException($"Pick and place needs a tool with jaws, but tool {tool.Name} has none.", nameof(template));

        ArgumentOutOfRangeException.ThrowIfNegative(Util.CheckFinite(approach, nameof(approach)), nameof(approach));
        ArgumentOutOfRangeException.ThrowIfNegative(Util.CheckFinite(retreat, nameof(retreat)), nameof(retreat));
        ArgumentOutOfRangeException.ThrowIfNegative(Util.CheckFinite(wait, nameof(wait)), nameof(wait));

        double opening = open ?? tool.Opening.T1;
        var linear = linearSpeed ?? template.Speed;
        var configuration = (template as CartesianTarget)?.Configuration;

        return
        [
            Move(Offset(pick, approach), Motions.Joint, template.Speed, Zone.Default, new SetGripper(opening)),
            Move(pick, Motions.Linear, linear, Zone.Default, Gripper(grip, part)),
            Move(Offset(pick, retreat), Motions.Linear, linear, template.Zone),
            Move(Offset(place, approach), Motions.Joint, template.Speed, template.Zone),
            Move(place, Motions.Linear, linear, Zone.Default, Gripper(opening, null)),
            Move(Offset(place, retreat), Motions.Linear, linear, template.Zone),
        ];

        CartesianTarget Move(Plane plane, Motions motion, Speed speed, Zone zone, Command? command = null)
        {
            // A configuration only applies to joint motions.
            return new(plane, motion == Motions.Joint ? configuration : null, motion, tool, speed, zone, command, template.Frame, template.External, template.ExternalCustom);
        }

        Command Gripper(double opening, Mesh? held)
        {
            return wait > 0
                ? new Group([new SetGripper(opening, held), new Wait(wait)])
                : new SetGripper(opening, held);
        }
    }

    static Plane Offset(Plane plane, double distance)
    {
        plane.Origin -= plane.ZAxis * distance;
        return plane;
    }
}
