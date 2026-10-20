using Rhino.Geometry;

namespace Robots.Grasshopper;

public class PickAndPlace() : Component(
    "Pick and Place",
    "Creates the targets of a pick and place with a gripper tool: joint moves to points above the pick and place, linear moves in and out along the tool's Z axis, and Set Gripper commands that open, grip the part and let it go. With lists of picks and places, outputs one branch of targets per part.",
    "Components",
    "{8E5D2F14-6B3A-4C9D-A7E1-2F0B9C4D6E83}",
    GH_Exposure.secondary)
{
    protected override void RegisterInputParams(GH_InputParamManager pManager)
    {
        _ = pManager.AddPlaneParameter("Pick", "Pk", "TCP plane gripping the part, with Z pointing into it.", GH_ParamAccess.item);
        _ = pManager.AddPlaneParameter("Place", "Pl", "TCP plane releasing the part.", GH_ParamAccess.item);
        _ = pManager.AddParameter(new TargetParameter(), "Target", "T", "Template target for the tool, speed, zone and frame, such as from Create Target. The tool must have jaws. Its plane and commands are not used.", GH_ParamAccess.item);
        _ = pManager.AddNumberParameter("Grip", "G", "Opening that grips the part, in mm.", GH_ParamAccess.item);
        _ = pManager.AddNumberParameter("Approach", "A", "Distance along the tool's Z axis to come in from, in mm.", GH_ParamAccess.item, 100);
        _ = pManager.AddNumberParameter("Retreat", "R", "Distance along the tool's Z axis to leave to, in mm.", GH_ParamAccess.item, 100);
        _ = pManager.AddMeshParameter("Part", "P", "Optional part, where it is before the pick. Program Simulation shows it in Parts.", GH_ParamAccess.item);
        _ = pManager.AddParameter(new SpeedParameter(), "Linear Speed", "S", "Optional speed of the linear moves in and out. Uses the template's speed if omitted.", GH_ParamAccess.item);
        _ = pManager.AddNumberParameter("Wait", "W", "Seconds to wait after the jaws close and after they open.", GH_ParamAccess.item, 0);
        _ = pManager.AddNumberParameter("Open", "Op", "Optional opening in mm the jaws open to before the pick and to let go. Uses the tool's largest opening if omitted; keep it small enough to clear neighbouring parts.", GH_ParamAccess.item);
        pManager[6].Optional = true;
        pManager[7].Optional = true;
        pManager[9].Optional = true;
    }

    protected override void RegisterOutputParams(GH_OutputParamManager pManager)
    {
        _ = pManager.AddParameter(new TargetParameter(), "Targets", "T", "Above the pick, pick, retreat, above the place, place and retreat.", GH_ParamAccess.list);
    }

    protected override void SolveComponent(IGH_DataAccess DA)
    {
        var targets = Robots.PickAndPlace.Create(
            DA.Get<Plane>(0),
            DA.Get<Plane>(1),
            DA.Get<Target>(2),
            DA.Get<double>(3),
            DA.Get<double>(4),
            DA.Get<double>(5),
            DA.Maybe<Mesh>(6),
            DA.Maybe<Speed>(7),
            DA.Get<double>(8),
            DA.MaybeValue<double>(9));

        _ = DA.SetDataList(0, targets);
    }
}
