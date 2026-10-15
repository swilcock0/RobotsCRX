using Rhino.Geometry;

namespace Robots.Grasshopper.Commands;

public class SetGripper() : CommandComponent(
    "Set Gripper",
    "Opens or closes the jaws of the target's tool in the simulation, optionally picking up a part that the tool then carries until the next Set Gripper. Adds no code to the program, so group it with the command that moves the real gripper.",
    "{3D8B6A21-5C4F-4F7E-9B1A-7E2C0D94A6B3}",
    GH_Exposure.secondary)
{
    protected override void RegisterInputParams(GH_InputParamManager pManager)
    {
        _ = pManager.AddNumberParameter("Opening", "O", "Opening between the jaws in mm, within the tool's range.", GH_ParamAccess.item);
        _ = pManager.AddMeshParameter("Part", "P", "Optional part to pick up, where it is before the gripper takes it. The next Set Gripper lets it go, and Program Simulation shows it in Parts. A held part counts as part of the tool in collision checks.", GH_ParamAccess.item);
        pManager[1].Optional = true;
    }

    protected override Command SolveCommand(IGH_DataAccess DA) => new Robots.Commands.SetGripper(DA.Get<double>(0), DA.Maybe<Mesh>(1));
}
