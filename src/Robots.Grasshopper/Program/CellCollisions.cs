using Rhino.Geometry;

namespace Robots.Grasshopper;

public class CellCollisions() : Component(
    "Cell Collisions",
    "Checks a single-robot cell for collisions: the tool, with any part it holds, against the robot's own arm and base, and the arm, tool and held part against the environment and the other parts, where they wait to be picked and where they were placed. Each separate piece of a mesh counts as its convex hull.",
    "Components",
    "{A6D2F4E8-1C3B-4E57-9F80-2B7D5C9E1A46}",
    GH_Exposure.quarternary)
{
    protected override void RegisterInputParams(GH_InputParamManager pManager)
    {
        _ = pManager.AddParameter(new ProgramParameter(), "Program", "P", "Robot program to check.", GH_ParamAccess.item);
        _ = pManager.AddMeshParameter("Environment", "E", "Optional fixed geometry to avoid, such as the pedestal, table and fixtures. Each separate piece counts as its convex hull, so model hollow or L-shaped fixtures as several pieces. A piece the robot's base touches is its mount, which the first link may touch too.", GH_ParamAccess.list);
        _ = pManager.AddBooleanParameter("Part Obstacles", "Po", "Avoid the parts picked up by Set Gripper while they wait to be picked and after they are placed.", GH_ParamAccess.item, true);
        _ = pManager.AddNumberParameter("Clearance", "Cl", "Millimeters the part obstacles are shrunk by, so parts resting on each other are not reported. Collisions closer than 1 mm count.", GH_ParamAccess.item, 2);
        _ = pManager.AddNumberParameter("Linear Step Size", "Ls", "Linear step size in mm used to check for collisions.", GH_ParamAccess.item, 10);
        _ = pManager.AddNumberParameter("Angular Step Size", "As", "Angular step size in rad used to check for collisions.", GH_ParamAccess.item, Math.PI / 18);
        pManager[1].Optional = true;
    }

    protected override void RegisterOutputParams(GH_OutputParamManager pManager)
    {
        _ = pManager.AddBooleanParameter("Collision Found", "C", "True if a collision was found.", GH_ParamAccess.item);
        _ = pManager.AddIntegerParameter("Target Indices", "I", "Targets the robot was moving to at each collision, the first collision per target.", GH_ParamAccess.list);
        _ = pManager.AddTextParameter("Collisions", "Co", "What collided on the way to each target.", GH_ParamAccess.list);
        _ = pManager.AddMeshParameter("Collided Meshes", "M", "The two meshes of each collision, in pairs.", GH_ParamAccess.list);
    }

    protected override void SolveComponent(IGH_DataAccess DA)
    {
        if (DA.Get<IProgram>(0) is not Program program)
            throw new ArgumentException("Input program cannot have custom code.");

        var contacts = program.CheckCellCollisions(new()
        {
            Environment = DA.MaybeList<Mesh>(1),
            Parts = DA.Get<bool>(2),
            Clearance = DA.Get<double>(3),
            LinearStep = DA.Get<double>(4),
            AngularStep = DA.Get<double>(5),
        });

        _ = DA.SetData(0, contacts.Count > 0);
        _ = DA.SetDataList(1, contacts.Select(contact => contact.TargetIndex));
        _ = DA.SetDataList(2, contacts.Select(contact => $"Target {contact.TargetIndex}: {contact}"));
        _ = DA.SetDataList(3, contacts.SelectMany(contact => contact.Meshes));
    }
}
