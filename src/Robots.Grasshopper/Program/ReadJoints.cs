namespace Robots.Grasshopper;

public class ReadJoints() : Component(
    "Read Joints",
    "Reads the robot's current joint positions from the controller.",
    "Components",
    "{B14392B6-D1B1-4121-A812-0C5A307392B9}",
    GH_Exposure.senary)
{
    const int RefreshInterval = 200;

    double[]? _joints;

    protected override void RegisterInputParams(GH_InputParamManager pManager)
    {
        _ = pManager.AddParameter(new RobotSystemParameter(), "Robot System", "R", "Robot system connected to the controller.", GH_ParamAccess.item);
        _ = pManager.AddTextParameter("IP", "IP", "IP address of the robot controller.", GH_ParamAccess.item);
        _ = pManager.AddBooleanParameter("Read", "R", "Read the joints. Keep it true to refresh them continuously.", GH_ParamAccess.item, false);
    }

    protected override void RegisterOutputParams(GH_OutputParamManager pManager)
    {
        _ = pManager.AddParameter(new JointsParameter(), "Joints", "J", "Current joint rotations in radians.", GH_ParamAccess.item);
    }

    protected override void SolveComponent(IGH_DataAccess DA)
    {
        var robotSystem = DA.Get<RobotSystem>(0);
        var remote = robotSystem.Remote ?? throw new NotSupportedException("No remote functionality for this robot.");
        var reader = remote as IRemoteJoints ?? throw new NotSupportedException("Reading joints is not supported for this robot.");
        remote.IP = DA.Get<string>(1);

        if (DA.Get(2, false))
        {
            var degrees = reader.ReadJointDegrees();
            _joints = [.. degrees.Select((degree, i) => robotSystem.DegreeToRadian(degree, i))];
            OnPingDocument()?.ScheduleSolution(RefreshInterval, _ => ExpireSolution(false));
        }

        _ = DA.SetData(0, _joints ?? throw new RuntimeWarningException("Set Read to true to read the joints."));
    }
}
