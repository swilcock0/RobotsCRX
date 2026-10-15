namespace Robots.Grasshopper;

public class StreamJoints() : Component(
    "Stream Joints",
    "Moves the robot to joints that can change while it moves, such as from sliders or a solver.",
    "Components",
    "{6F0C2D47-8E0B-4E83-9C0A-3B5D1A7E42F9}",
    GH_Exposure.senary)
{
    const int RefreshInterval = 200;

    // Unknown until the first solution, so a file saved while streaming does not start the robot when opened.
    bool? _stream;

    protected override void RegisterInputParams(GH_InputParamManager pManager)
    {
        var streamDescription = """
            Set to true to start following the joints, and to false to stop smoothly.
            - Fanuc: Streams with Stream Motion (J519) at a quarter of the controller's speed limits. The joints must stay within 20 deg of the robot, so move them in small steps. After an error, set Stream to false and true again to restart.
            """.UseCRLF();

        _ = pManager.AddParameter(new RobotSystemParameter(), "Robot System", "R", "Robot system connected to the controller.", GH_ParamAccess.item);
        _ = pManager.AddTextParameter("IP", "IP", "IP address of the robot controller.", GH_ParamAccess.item);
        _ = pManager.AddParameter(new JointsParameter(), "Joints", "J", "Joint rotations in radians for the robot to follow.", GH_ParamAccess.item);
        _ = pManager.AddBooleanParameter("Stream", "S", streamDescription, GH_ParamAccess.item, false);
    }

    protected override void RegisterOutputParams(GH_OutputParamManager pManager)
    {
        _ = pManager.AddTextParameter("Log", "L", "Remote connection log.", GH_ParamAccess.list);
    }

    protected override void SolveComponent(IGH_DataAccess DA)
    {
        var robotSystem = DA.Get<RobotSystem>(0);
        var remote = robotSystem.Remote ?? throw new NotSupportedException("No remote functionality for this robot.");
        var follower = remote as IRemoteFollow ?? throw new NotSupportedException("Streaming joints is not supported for this robot.");
        remote.IP = DA.Get<string>(1);
        var joints = DA.Get<double[]>(2);
        bool stream = DA.Get(3, false);
        var previous = _stream;

        lock (remote.Log)
            _ = DA.SetDataList(0, remote.Log);

        if (stream && previous is null)
            throw new RuntimeWarningException("Set Stream to false and true again to start.");

        _stream = stream;

        // Refreshes the log while streaming.
        if (stream)
            OnPingDocument()?.ScheduleSolution(RefreshInterval, _ => ExpireSolution(false));

        if (stream && previous == false)
            follower.StartFollowing(robotSystem, joints);
        else if (stream)
            follower.Follow(joints);
        else if (previous == true)
            remote.Pause();
    }
}
