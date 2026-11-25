namespace Robots;

public interface IRemote
{
    List<string> Log { get; }
    string? IP { get; set; }
    void Upload(IProgram program);
    void Pause();
    void Play();
}

public interface IRemoteNotifier
{
    Action? Update { get; set; }
}

public interface IRemoteJoints
{
    /// <summary>
    /// Reads the robot's current joint positions in manufacturer degrees.
    /// </summary>
    double[] ReadJointDegrees();
}

public interface IRemoteFollow
{
    /// <summary>
    /// Starts moving the robot towards the joints, in radians, and keeps following
    /// the latest joints passed to <see cref="Follow"/> until <see cref="IRemote.Pause"/>.
    /// </summary>
    void StartFollowing(RobotSystem system, double[] joints);

    /// <summary>
    /// Sets new joints, in radians, for a robot that is following.
    /// </summary>
    void Follow(double[] joints);
}

public interface IRemoteGripper
{
    /// <summary>
    /// IP address of a Weiss Robotics WSG gripper, such as the Schunk WSG 50, that
    /// the program's Set Gripper commands drive, or null for none.
    /// </summary>
    string? GripperIP { get; set; }

    /// <summary>
    /// Finger speed of the gripper's moves, grasps and releases, in millimeters per second.
    /// </summary>
    double GripperSpeed { get; set; }
}
