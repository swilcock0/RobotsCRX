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
