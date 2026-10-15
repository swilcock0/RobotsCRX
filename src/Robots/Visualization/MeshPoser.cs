namespace Robots;

public interface IMeshPoser
{
    void Pose(IReadOnlyList<KinematicSolution> solutions, Tool[] tools);
}

public static class IMeshPoserExtensions
{
    extension(IMeshPoser poser)
    {
        /// <param name="atTarget">The robot has reached the target and run its commands, such as gripper changes.</param>
        public void Pose(IReadOnlyList<KinematicSolution> solutions, SystemTarget systemTarget, bool atTarget = false)
        {
            var tools = systemTarget.ProgramTargets.Map(t =>
            {
                var gripper = atTarget ? t.GripperAtTarget : t.Gripper;
                var tool = t.Target.Tool.WithOpening(gripper.Opening);
                return gripper.Held?.Carried(tool) ?? tool;
            });
            poser.Pose(solutions, tools);
        }
    }
}
