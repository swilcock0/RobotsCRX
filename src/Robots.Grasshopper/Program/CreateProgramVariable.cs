using GH_IO.Serialization;
using Grasshopper.Kernel.Parameters;
using Rhino.Geometry;
using static Robots.Grasshopper.ProgramToolpaths;

namespace Robots.Grasshopper;

public class CreateProgramVariable() : Component(
    "Create Program",
    "Creates and checks a robot program.",
    "Components",
    ComponentIds.CreateProgram,
    GH_Exposure.quarternary)
    , IGH_VariableParameterComponent
{
    const string CollisionsDescription = "Avoid collisions in a single-robot cell: the tool and any part it holds against the robot's arm, and the arm, tool and held part against the environment and the parts picked up by Set Gripper. The targets of each Pick and Place are made again turned 180 degrees or moving higher between the pick and place if that clears them, and colliding joint moves, in a Pick and Place or not, are routed around obstacles through joint waypoints or made with the arm in another configuration, with a warning. Other targets are kept, and remaining collisions, such as in linear moves, are errors.";
    const string EnvironmentDescription = "Optional fixed geometry to avoid when checking collisions, such as the pedestal and table. Each separate piece counts as its convex hull, so model hollow or L-shaped fixtures as several pieces. A piece the robot's base touches is its mount, which the first link may touch too.";

    RobotSystem? _robotSystem;

    protected override void RegisterInputParams(GH_InputParamManager pManager)
    {
        _ = pManager.AddTextParameter("Name", "N", "Program name.", GH_ParamAccess.item, "DefaultProgram");
        _ = pManager.AddParameter(new RobotSystemParameter(), "Robot System", "R", "Robot system used by the program.", GH_ParamAccess.item);
        _ = pManager.AddParameter(new ToolpathParameter(), "Targets", "T", "List of targets or toolpaths.", GH_ParamAccess.list);
        _ = pManager.AddParameter(new CommandParameter(), "Init Commands", "C", "Optional list of commands that run at the start of the program.", GH_ParamAccess.list);
        _ = pManager.AddIntegerParameter("Multi-File Indices", "I", "Optional indices used to split the program into multiple files. Each index is the first target of an additional file.", GH_ParamAccess.list);
        _ = pManager.AddNumberParameter("Step Size", "S", "Distance in mm used to step through linear motions for error checking and simulation. Smaller values are more accurate but slower.", GH_ParamAccess.item, 1);
        _ = pManager.AddBooleanParameter("Collisions", "Co", CollisionsDescription, GH_ParamAccess.item, false);
        _ = pManager.AddMeshParameter("Environment", "E", EnvironmentDescription, GH_ParamAccess.list);
        pManager[2].Optional = true;
        pManager[3].Optional = true;
        pManager[4].Optional = true;
        pManager[7].Optional = true;
    }

    /// <summary>
    /// Files saved before the collision inputs were added keep their inputs, so the
    /// collision inputs are added after them.
    /// </summary>
    public override bool Read(GH_IReader reader)
    {
        bool read = base.Read(reader);

        if (InputIndex("Collisions") == -1)
        {
            Param_Boolean collisions = new() { Name = "Collisions", NickName = "Co", Description = CollisionsDescription, Access = GH_ParamAccess.item };
            collisions.SetPersistentData(false);
            _ = Params.RegisterInputParam(collisions);
        }

        if (InputIndex("Environment") == -1)
            _ = Params.RegisterInputParam(new Param_Mesh { Name = "Environment", NickName = "E", Description = EnvironmentDescription, Access = GH_ParamAccess.list, Optional = true });

        return read;
    }

    protected override void RegisterOutputParams(GH_OutputParamManager pManager)
    {
        _ = pManager.AddParameter(new ProgramParameter(), "Program", "P", "Robot program.", GH_ParamAccess.item);
        _ = pManager.AddTextParameter("Code", "C", "Generated robot code.", GH_ParamAccess.tree);
        _ = pManager.AddNumberParameter("Duration", "D", "Program duration in seconds.", GH_ParamAccess.item);
        _ = pManager.AddTextParameter("Warnings", "W", "Program warnings.", GH_ParamAccess.list);
        _ = pManager.AddTextParameter("Errors", "E", "Program errors.", GH_ParamAccess.list);
    }

    void SetTargetInputs(int count)
    {
        var inputs = Params.Input.Where(IsTargetInput).ToList();

        bool paramsChanged = false;

        foreach (var param in inputs.Skip(count))
        {
            _ = Params.UnregisterInputParameter(param, false);
            paramsChanged = true;
        }

        for (int i = 0; i < count; i++)
        {
            IGH_Param param;

            if (i < inputs.Count)
            {
                param = inputs[i];
            }
            else
            {
                param = new ToolpathParameter { Access = GH_ParamAccess.list, Optional = true };
                _ = Params.RegisterInputParam(param, i + 2);
                paramsChanged = true;
            }

            ConfigureTargetInput(param, i, count);
        }

        if (paramsChanged)
            Params.OnParametersChanged();
    }

    protected override void SolveComponent(IGH_DataAccess DA)
    {
        var name = DA.Get<string>(RequiredInputIndex("Name"));
        var robotSystem = DA.Get<RobotSystem>(RequiredInputIndex("Robot System"));

        if (robotSystem != _robotSystem)
        {
            _robotSystem = robotSystem;
            SetTargetInputs(robotSystem.RobotCount);
        }

        var initCommandValues = MaybeList<Command>(DA, "Init Commands");
        var initCommands = initCommandValues.Length > 0 ? new Robots.Commands.Group(initCommandValues) : null;
        var multiFileIndices = MaybeList<int>(DA, "Multi-File Indices");
        var stepSizeIndex = InputIndex("Step Size");
        var stepSize = stepSizeIndex == -1 ? 1.0 : DA.Get<double>(stepSizeIndex);
        var toolpaths = ProgramToolpaths.Read(DA, Params);
        CollisionOptions? collisions = DA.Get("Collisions", false)
            ? new() { Environment = MaybeList<Mesh>(DA, "Environment") }
            : null;

        Program program = new(name, robotSystem, toolpaths, initCommands, multiFileIndices, stepSize, collisions);

        _ = DA.SetData(0, program);

        if (program.Code is not null)
            DA.SetTextTree(1, program.Code, DA.ParameterTargetPath(2));

        _ = DA.SetData(2, program.Duration);

        if (program.Warnings.Count > 0)
        {
            _ = DA.SetDataList(3, program.Warnings);
            AddRuntimeMessage(GH_RuntimeMessageLevel.Warning, "Program has warnings.");
        }

        if (program.Errors.Count > 0)
        {
            _ = DA.SetDataList(4, program.Errors);
            AddRuntimeMessage(GH_RuntimeMessageLevel.Error, "Program has errors.");
        }
    }

    int RequiredInputIndex(string name)
    {
        int index = InputIndex(name);

        return index == -1
            ? throw new InvalidOperationException($"Create Program input '{name}' is missing.")
            : index;
    }

    int InputIndex(string name) => Params.Input.FindIndex(p => p.Name.EqualsIgnoreCase(name));

    T[] MaybeList<T>(IGH_DataAccess DA, string name)
    {
        int index = InputIndex(name);
        return index == -1 ? [] : DA.MaybeList<T>(index);
    }

    bool IGH_VariableParameterComponent.CanInsertParameter(GH_ParameterSide side, int index) => false;
    bool IGH_VariableParameterComponent.CanRemoveParameter(GH_ParameterSide side, int index) => false;
    IGH_Param IGH_VariableParameterComponent.CreateParameter(GH_ParameterSide side, int index) => default!;
    bool IGH_VariableParameterComponent.DestroyParameter(GH_ParameterSide side, int index) => false;
    void IGH_VariableParameterComponent.VariableParameterMaintenance() { }

    internal IGH_Param EnsureTargetInputForUpgrade(int targetIndex)
    {
        int inputIndex = targetIndex + 2;

        while (TargetInputIndices(Params).Length <= targetIndex)
        {
            ToolpathParameter param = new() { Access = GH_ParamAccess.list, Optional = true };
            _ = Params.RegisterInputParam(param, inputIndex);
        }

        var targets = TargetInputIndices(Params);
        int count = targets.Length;

        for (int i = 0; i < targets.Length; i++)
            ConfigureTargetInput(Params.Input[targets[i]], i, count);

        return Params.Input[targets[targetIndex]];
    }

    static void ConfigureTargetInput(IGH_Param param, int index, int count)
    {
        int robot = index + 1;
        bool isOne = count == 1;

        param.Name = isOne ? "Targets" : $"Targets {robot}";
        param.NickName = isOne ? "T" : $"T{robot}";
        param.Description = isOne ? "List of targets or toolpaths." : $"List of targets or toolpaths for robot {robot}.";
    }
}
