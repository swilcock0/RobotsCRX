using Rhino.Geometry;
using Grasshopper.Kernel.Types;

namespace Robots.Grasshopper;

public class LoadRobotSystem() : Component(
    "Load Robot System",
    "Loads a robot system from the library.",
    "Components",
    "{7722D7E3-98DE-49B5-9B1D-E0D1B938B4A7}"), IDisposable
{
    LibraryForm? _form;

    protected override void RegisterInputParams(GH_InputParamManager pManager)
    {
        _ = pManager.AddTextParameter("Name", "N", "Robot system name in the library.", GH_ParamAccess.item);
        _ = pManager.AddPlaneParameter("Base", "P", "Robot system base plane.", GH_ParamAccess.item, Plane.WorldXY);
        _ = pManager.AddParameter(new PostProcessorParameter(), "Post Processor", "Ps", "Optional alternative post processor.", GH_ParamAccess.item);
        _ = pManager.AddGenericParameter("Joint Limits", "L", "Optional narrower ranges for the robot arm joints in order, as domains in degrees as in the robot library and on the teach pendant, within each joint's own range. A number instead of a domain, such as -1, keeps that joint's full range, and so do joints after the last limit: one domain limits only J1, and {-1, -1, -1, -45 To 45} only J4. Inverse kinematics only picks solutions within them, targets outside them are errors, and Stream Joints refuses joints outside them.", GH_ParamAccess.list);
        pManager[2].Optional = true;
        pManager[3].Optional = true;
    }

    protected override void RegisterOutputParams(GH_OutputParamManager pManager)
    {
        _ = pManager.AddParameter(new RobotSystemParameter(), "Robot System", "R", "Loaded robot system.", GH_ParamAccess.item);
    }

    public override void AddedToDocument(GH_Document document)
    {
        base.AddedToDocument(document);
        _ = LibraryParam.CreateIfEmpty(document, this, ElementType.RobotSystem);
    }

    protected override void SolveComponent(IGH_DataAccess DA)
    {
        var robotSystem = FileIO.LoadRobotSystem(DA.Get<string>(0), DA.Get<Plane>(1), postProcessor: DA.Maybe<IPostProcessor>(2), jointLimits: [.. DA.MaybeList<IGH_Goo>(3).Select(JointLimit)]);
        _ = DA.SetData(0, robotSystem);
    }

    /// <summary>
    /// A domain limits a joint. A number keeps its range, read as a single value
    /// rather than Grasshopper's domain from 0 to the number.
    /// </summary>
    static Interval JointLimit(IGH_Goo goo)
    {
        if (goo is GH_Interval interval)
            return interval.Value;

        if (GH_Convert.ToDouble(goo, out double value, GH_Conversion.Both))
            return new(value, value);

        Interval limit = default;

        return GH_Convert.ToInterval(goo, ref limit, GH_Conversion.Both)
            ? limit
            : throw new ArgumentException($"Joint limit '{goo}' is neither a domain nor a number.");
    }

    public override void CreateAttributes()
    {
        m_attributes = new ComponentButton(this, "Libraries", ToggleForm);
    }

    public override void RemovedFromDocument(GH_Document document)
    {
        base.RemovedFromDocument(document);

        _ = (_form?.Visible = false);

        if (LibraryParam.IsConnected(this, out var libraryParam))
            _ = document.RemoveObject(libraryParam, false);
    }

    void ToggleForm()
    {
        if (_form is null)
        {
            OnlineLibrary library = new();
            library.LibraryChanged += () =>
            {
                if (LibraryParam.IsConnected(this, out var libraryParam))
                    libraryParam.UpdateAndExpire();
            };

            _form = new(library);
        }

        _form.Visible = !_form.Visible;
    }

    public void Dispose()
    {
        _form?.Dispose();
        _form = null;
        GC.SuppressFinalize(this);
    }
}
