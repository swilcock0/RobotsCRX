using System.Drawing;
using System.Windows.Forms;
using GH_IO.Serialization;

namespace Robots.Grasshopper;

sealed class MissingInputException() : Exception("Input data is missing.");

sealed class RuntimeWarningException(string message) : Exception(message);

public abstract class Component(
    string name,
    string nickname,
    string description,
    string subcategory,
    string id,
    GH_Exposure exposure = GH_Exposure.primary)
    : GH_Component(name, nickname, description, RobotsInfo.Category, subcategory)
{
    protected Component(
        string name,
        string description,
        string subcategory,
        string id,
        GH_Exposure exposure = GH_Exposure.primary)
        : this(name, InferNickname(name), description, subcategory, id, exposure) { }

    public override GH_Exposure Exposure => exposure;
    public override Guid ComponentGuid => new(id);
    public override bool IsPreviewCapable => Params.Output.Any(static param =>
        param is IGH_PreviewObject { IsPreviewCapable: true });
    protected override Bitmap Icon => Util.GetIcon(GetType());

    protected override string HtmlHelp_Source() => throw new NotSupportedException("Robots help opens in the system browser.");

    public override void CreateAttributes()
    {
        m_attributes = new ComponentAttributes(this);
    }

    public override bool AppendMenuItems(ToolStripDropDown menu)
    {
        var result = base.AppendMenuItems(menu);
        Util.ReplaceHelpMenuItem(menu, Util.ComponentHelpUrl(this));

        return result;
    }

    /// <summary>
    /// Files saved before an input or output was added have no chunk for it, which
    /// Grasshopper reports as a missing archive chunk. Reads the parameters the file
    /// has and keeps the later ones at their defaults.
    /// </summary>
    public override bool Read(GH_IReader reader)
    {
        if (this is IGH_VariableParameterComponent)
            return base.Read(reader);

        var inputs = Added(Params.Input, reader, "param_input");
        var outputs = Added(Params.Output, reader, "param_output");

        foreach (var param in inputs)
            _ = Params.UnregisterInputParameter(param, false);

        foreach (var param in outputs)
            _ = Params.UnregisterOutputParameter(param, false);

        try
        {
            return base.Read(reader);
        }
        finally
        {
            foreach (var param in inputs)
                _ = Params.RegisterInputParam(param);

            foreach (var param in outputs)
                _ = Params.RegisterOutputParam(param);
        }
    }

    static IGH_Param[] Added(List<IGH_Param> parameters, GH_IReader reader, string chunkName)
    {
        int saved = 0;

        while (reader.ChunkExists(chunkName, saved))
            saved++;

        return [.. parameters.Skip(saved)];
    }

    protected override void RegisterInputParams(GH_InputParamManager pManager) { }
    protected override void RegisterOutputParams(GH_OutputParamManager pManager) { }
    protected abstract void SolveComponent(IGH_DataAccess DA);

    protected sealed override void SolveInstance(IGH_DataAccess DA)
    {
        try
        {
            SolveComponent(DA);
        }
        catch (MissingInputException)
        {
            DA.AbortComponentSolution();
        }
        catch (Exception e)
        {
            var level = e is RuntimeWarningException
                ? GH_RuntimeMessageLevel.Warning
                : GH_RuntimeMessageLevel.Error;

            AddRuntimeMessage(level, RuntimeMessage(e));
            DA.AbortComponentSolution();
        }
    }

    static string InferNickname(string name) => string.Concat(name.Split(' ', StringSplitOptions.RemoveEmptyEntries));

    static string RuntimeMessage(Exception exception)
    {
        var message = exception is System.Xml.XmlException xml
            ? $"Invalid XML syntax in \"{Path.GetFileName(xml.SourceUri)}\"."
            : exception.Message;

        return CleanArgumentMessage(message);
    }

    static string CleanArgumentMessage(string message)
    {
        var lines = message.UseLF().Split('\n');
        var filtered = lines
            .Where(line =>
                !line.StartsWith("Parameter ", StringComparison.Ordinal)
                && !line.StartsWith("Actual value was ", StringComparison.Ordinal))
            .ToArray();

        return string.Join(Environment.NewLine, filtered);
    }
}
