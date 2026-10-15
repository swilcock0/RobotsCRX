using Grasshopper;

namespace Robots.Grasshopper;

/// <summary>
/// Warns when another copy of Robots, such as the official Robots package, is
/// installed next to RobotsCRX. Both use the same component IDs and assembly
/// names, so only one of them can load.
/// </summary>
public class RobotsPriority : GH_AssemblyPriority
{
    public override GH_LoadingInstruction PriorityLoad()
    {
        var others = OtherInstallations();

        if (others.Length > 0)
        {
            var message = $"""
                Another copy of Robots is installed next to RobotsCRX, which replaces it:
                {string.Join(Environment.NewLine, others)}

                Uninstall the Robots package in _PackageManager, or remove the other copy, then restart Rhino. Until then, Grasshopper may load the wrong components.
                """;

            Rhino.RhinoApp.WriteLine(message);
            _ = Rhino.UI.Dialogs.ShowMessage(message, "RobotsCRX");
        }

        return GH_LoadingInstruction.Proceed;
    }

    static string[] OtherInstallations()
    {
        var own = typeof(RobotsPriority).Assembly.Location;

        // Empty when Grasshopper loads assemblies from memory.
        if (own.Length == 0)
            return [];

        own = Path.GetFullPath(own);
        var fileName = Path.GetFileName(own);

        // The official package installs into packages/<version>/Robots, which is not always an assembly folder.
        (string Folder, bool Recurse)[] folders =
        [
            (Settings.PluginPath, true),
            .. Folders.AssemblyFolders.Select(folder => (folder.Folder, folder.SearchOption == SearchOption.AllDirectories))
        ];

        return [.. folders
            .Where(folder => Directory.Exists(folder.Folder))
            .SelectMany(folder => Directory.EnumerateFiles(folder.Folder, fileName, new EnumerationOptions { RecurseSubdirectories = folder.Recurse, IgnoreInaccessible = true }))
            .Select(Path.GetFullPath)
            .Where(path => !string.Equals(path, own, StringComparison.OrdinalIgnoreCase))
            .Distinct(StringComparer.OrdinalIgnoreCase)];
    }
}
