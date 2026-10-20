using System.Drawing;
using Rhino.Geometry;

namespace Robots.Grasshopper;

public class Reachability() : Component(
    "Reachability",
    "Scores how well a 6-axis robot reaches points, with the tool turned through several rotations about its axis. A rotation scores 0 out of reach or near a singularity, and up to 1 as the wrist moves away from the J5 = 0 flip boundary, where linear moves can flip the wrist. Red is 0, amber 0.5, green 1 and grey out of reach.",
    "Components",
    "{5C1E7A92-3B48-4D6F-8E25-A9D04B7F61C3}",
    GH_Exposure.quinary)
{
    static readonly string[] LegendTags = ["0: out of reach or at a flip", "0.5", "1: clear for every rotation", "No rotation reaches"];

    protected override void RegisterInputParams(GH_InputParamManager pManager)
    {
        _ = pManager.AddParameter(new RobotSystemParameter(), "Robot System", "R", "Robot system to check.", GH_ParamAccess.item);
        _ = pManager.AddPointParameter("Points", "Pt", "Points to check, such as a grid over the table. The TCP goes to each.", GH_ParamAccess.list);
        _ = pManager.AddPlaneParameter("Orientation", "O", "Tool orientation; only its axes are used, and it is turned about its Z axis. Defaults to the tool pointing straight down.", GH_ParamAccess.item, new Plane(Point3d.Origin, Vector3d.XAxis, -Vector3d.YAxis));
        _ = pManager.AddIntegerParameter("Rotations", "Rt", "Number of tool rotations, evenly spaced about its Z axis, to score each point by.", GH_ParamAccess.item, 8);
        _ = pManager.AddParameter(new ToolParameter(), "Tool", "T", "Optional tool.", GH_ParamAccess.item);
        _ = pManager.AddParameter(new JointsParameter(), "Reference", "J", "Optional joints in radians the robot comes from, such as its start pose. The robot reaches each point with the solution closest to them, as a program would.", GH_ParamAccess.item);
        _ = pManager.AddNumberParameter("Cell Size", "S", "Size in mm of each point's square on the map, such as the grid spacing.", GH_ParamAccess.item, 40);
        pManager[4].Optional = true;
        pManager[5].Optional = true;
    }

    protected override void RegisterOutputParams(GH_OutputParamManager pManager)
    {
        _ = pManager.AddMeshParameter("Map", "M", "A square per point colored by its score: red 0, amber 0.5, green 1, grey out of reach.", GH_ParamAccess.item);
        _ = pManager.AddNumberParameter("Score", "Sc", "Score per point from 0 to 1: the mean over the rotations.", GH_ParamAccess.list);
        _ = pManager.AddNumberParameter("Best Rotation", "B", "Rotation in degrees about the tool's Z axis with the most wrist margin, per point.", GH_ParamAccess.list);
        _ = pManager.AddColourParameter("Legend Colours", "LC", "Colors of the score scale, for the Legend component.", GH_ParamAccess.list);
        _ = pManager.AddTextParameter("Legend Tags", "LT", "Tags of the score scale, for the Legend component.", GH_ParamAccess.list);
    }

    protected override void SolveComponent(IGH_DataAccess DA)
    {
        var system = DA.Get<RobotSystem>(0);
        var points = DA.MaybeList<Point3d>(1);
        var orientation = DA.Get<Plane>(2);
        var scores = Robots.Reachability.Score(system, points, orientation, DA.Get<int>(3), DA.Maybe<Tool>(4), DA.Maybe<double[]>(5));
        Plane[] planes = [.. points.Select(point => new Plane(point, orientation.XAxis, orientation.YAxis))];

        _ = DA.SetData(0, Robots.Reachability.Map(planes, scores, DA.Get<double>(6)));
        _ = DA.SetDataList(1, scores.Select(score => score.Score));
        _ = DA.SetDataList(2, scores.Select(score => score.BestRotation));
        _ = DA.SetDataList(3, new[] { Robots.Reachability.ScoreColor(0), Robots.Reachability.ScoreColor(0.5), Robots.Reachability.ScoreColor(1), Color.FromArgb(150, 150, 150) });
        _ = DA.SetDataList(4, LegendTags);
    }
}
