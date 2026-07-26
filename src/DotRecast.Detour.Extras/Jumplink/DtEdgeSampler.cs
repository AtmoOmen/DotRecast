using System.Collections.Generic;
using DotRecast.Core.Numerics;

namespace DotRecast.Detour.Extras.Jumplink;

public class DtEdgeSampler
{
    public readonly DtGroundSegment       start = new();
    public readonly List<DtGroundSegment> end   = new();
    public readonly IDtTrajectory         trajectory;

    public readonly RcVec3f ax;
    public readonly RcVec3f ay;
    public readonly RcVec3f az;

    public DtEdgeSampler
    (
        DtJumpEdge    edge,
        IDtTrajectory trajectory
    )
    {
        this.trajectory = trajectory;
        ax              = RcVec3f.Subtract(edge.sq, edge.sp);
        ax              = RcVec3f.Normalize(ax);

        az = new RcVec3f(ax.Z, 0, -ax.X);
        az = RcVec3f.Normalize(az);

        ay = new RcVec3f(0, 1, 0);
    }
}
