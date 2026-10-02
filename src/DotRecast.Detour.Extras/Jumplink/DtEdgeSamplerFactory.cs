using System;
using DotRecast.Core.Numerics;

namespace DotRecast.Detour.Extras.Jumplink;

public class DtEdgeSamplerFactory
{
    public DtEdgeSampler Get
    (
        DtJumpLinkBuilderConfig acfg,
        DtJumpLinkType          type,
        DtJumpEdge              edge
    )
    {
        IDtTrajectory trajectory;

        switch (type.Bit)
        {
            case DtJumpLinkType.EDGE_JUMP_BIT:
                trajectory = new DtBallisticTrajectory(acfg.verticalSpeed, acfg.gravity);
                break;
            case DtJumpLinkType.EDGE_CLIMB_DOWN_BIT:
                trajectory = new DtBallisticTrajectory(0, acfg.gravity);
                break;
            case DtJumpLinkType.EDGE_JUMP_OVER_BIT:
            default:
                throw new ArgumentException("Unsupported jump type " + type);
        }

        var es = new DtEdgeSampler(edge, trajectory);

        es.start.height = acfg.agentClimb * 2;
        var offset = new RcVec3f();
        Trans2d(ref offset, es.az, es.ay, new RcVec2f(acfg.startDistance, -acfg.agentClimb));
        Vadd(ref es.start.p, edge.sp, offset);
        Vadd(ref es.start.q, edge.sq, offset);

        var dx       = acfg.endDistance - (2 * acfg.agentRadius);
        var cs       = acfg.cellSize;
        var nsamples = Math.Max(2, (int)MathF.Ceiling(dx / cs));

        for (var j = 0; j < nsamples; ++j)
        {
            var v  = j / (float)(nsamples - 1);
            var ox = (2 * acfg.agentRadius) + (dx * v);
            Trans2d(ref offset, es.az, es.ay, new RcVec2f(ox, acfg.minHeight));
            var end = new DtGroundSegment();
            end.height = acfg.heightRange;
            Vadd(ref end.p, edge.sp, offset);
            Vadd(ref end.q, edge.sq, offset);
            es.end.Add(end);
        }

        return es;
    }

    private void Vadd
    (
        ref RcVec3f dest,
        RcVec3f     v1,
        RcVec3f     v2
    )
    {
        dest.X = v1.X + v2.X;
        dest.Y = v1.Y + v2.Y;
        dest.Z = v1.Z + v2.Z;
    }

    private void Trans2d
    (
        ref RcVec3f dst,
        RcVec3f     ax,
        RcVec3f     ay,
        RcVec2f     pt
    )
    {
        dst.X = (ax.X * pt.X) + (ay.X * pt.Y);
        dst.Y = (ax.Y * pt.X) + (ay.Y * pt.Y);
        dst.Z = (ax.Z * pt.X) + (ay.Z * pt.Y);
    }
}
