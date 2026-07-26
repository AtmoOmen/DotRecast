using System;
using DotRecast.Core.Numerics;
using DotRecast.Recast;

namespace DotRecast.Detour.Extras.Jumplink;

public class DtTrajectorySampler
{
    public void Sample
    (
        DtJumpLinkBuilderConfig acfg,
        RcHeightfield           heightfield,
        DtEdgeSampler           es
    )
    {
        var nsamples = es.start.gsamples.Length;

        for (var i = 0; i < nsamples; ++i)
        {
            var ssmp = es.start.gsamples[i];

            foreach (var end in es.end)
            {
                var esmp = end.gsamples[i];
                if (!ssmp.validHeight || !esmp.validHeight)
                    continue;

                if (!SampleTrajectory(acfg, heightfield, ssmp.p, esmp.p, es.trajectory))
                    continue;

                ssmp.validTrajectory = true;
                esmp.validTrajectory = true;
            }
        }
    }

    private bool SampleTrajectory
    (
        DtJumpLinkBuilderConfig acfg,
        RcHeightfield           solid,
        RcVec3f                 pa,
        RcVec3f                 pb,
        IDtTrajectory           tra
    )
    {
        var cs       = Math.Min(acfg.cellSize, acfg.cellHeight);
        var d        = RcVec.Dist2D(pa, pb) + MathF.Abs(pa.Y - pb.Y);
        var nsamples = Math.Max(2, (int)MathF.Ceiling(d / cs));

        for (var i = 0; i < nsamples; ++i)
        {
            var u = i / (float)(nsamples - 1);
            var p = tra.Apply(pa, pb, u);
            if (CheckHeightfieldCollision(solid, p.X, p.Y + acfg.groundTolerance, p.Y + acfg.agentHeight, p.Z))
                return false;
        }

        return true;
    }

    private bool CheckHeightfieldCollision
    (
        RcHeightfield solid,
        float         x,
        float         ymin,
        float         ymax,
        float         z
    )
    {
        var w    = solid.width;
        var h    = solid.height;
        var cs   = solid.cs;
        var ch   = solid.ch;
        var orig = solid.bmin;
        var ix   = (int)MathF.Floor((x - orig.X) / cs);
        var iz   = (int)MathF.Floor((z - orig.Z) / cs);

        if (ix < 0 || iz < 0 || ix > w || iz > h)
            return false;

        var spanIndex = solid.spans[ix + (iz * w)];
        if (spanIndex == 0)
            return false;

        while (spanIndex != 0)
        {
            ref var s     = ref solid.Span(spanIndex);
            var     symin = orig.Y + (s.smin * ch);
            var     symax = orig.Y + (s.smax * ch);
            if (OverlapRange(ymin, ymax, symin, symax))
                return true;

            spanIndex = s.next;
        }

        return false;
    }

    private bool OverlapRange
    (
        float amin,
        float amax,
        float bmin,
        float bmax
    ) =>
        amin > bmax || amax < bmin ?
            false :
            true;
}
