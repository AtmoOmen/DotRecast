using System;
using DotRecast.Core.Numerics;
using DotRecast.Recast;

namespace DotRecast.Detour.Extras.Jumplink;

public abstract class DtAbstractGroundSampler : IDtGroundSampler
{
    public delegate bool ComputeNavMeshHeight
    (
        RcVec3f   pt,
        float     cellSize,
        out float height
    );

    protected void SampleGround
    (
        DtJumpLinkBuilderConfig acfg,
        DtEdgeSampler           es,
        ComputeNavMeshHeight    heightFunc
    )
    {
        var cs        = acfg.cellSize;
        var dist      = MathF.Sqrt(RcVec.Dist2DSqr(es.start.p, es.start.q));
        var ngsamples = Math.Max(2, (int)MathF.Ceiling(dist / cs));

        SampleGroundSegment(heightFunc, es.start, ngsamples);
        foreach (var end in es.end)
            SampleGroundSegment(heightFunc, end, ngsamples);
    }

    public abstract void Sample
    (
        DtJumpLinkBuilderConfig acfg,
        RcBuilderResult         result,
        DtEdgeSampler           es
    );

    protected void SampleGroundSegment
    (
        ComputeNavMeshHeight heightFunc,
        DtGroundSegment      seg,
        int                  nsamples
    )
    {
        seg.gsamples = new DtGroundSample[nsamples];

        for (var i = 0; i < nsamples; ++i)
        {
            var u = i / (float)(nsamples - 1);

            var s = new DtGroundSample();
            seg.gsamples[i] = s;
            var pt      = RcVec3f.Lerp(seg.p, seg.q, u);
            var success = heightFunc.Invoke(pt, seg.height, out var height);
            s.p.X = pt.X;
            s.p.Y = height;
            s.p.Z = pt.Z;

            if (!success)
                continue;

            s.validHeight = true;
        }
    }
}
