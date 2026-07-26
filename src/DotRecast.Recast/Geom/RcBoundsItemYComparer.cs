using System.Collections.Generic;

namespace DotRecast.Recast.Geom;

public class RcBoundsItemYComparer : IComparer<RcBoundsItem>
{
    public static readonly RcBoundsItemYComparer Shared = new();

    private RcBoundsItemYComparer()
    {
    }

    public int Compare
    (
        RcBoundsItem a,
        RcBoundsItem b
    ) =>
        a.bmin.Y.CompareTo(b.bmin.Y);
}
