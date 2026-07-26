using System.Collections.Generic;

namespace DotRecast.Recast.Geom;

public class RcBoundsItemXComparer : IComparer<RcBoundsItem>
{
    public static readonly RcBoundsItemXComparer Shared = new();

    private RcBoundsItemXComparer()
    {
    }

    public int Compare
    (
        RcBoundsItem a,
        RcBoundsItem b
    ) =>
        a.bmin.X.CompareTo(b.bmin.X);
}
