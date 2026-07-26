using System.Collections.Generic;

namespace DotRecast.Detour;

public class DtBVItemXComparer : IComparer<DtBVItem>
{
    public static readonly DtBVItemXComparer Shared = new();

    private DtBVItemXComparer()
    {
    }

    public int Compare
    (
        DtBVItem a,
        DtBVItem b
    ) =>
        a.bmin.X.CompareTo(b.bmin.X);
}
