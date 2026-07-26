using System.Collections.Generic;

namespace DotRecast.Detour;

public class DtBVItemYComparer : IComparer<DtBVItem>
{
    public static readonly DtBVItemYComparer Shared = new();

    private DtBVItemYComparer()
    {
    }

    public int Compare
    (
        DtBVItem a,
        DtBVItem b
    ) =>
        a.bmin.Y.CompareTo(b.bmin.Y);
}
