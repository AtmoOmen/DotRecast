using System.Collections.Generic;

namespace DotRecast.Detour;

public class DtBVItemZComparer : IComparer<DtBVItem>
{
    public static readonly DtBVItemZComparer Shared = new();

    private DtBVItemZComparer()
    {
    }

    public int Compare
    (
        DtBVItem a,
        DtBVItem b
    ) =>
        a.bmin.Z.CompareTo(b.bmin.Z);
}
