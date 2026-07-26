using System.Collections.Generic;

namespace DotRecast.Recast;

public class RcPotentialDiagonalComparer : IComparer<RcPotentialDiagonal>
{
    public static readonly RcPotentialDiagonalComparer Shared = new();

    private RcPotentialDiagonalComparer()
    {
    }

    public int Compare
    (
        RcPotentialDiagonal va,
        RcPotentialDiagonal vb
    )
    {
        var a = va;
        var b = vb;
        return a.dist.CompareTo(b.dist);
    }
}
