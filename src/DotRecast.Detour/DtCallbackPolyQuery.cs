using System;

namespace DotRecast.Detour;

public class DtCallbackPolyQuery : IDtPolyQuery
{
    private readonly Action<DtMeshTile, DtPoly, long> _callback;

    public DtCallbackPolyQuery
    (
        Action<DtMeshTile, DtPoly, long> callback
    ) =>
        _callback = callback;

    public void Process
    (
        DtMeshTile         tile,
        ReadOnlySpan<int>  polys,
        ReadOnlySpan<long> polyRefs,
        int                count
    )
    {
        for (var i = 0; i < count; ++i)
        {
            var polyIdx = polys[i];
            var poly    = tile.data.polys[polyIdx];
            _callback?.Invoke(tile, poly, polyRefs[i]);
        }
    }
}
