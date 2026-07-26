/*
Copyright (c) 2009-2010 Mikko Mononen memon@inside.org
recast4j copyright (c) 2015-2019 Piotr Piastucki piotr@jtilia.org
DotRecast Copyright (c) 2023-2024 Choi Ikpil ikpil@naver.com

This software is provided 'as-is', without any express or implied
warranty.  In no event will the authors be held liable for any damages
arising from the use of this software.
Permission is granted to anyone to use this software for any purpose,
including commercial applications, and to alter it and redistribute it
freely, subject to the following restrictions:
1. The origin of this software must not be misrepresented; you must not
 claim that you wrote the original software. If you use this software
 in a product, an acknowledgment in the product documentation would be
 appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be
 misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

using System;
using System.Collections.Generic;
using System.Linq;
using System.Runtime.CompilerServices;

namespace DotRecast.Detour.Crowd;

public class DtProximityGrid
{
    private readonly float                                _cellSize;
    private readonly float                                _invCellSize;
    private readonly Dictionary<long, List<DtCrowdAgent>> _items;

    public DtProximityGrid
    (
        float cellSize
    )
    {
        _cellSize    = cellSize;
        _invCellSize = 1.0f / cellSize;
        _items       = new Dictionary<long, List<DtCrowdAgent>>();
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static long CombineKey
    (
        int x,
        int y
    )
    {
        var ux = (uint)x;
        var uy = (uint)y;
        return ((long)ux << 32) | uy;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static void DecomposeKey
    (
        long    key,
        out int x,
        out int y
    )
    {
        var ux = (uint)(key >> 32);
        var uy = (uint)key;
        x = (int)ux;
        y = (int)uy;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public void Clear() =>
        _items.Clear();

    public void AddItem
    (
        DtCrowdAgent agent,
        float        minx,
        float        miny,
        float        maxx,
        float        maxy
    )
    {
        var iminx = (int)MathF.Floor(minx * _invCellSize);
        var iminy = (int)MathF.Floor(miny * _invCellSize);
        var imaxx = (int)MathF.Floor(maxx * _invCellSize);
        var imaxy = (int)MathF.Floor(maxy * _invCellSize);

        for (var y = iminy; y <= imaxy; ++y)
        for (var x = iminx; x <= imaxx; ++x)
        {
            var key = CombineKey(x, y);

            if (!_items.TryGetValue(key, out var ids))
            {
                ids = new List<DtCrowdAgent>();
                _items.Add(key, ids);
            }

            ids.Add(agent);
        }
    }

    public int QueryItems
    (
        float     minx,
        float     miny,
        float     maxx,
        float     maxy,
        Span<int> ids,
        int       maxIds
    )
    {
        var iminx = (int)MathF.Floor(minx * _invCellSize);
        var iminy = (int)MathF.Floor(miny * _invCellSize);
        var imaxx = (int)MathF.Floor(maxx * _invCellSize);
        var imaxy = (int)MathF.Floor(maxy * _invCellSize);

        var n = 0;

        for (var y = iminy; y <= imaxy; ++y)
        for (var x = iminx; x <= imaxx; ++x)
        {
            var key     = CombineKey(x, y);
            var hasPool = _items.TryGetValue(key, out var pool);
            if (!hasPool)
                continue;

            for (var idx = 0; idx < pool.Count; ++idx)
            {
                var item = pool[idx];

                // Check if the id exists already.
                var end = n;
                var i   = 0;
                while (i != end && ids[i] != item.idx)
                    ++i;

                // Item not found, add it.
                if (i == n)
                {
                    ids[n++] = item.idx;

                    if (n >= maxIds)
                        return n;
                }
            }
        }

        return n;
    }

    public IEnumerable<(long, int)> GetItemCounts() =>
        _items
            .Where(e => e.Value.Count > 0)
            .Select(e => (e.Key, e.Value.Count));

    public float GetCellSize() =>
        _cellSize;
}
