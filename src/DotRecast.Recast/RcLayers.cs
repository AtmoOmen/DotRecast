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
using DotRecast.Core;

namespace DotRecast.Recast;

using static RcRecast;

public static class RcLayers
{
    private static void AddUnique
    (
        List<int> a,
        int       v
    )
    {
        if (!a.Contains(v))
            a.Add(v);
    }

    private static bool Contains
    (
        List<int> a,
        int       v
    ) =>
        a.Contains(v);

    private static bool OverlapRange
    (
        int amin,
        int amax,
        int bmin,
        int bmax
    ) =>
        amin > bmax || amax < bmin ?
            false :
            true;

    /// @par
    /// 
    /// See the #rcConfig documentation for more information on the configuration parameters.
    /// 
    /// @see rcAllocHeightfieldLayerSet, rcCompactHeightfield, rcHeightfieldLayerSet, rcConfig
    /// @}
    /// @name Layer, Contour, Polymesh, and Detail Mesh Functions
    /// @see rcHeightfieldLayer, rcContourSet, rcPolyMesh, rcPolyMeshDetail
    /// @{
    /// Builds a layer set from the specified compact heightfield.
    /// @ingroup recast
    /// @param[in,out]	ctx				The build context to use during the operation.
    /// @param[in]		chf				A fully built compact heightfield.
    /// @param[in]		borderSize		The size of the non-navigable border around the heightfield. [Limit: >=0] 
    /// [Units: vx]
    /// @param[in]		walkableHeight	Minimum floor to 'ceiling' height that will still allow the floor area 
    /// to be considered walkable. [Limit: >= 3] [Units: vx]
    /// @param[out]		lset			The resulting layer set. (Must be pre-allocated.)
    /// @returns True if the operation completed successfully.
    public static bool BuildHeightfieldLayers
    (
        RcContext                 ctx,
        RcCompactHeightfield      chf,
        int                       borderSize,
        int                       walkableHeight,
        out RcHeightfieldLayerSet lset
    )
    {
        lset = null;
        using var timer = ctx.ScopedTimer(RcTimerLabel.RC_TIMER_BUILD_LAYERS);

        var w = chf.width;
        var h = chf.height;

        Span<byte> srcReg = stackalloc byte[chf.spanCount];
        srcReg.Fill(0xFF);

        var nsweeps = chf.width;
        var sweeps  = new RcLayerSweepSpan[nsweeps];
        for (var i = 0; i < sweeps.Length; i++)
            sweeps[i] = new RcLayerSweepSpan();

        // Partition walkable area into monotone regions.
        Span<int> prevCount = stackalloc int[256];
        byte      regId     = 0;

        // Sweep one line at a time.
        for (var y = borderSize; y < h - borderSize; ++y)
        {
            // Collect spans from this row.
            prevCount.Fill(0);
            byte sweepId = 0;

            for (var x = borderSize; x < w - borderSize; ++x)
            {
                ref var c = ref chf.cells[x + (y * w)];

                for (int i = c.index, ni = c.index + c.count; i < ni; ++i)
                {
                    ref var s = ref chf.spans[i];
                    if (chf.areas[i] == RC_NULL_AREA)
                        continue;

                    byte sid = 0xFF;

                    // -x
                    if (GetCon(s, 0) != RC_NOT_CONNECTED)
                    {
                        var ax = x                              + GetDirOffsetX(0);
                        var ay = y                              + GetDirOffsetY(0);
                        var ai = chf.cells[ax + (ay * w)].index + GetCon(s, 0);
                        if (chf.areas[ai] != RC_NULL_AREA && srcReg[ai] != 0xff)
                            sid = srcReg[ai];
                    }

                    if (sid == 0xff)
                    {
                        sid             = sweepId++;
                        sweeps[sid].nei = 0xff;
                        sweeps[sid].ns  = 0;
                    }

                    // -y
                    if (GetCon(s, 3) != RC_NOT_CONNECTED)
                    {
                        var ax = x                              + GetDirOffsetX(3);
                        var ay = y                              + GetDirOffsetY(3);
                        var ai = chf.cells[ax + (ay * w)].index + GetCon(s, 3);
                        var nr = srcReg[ai];

                        if (nr != 0xff)
                        {
                            // Set neighbour when first valid neighbour is encoutered.
                            if (sweeps[sid].ns == 0)
                                sweeps[sid].nei = nr;

                            if (sweeps[sid].nei == nr)
                            {
                                // Update existing neighbour
                                sweeps[sid].ns++;
                                prevCount[nr]++;
                            }
                            else
                            {
                                // This is hit if there is nore than one neighbour.
                                // Invalidate the neighbour.
                                sweeps[sid].nei = 0xff;
                            }
                        }
                    }

                    srcReg[i] = sid;
                }
            }

            // Create unique ID.
            for (var i = 0; i < sweepId; ++i)
                // If the neighbour is set and there is only one continuous connection to it,
                // the sweep will be merged with the previous one, else new region is created.
                if (sweeps[i].nei != 0xff && prevCount[sweeps[i].nei] == sweeps[i].ns)
                    sweeps[i].id = sweeps[i].nei;
                else
                {
                    if (regId == 255)
                    {
                        RcThrowHelper.ThrowException("rcBuildHeightfieldLayers: Region ID overflow.");
                        return false;
                    }

                    sweeps[i].id = regId++;
                }

            // Remap local sweep ids to region ids.
            for (var x = borderSize; x < w - borderSize; ++x)
            {
                ref var c = ref chf.cells[x + (y * w)];
                for (int i = c.index, ni = c.index + c.count; i < ni; ++i)
                    if (srcReg[i] != 0xff)
                        srcReg[i] = sweeps[srcReg[i]].id;
            }
        }

        // Allocate and init layer regions.
        int nregs = regId;
        var regs  = new RcLayerRegion[nregs];

        // Construct regions
        for (var i = 0; i < nregs; ++i)
            regs[i] = new RcLayerRegion(i);

        // Find region neighbours and overlapping regions.
        var lregs = new List<int>();

        for (var y = 0; y < h; ++y)
        for (var x = 0; x < w; ++x)
        {
            ref var c = ref chf.cells[x + (y * w)];

            lregs.Clear();

            for (int i = c.index, ni = c.index + c.count; i < ni; ++i)
            {
                ref var s  = ref chf.spans[i];
                int     ri = srcReg[i];
                if (ri == 0xff)
                    continue;

                regs[ri].ymin = Math.Min(regs[ri].ymin, s.y);
                regs[ri].ymax = Math.Max(regs[ri].ymax, s.y);

                // Collect all region layers.
                lregs.Add(ri);

                // Update neighbours
                for (var dir = 0; dir < 4; ++dir)
                    if (GetCon(s, dir) != RC_NOT_CONNECTED)
                    {
                        var ax  = x                              + GetDirOffsetX(dir);
                        var ay  = y                              + GetDirOffsetY(dir);
                        var ai  = chf.cells[ax + (ay * w)].index + GetCon(s, dir);
                        int rai = srcReg[ai];

                        if (rai != 0xff && rai != ri)
                        {
                            // Don't check return value -- if we cannot add the neighbor
                            // it will just cause a few more regions to be created, which
                            // is fine.
                            AddUnique(regs[ri].neis, rai);
                        }
                    }
            }

            // Update overlapping regions.
            for (var i = 0; i < lregs.Count - 1; ++i)
            for (var j = i + 1; j < lregs.Count; ++j)
                if (lregs[i] != lregs[j])
                {
                    var ri = regs[lregs[i]];
                    var rj = regs[lregs[j]];
                    AddUnique(ri.layers, lregs[j]);
                    AddUnique(rj.layers, lregs[i]);
                }
        }

        // Create 2D layers from regions.
        byte layerId = 0;

        const int  MAX_STACK = 64;
        Span<byte> stack     = stackalloc byte[MAX_STACK];
        var        nstack    = 0;

        for (var i = 0; i < nregs; ++i)
        {
            var root = regs[i];
            // Skip already visited.
            if (root.layerId != 0xff)
                continue;

            // Start search.
            root.layerId = layerId;
            root.@base   = true;

            nstack          = 0;
            stack[nstack++] = (byte)i;

            while (0 != nstack)
            {
                // Pop front
                var reg = regs[stack[0]];
                nstack--;
                for (var j = 0; j < nstack; ++j)
                    stack[j] = stack[j + 1];

                foreach (var nei in reg.neis)
                {
                    var regn = regs[nei];
                    // Skip already visited.
                    if (regn.layerId != 0xff)
                        continue;

                    // Skip if the neighbour is overlapping root region.
                    if (Contains(root.layers, nei))
                        continue;

                    // Skip if the height range would become too large.
                    var ymin = Math.Min(root.ymin, regn.ymin);
                    var ymax = Math.Max(root.ymax, regn.ymax);
                    if (ymax - ymin >= 255)
                        continue;

                    if (nstack < MAX_STACK)
                    {
                        // Deepen
                        stack[nstack++] = (byte)nei;

                        // Mark layer id
                        regn.layerId = layerId;
                        // Merge current layers to root.
                        foreach (var layer in regn.layers)
                            AddUnique(root.layers, layer);

                        root.ymin = Math.Min(root.ymin, regn.ymin);
                        root.ymax = Math.Max(root.ymax, regn.ymax);
                    }
                }
            }

            layerId++;
        }

        // Merge non-overlapping regions that are close in height.
        var mergeHeight = walkableHeight * 4;

        for (var i = 0; i < nregs; ++i)
        {
            var ri = regs[i];
            if (!ri.@base)
                continue;

            var newId = ri.layerId;

            for (;;)
            {
                var oldId = 0xff;

                for (var j = 0; j < nregs; ++j)
                {
                    if (i == j)
                        continue;
                    var rj = regs[j];
                    if (!rj.@base)
                        continue;

                    // Skip if the regions are not close to each other.
                    if (!OverlapRange(ri.ymin, ri.ymax + mergeHeight, rj.ymin, rj.ymax + mergeHeight))
                        continue;
                    // Skip if the height range would become too large.
                    var ymin = Math.Min(ri.ymin, rj.ymin);
                    var ymax = Math.Max(ri.ymax, rj.ymax);
                    if (ymax - ymin >= 255)
                        continue;

                    // Make sure that there is no overlap when merging 'ri' and 'rj'.
                    var overlap = false;

                    // Iterate over all regions which have the same layerId as 'rj'
                    for (var k = 0; k < nregs; ++k)
                    {
                        if (regs[k].layerId != rj.layerId)
                            continue;

                        // Check if region 'k' is overlapping region 'ri'
                        // Index to 'regs' is the same as region id.
                        if (Contains(ri.layers, k))
                        {
                            overlap = true;
                            break;
                        }
                    }

                    // Cannot merge of regions overlap.
                    if (overlap)
                        continue;

                    // Can merge i and j.
                    oldId = rj.layerId;
                    break;
                }

                // Could not find anything to merge with, stop.
                if (oldId == 0xff)
                    break;

                // Merge
                for (var j = 0; j < nregs; ++j)
                {
                    var rj = regs[j];

                    if (rj.layerId == oldId)
                    {
                        rj.@base   = false;
                        // Remap layerIds.
                        rj.layerId = newId;
                        // Add overlaid layers from 'rj' to 'ri'.
                        foreach (var layer in rj.layers)
                            AddUnique(ri.layers, layer);

                        // Update height bounds.
                        ri.ymin = Math.Min(ri.ymin, rj.ymin);
                        ri.ymax = Math.Max(ri.ymax, rj.ymax);
                    }
                }
            }
        }

        // Compact layerIds
        Span<byte> remap = stackalloc byte[256];

        // Find number of unique layers.
        layerId = 0;
        for (var i = 0; i < nregs; ++i)
            remap[regs[i].layerId] = 1;
        for (var i = 0; i < 256; ++i)
            if (remap[i] != 0)
                remap[i] = layerId++;
            else
                remap[i] = 0xff;

        // Remap ids.
        for (var i = 0; i < nregs; ++i)
            regs[i].layerId = remap[regs[i].layerId];

        // No layers, return empty.
        if (layerId == 0)
            return true;

        // Create layers.
        // RcAssert(lset.layers == 0);

        var lw = w - (borderSize * 2);
        var lh = h - (borderSize * 2);

        // Build contracted bbox for layers.
        var bmin = chf.bmin;
        var bmax = chf.bmax;
        bmin.X += borderSize * chf.cs;
        bmin.Z += borderSize * chf.cs;
        bmax.X -= borderSize * chf.cs;
        bmax.Z -= borderSize * chf.cs;

        lset        = new RcHeightfieldLayerSet();
        lset.layers = new RcHeightfieldLayer[layerId];
        for (var i = 0; i < lset.layers.Length; i++)
            lset.layers[i] = new RcHeightfieldLayer();

        // Store layers.
        for (var i = 0; i < lset.layers.Length; ++i)
        {
            var curId = i;

            var layer = lset.layers[i];

            var gridSize = lw * lh;

            layer.heights = new int[gridSize];
            Array.Fill(layer.heights, 0xFF);
            layer.areas = new int[gridSize];
            layer.cons  = new int[gridSize];

            // Find layer height bounds.
            int hmin = 0, hmax = 0;

            for (var j = 0; j < nregs; ++j)
                if (regs[j].@base && regs[j].layerId == curId)
                {
                    hmin = regs[j].ymin;
                    hmax = regs[j].ymax;
                }

            layer.width  = lw;
            layer.height = lh;
            layer.cs     = chf.cs;
            layer.ch     = chf.ch;

            // Adjust the bbox to fit the heightfield.
            layer.bmin   = bmin;
            layer.bmax   = bmax;
            layer.bmin.Y = bmin.Y + (hmin * chf.ch);
            layer.bmax.Y = bmin.Y + (hmax * chf.ch);
            layer.hmin   = hmin;
            layer.hmax   = hmax;

            // Update usable data region.
            layer.minx = layer.width;
            layer.maxx = 0;
            layer.miny = layer.height;
            layer.maxy = 0;

            // Copy height and area from compact heightfield.
            for (var y = 0; y < lh; ++y)
            for (var x = 0; x < lw; ++x)
            {
                var     cx = borderSize + x;
                var     cy = borderSize + y;
                ref var c  = ref chf.cells[cx + (cy * w)];

                for (int j = c.index, nj = c.index + c.count; j < nj; ++j)
                {
                    ref var s = ref chf.spans[j];
                    // Skip unassigned regions.
                    if (srcReg[j] == 0xff)
                        continue;
                    // Skip of does nto belong to current layer.
                    int lid = regs[srcReg[j]].layerId;
                    if (lid != curId)
                        continue;

                    // Update data bounds.
                    layer.minx = Math.Min(layer.minx, x);
                    layer.maxx = Math.Max(layer.maxx, x);
                    layer.miny = Math.Min(layer.miny, y);
                    layer.maxy = Math.Max(layer.maxy, y);

                    // Store height and area type.
                    var idx = x                     + (y * lw);
                    layer.heights[idx] = (char)(s.y - hmin);
                    layer.areas[idx]   = chf.areas[j];

                    // Check connection.
                    var portal = (char)0;
                    var con    = (char)0;

                    for (var dir = 0; dir < 4; ++dir)
                        if (GetCon(s, dir) != RC_NOT_CONNECTED)
                        {
                            var ax = cx                             + GetDirOffsetX(dir);
                            var ay = cy                             + GetDirOffsetY(dir);
                            var ai = chf.cells[ax + (ay * w)].index + GetCon(s, dir);
                            var alid = srcReg[ai] != 0xff ?
                                           regs[srcReg[ai]].layerId :
                                           0xff;

                            // Portal mask
                            if (chf.areas[ai] != RC_NULL_AREA && lid != alid)
                            {
                                portal |= (char)(1 << dir);
                                // Update height so that it matches on both sides of the portal.
                                ref var @as = ref chf.spans[ai];
                                if (@as.y > hmin)
                                    layer.heights[idx] = Math.Max(layer.heights[idx], (char)(@as.y - hmin));
                            }

                            // Valid connection mask
                            if (chf.areas[ai] != RC_NULL_AREA && lid == alid)
                            {
                                var nx = ax - borderSize;
                                var ny = ay - borderSize;
                                if (nx >= 0 && ny >= 0 && nx < lw && ny < lh)
                                    con |= (char)(1 << dir);
                            }
                        }

                    layer.cons[idx] = (portal << 4) | con;
                }
            }

            if (layer.minx > layer.maxx)
                layer.minx = layer.maxx = 0;
            if (layer.miny > layer.maxy)
                layer.miny = layer.maxy = 0;
        }

        return true;
    }
}
