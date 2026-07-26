/*
recast4j copyright (c) 2021 Piotr Piastucki piotr@jtilia.org
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
using DotRecast.Core;

namespace DotRecast.Recast;

using static RcRecast;

public static class RcCompacts
{
    private const int MAX_HEIGHT = RC_SPAN_MAX_HEIGHT;

    /// @}
    /// @name Compact Heightfield Functions
    /// @see rcCompactHeightfield
    /// @{
    /// Builds a compact heightfield representing open space, from a heightfield representing solid space.
    /// 
    /// This is just the beginning of the process of fully building a compact heightfield.
    /// Various filters may be applied, then the distance field and regions built.
    /// E.g: #rcBuildDistanceField and #rcBuildRegions
    /// 
    /// See the #rcConfig documentation for more information on the configuration parameters.
    /// 
    /// @see rcAllocCompactHeightfield, rcHeightfield, rcCompactHeightfield, rcConfig
    /// @ingroup recast
    /// 
    /// @param[in,out]	context				The build context to use during the operation.
    /// @param[in]		walkableHeight		Minimum floor to 'ceiling' height that will still allow the floor area 
    /// to be considered walkable. [Limit: >= 3] [Units: vx]
    /// @param[in]		walkableClimb		Maximum ledge height that is considered to still be traversable. 
    /// [Limit: >=0] [Units: vx]
    /// @param[in]		heightfield			The heightfield to be compacted.
    /// @param[out]		compactHeightfield	The resulting compact heightfield. (Must be pre-allocated.)
    /// @returns True if the operation completed successfully.
    public static RcCompactHeightfield BuildCompactHeightfield
    (
        RcContext     context,
        int           walkableHeight,
        int           walkableClimb,
        RcHeightfield heightfield
    )
    {
        using var timer = context.ScopedTimer(RcTimerLabel.RC_TIMER_BUILD_COMPACTHEIGHTFIELD);

        var xSize     = heightfield.width;
        var zSize     = heightfield.height;
        var spanCount = GetHeightFieldSpanCount(context, heightfield);

        // Fill in header.
        var compactHeightfield = new RcCompactHeightfield();
        compactHeightfield.width          =  xSize;
        compactHeightfield.height         =  zSize;
        compactHeightfield.borderSize     =  heightfield.borderSize;
        compactHeightfield.spanCount      =  spanCount;
        compactHeightfield.walkableHeight =  walkableHeight;
        compactHeightfield.walkableClimb  =  walkableClimb;
        compactHeightfield.maxRegions     =  0;
        compactHeightfield.bmin           =  heightfield.bmin;
        compactHeightfield.bmax           =  heightfield.bmax;
        compactHeightfield.bmax.Y         += walkableHeight * heightfield.ch;
        compactHeightfield.cs             =  heightfield.cs;
        compactHeightfield.ch             =  heightfield.ch;
        compactHeightfield.cells          =  new RcCompactCell[xSize * zSize];
        //chf.spans = new RcCompactSpan[spanCount];
        compactHeightfield.areas          =  new int[spanCount];

        var tempSpans = new RcCompactSpanBuilder[spanCount];

        // Fill in cells and spans.
        var currentCellIndex = 0;
        var numColumns       = xSize * zSize;

        for (var columnIndex = 0; columnIndex < numColumns; ++columnIndex)
        {
            var spanIndex = heightfield.spans[columnIndex];
            var tmpIdx    = currentCellIndex;
            var tmpCount  = 0;

            while (spanIndex != 0)
            {
                ref var span = ref heightfield.Span(spanIndex);

                if (span.area != RC_NULL_AREA)
                {
                    var bot = span.smax;
                    var top = span.next != 0 ?
                                  heightfield.Span(span.next).smin :
                                  MAX_HEIGHT;
                    tempSpans[currentCellIndex].y              = Math.Clamp(bot,       0, MAX_HEIGHT);
                    tempSpans[currentCellIndex].h              = Math.Clamp(top - bot, 0, MAX_HEIGHT);
                    compactHeightfield.areas[currentCellIndex] = span.area;
                    currentCellIndex++;
                    tmpCount++;
                }

                spanIndex = span.next;
            }

            compactHeightfield.cells[columnIndex] = new RcCompactCell(tmpIdx, tmpCount);
        }

        // Find neighbour connections.
        const int MAX_LAYERS    = RC_NOT_CONNECTED - 1;
        var       maxLayerIndex = 0;
        var       zStride       = xSize; // for readability

        for (var z = 0; z < zSize; ++z)
        for (var x = 0; x < xSize; ++x)
        {
            ref var cell = ref compactHeightfield.cells[x + (z * zStride)];

            for (int i = cell.index, ni = cell.index + cell.count; i < ni; ++i)
            {
                ref var s = ref tempSpans[i];

                for (var dir = 0; dir < 4; ++dir)
                {
                    SetCon(ref s, dir, RC_NOT_CONNECTED);
                    var neighborX = x + GetDirOffsetX(dir);
                    var neighborZ = z + GetDirOffsetY(dir);

                    // First check that the neighbour cell is in bounds.
                    if (neighborX < 0 || neighborZ < 0 || neighborX >= xSize || neighborZ >= zSize)
                        continue;

                    // Iterate over all neighbour spans and check if any of the is
                    // accessible from current cell.
                    ref var neighborCell = ref compactHeightfield.cells[neighborX + (neighborZ * xSize)];

                    for (int k = neighborCell.index, nk = neighborCell.index + neighborCell.count; k < nk; ++k)
                    {
                        ref var neighborSpan = ref tempSpans[k];
                        var     bot          = Math.Max(s.y, neighborSpan.y);
                        var     top          = Math.Min(s.y + s.h, neighborSpan.y + neighborSpan.h);

                        // Check that the gap between the spans is walkable,
                        // and that the climb height between the gaps is not too high.
                        if (top - bot >= walkableHeight && MathF.Abs(neighborSpan.y - s.y) <= walkableClimb)
                        {
                            // Mark direction as walkable.
                            var layerIndex = k - neighborCell.index;

                            if (layerIndex < 0 || layerIndex > MAX_LAYERS)
                            {
                                maxLayerIndex = Math.Max(maxLayerIndex, layerIndex);
                                continue;
                            }

                            SetCon(ref s, dir, layerIndex);
                            break;
                        }
                    }
                }
            }
        }

        if (maxLayerIndex > MAX_LAYERS)
            throw new Exception($"rcBuildCompactHeightfield: Heightfield has too many layers {maxLayerIndex} (max: {MAX_LAYERS})");

        compactHeightfield.spans = GC.AllocateUninitializedArray<RcCompactSpan>(spanCount);
        for (var i = 0; i < spanCount; ++i)
            compactHeightfield.spans[i] = tempSpans[i].Build();

        return compactHeightfield;
    }

    /// Returns the number of spans contained in the specified heightfield.
    /// @ingroup recast
    /// @param[in,out]	context		The build context to use during the operation.
    /// @param[in]		heightfield	An initialized heightfield.
    /// @returns The number of spans in the heightfield.
    private static int GetHeightFieldSpanCount
    (
        RcContext     context,
        RcHeightfield heightfield
    )
    {
        var numCols   = heightfield.width * heightfield.height;
        var spanCount = 0;
        for (var columnIndex = 0; columnIndex < numCols; ++columnIndex)
        for (var span = heightfield.spans[columnIndex]; span != 0; span = heightfield.Span(span).next)
            if (heightfield.Span(span).area != RC_NULL_AREA)
                spanCount++;

        return spanCount;
    }
}
