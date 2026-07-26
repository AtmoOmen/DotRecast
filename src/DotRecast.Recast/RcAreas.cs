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
using DotRecast.Core;
using DotRecast.Core.Collections.Extensions;
using DotRecast.Core.Numerics;

namespace DotRecast.Recast;

using static RcRecast;

public static class RcAreas
{
    /// Erodes the walkable area within the heightfield by the specified radius.
    /// 
    /// Basically, any spans that are closer to a boundary or obstruction than the specified radius 
    /// are marked as un-walkable.
    /// 
    /// This method is usually called immediately after the heightfield has been built.
    /// 
    /// @see rcCompactHeightfield, rcBuildCompactHeightfield, rcConfig::walkableRadius
    /// @ingroup recast
    /// 
    /// @param[in,out]	context				The build context to use during the operation.
    /// @param[in]		erosionRadius		The radius of erosion. [Limits: 0
    /// < value
    /// < 255] [Units: vx]
    ///     @ param[ in, out] compactHeightfield The populated compact heightfield to erode.
    ///     @ returns True if the operation completed successfully.
    public static void ErodeWalkableArea
    (
        RcContext            context,
        int                  erosionRadius,
        RcCompactHeightfield compactHeightfield
    )
    {
        var xSize   = compactHeightfield.width;
        var zSize   = compactHeightfield.height;
        var zStride = xSize; // For readability

        using var timer = context.ScopedTimer(RcTimerLabel.RC_TIMER_ERODE_AREA);

        var distanceToBoundary = new int[compactHeightfield.spanCount];
        Array.Fill(distanceToBoundary, 255);

        // Mark boundary cells.
        for (var z = 0; z < zSize; ++z)
        for (var x = 0; x < xSize; ++x)
        {
            ref var cell = ref compactHeightfield.cells[x + (z * zStride)];

            for (int spanIndex = cell.index, maxSpanIndex = cell.index + cell.count; spanIndex < maxSpanIndex; ++spanIndex)
                if (compactHeightfield.areas[spanIndex] == RC_NULL_AREA)
                    distanceToBoundary[spanIndex] = 0;
                else
                {
                    ref var span = ref compactHeightfield.spans[spanIndex];

                    // Check that there is a non-null adjacent span in each of the 4 cardinal directions.
                    var neighborCount = 0;

                    for (var direction = 0; direction < 4; ++direction)
                    {
                        var neighborConnection = GetCon(span, direction);
                        if (neighborConnection == RC_NOT_CONNECTED)
                            break;

                        var neighborX         = x                                                                 + GetDirOffsetX(direction);
                        var neighborZ         = z                                                                 + GetDirOffsetY(direction);
                        var neighborSpanIndex = compactHeightfield.cells[neighborX + (neighborZ * zStride)].index + GetCon(span, direction);
                        if (compactHeightfield.areas[neighborSpanIndex] == RC_NULL_AREA)
                            break;

                        neighborCount++;
                    }

                    // At least one missing neighbour, so this is a boundary cell.
                    if (neighborCount != 4)
                        distanceToBoundary[spanIndex] = 0;
                }
        }

        int newDistance;

        // Pass 1
        for (var z = 0; z < zSize; ++z)
        for (var x = 0; x < xSize; ++x)
        {
            ref var cell         = ref compactHeightfield.cells[x + (z * zStride)];
            var     maxSpanIndex = cell.index + cell.count;

            for (var spanIndex = cell.index; spanIndex < maxSpanIndex; ++spanIndex)
            {
                ref var span = ref compactHeightfield.spans[spanIndex];

                if (GetCon(span, 0) != RC_NOT_CONNECTED)
                {
                    // (-1,0)
                    var     aX     = x                                                 + GetDirOffsetX(0);
                    var     aY     = z                                                 + GetDirOffsetY(0);
                    var     aIndex = compactHeightfield.cells[aX + (aY * xSize)].index + GetCon(span, 0);
                    ref var aSpan  = ref compactHeightfield.spans[aIndex];
                    newDistance = Math.Min(distanceToBoundary[aIndex] + 2, 255);
                    if (newDistance < distanceToBoundary[spanIndex])
                        distanceToBoundary[spanIndex] = newDistance;

                    // (-1,-1)
                    if (GetCon(aSpan, 3) != RC_NOT_CONNECTED)
                    {
                        var bX     = aX                                                + GetDirOffsetX(3);
                        var bY     = aY                                                + GetDirOffsetY(3);
                        var bIndex = compactHeightfield.cells[bX + (bY * xSize)].index + GetCon(aSpan, 3);
                        newDistance = Math.Min(distanceToBoundary[bIndex] + 3, 255);
                        if (newDistance < distanceToBoundary[spanIndex])
                            distanceToBoundary[spanIndex] = newDistance;
                    }
                }

                if (GetCon(span, 3) != RC_NOT_CONNECTED)
                {
                    // (0,-1)
                    var     aX     = x                                                 + GetDirOffsetX(3);
                    var     aY     = z                                                 + GetDirOffsetY(3);
                    var     aIndex = compactHeightfield.cells[aX + (aY * xSize)].index + GetCon(span, 3);
                    ref var aSpan  = ref compactHeightfield.spans[aIndex];
                    newDistance = Math.Min(distanceToBoundary[aIndex] + 2, 255);
                    if (newDistance < distanceToBoundary[spanIndex])
                        distanceToBoundary[spanIndex] = newDistance;

                    // (1,-1)
                    if (GetCon(aSpan, 2) != RC_NOT_CONNECTED)
                    {
                        var bX     = aX                                                + GetDirOffsetX(2);
                        var bY     = aY                                                + GetDirOffsetY(2);
                        var bIndex = compactHeightfield.cells[bX + (bY * xSize)].index + GetCon(aSpan, 2);
                        newDistance = Math.Min(distanceToBoundary[bIndex] + 3, 255);
                        if (newDistance < distanceToBoundary[spanIndex])
                            distanceToBoundary[spanIndex] = newDistance;
                    }
                }
            }
        }

        // Pass 2
        for (var z = zSize - 1; z >= 0; --z)
        for (var x = xSize - 1; x >= 0; --x)
        {
            ref var cell         = ref compactHeightfield.cells[x + (z * zStride)];
            var     maxSpanIndex = cell.index + cell.count;

            for (var i = cell.index; i < maxSpanIndex; ++i)
            {
                ref var span = ref compactHeightfield.spans[i];

                if (GetCon(span, 2) != RC_NOT_CONNECTED)
                {
                    // (1,0)
                    var     aX     = x                                                 + GetDirOffsetX(2);
                    var     aY     = z                                                 + GetDirOffsetY(2);
                    var     aIndex = compactHeightfield.cells[aX + (aY * xSize)].index + GetCon(span, 2);
                    ref var aSpan  = ref compactHeightfield.spans[aIndex];
                    newDistance = Math.Min(distanceToBoundary[aIndex] + 2, 255);
                    if (newDistance < distanceToBoundary[i])
                        distanceToBoundary[i] = newDistance;

                    // (1,1)
                    if (GetCon(aSpan, 1) != RC_NOT_CONNECTED)
                    {
                        var bX     = aX                                                + GetDirOffsetX(1);
                        var bY     = aY                                                + GetDirOffsetY(1);
                        var bIndex = compactHeightfield.cells[bX + (bY * xSize)].index + GetCon(aSpan, 1);
                        newDistance = Math.Min(distanceToBoundary[bIndex] + 3, 255);
                        if (newDistance < distanceToBoundary[i])
                            distanceToBoundary[i] = newDistance;
                    }
                }

                if (GetCon(span, 1) != RC_NOT_CONNECTED)
                {
                    // (0,1)
                    var     aX     = x                                                 + GetDirOffsetX(1);
                    var     aY     = z                                                 + GetDirOffsetY(1);
                    var     aIndex = compactHeightfield.cells[aX + (aY * xSize)].index + GetCon(span, 1);
                    ref var aSpan  = ref compactHeightfield.spans[aIndex];
                    newDistance = Math.Min(distanceToBoundary[aIndex] + 2, 255);
                    if (newDistance < distanceToBoundary[i])
                        distanceToBoundary[i] = newDistance;

                    // (-1,1)
                    if (GetCon(aSpan, 0) != RC_NOT_CONNECTED)
                    {
                        var bX     = aX                                                + GetDirOffsetX(0);
                        var bY     = aY                                                + GetDirOffsetY(0);
                        var bIndex = compactHeightfield.cells[bX + (bY * xSize)].index + GetCon(aSpan, 0);
                        newDistance = Math.Min(distanceToBoundary[bIndex] + 3, 255);
                        if (newDistance < distanceToBoundary[i])
                            distanceToBoundary[i] = newDistance;
                    }
                }
            }
        }

        var minBoundaryDistance = erosionRadius * 2;
        for (var spanIndex = 0; spanIndex < compactHeightfield.spanCount; ++spanIndex)
            if (distanceToBoundary[spanIndex] < minBoundaryDistance)
                compactHeightfield.areas[spanIndex] = RC_NULL_AREA;
    }

    /// Applies a median filter to walkable area types (based on area id), removing noise.
    /// 
    /// This filter is usually applied after applying area id's using functions
    /// such as #rcMarkBoxArea, #rcMarkConvexPolyArea, and #rcMarkCylinderArea.
    /// 
    /// @see rcCompactHeightfield
    /// @ingroup recast
    /// 
    /// @param[in,out]	context		The build context to use during the operation.
    /// @param[in,out]	compactHeightfield		A populated compact heightfield.
    /// @returns True if the operation completed successfully.
    public static bool MedianFilterWalkableArea
    (
        RcContext            context,
        RcCompactHeightfield compactHeightfield
    )
    {
        var xSize   = compactHeightfield.width;
        var zSize   = compactHeightfield.height;
        var zStride = xSize; // For readability

        using var timer = context.ScopedTimer(RcTimerLabel.RC_TIMER_MEDIAN_AREA);

        var areas = new int[compactHeightfield.spanCount];

        for (var z = 0; z < zSize; ++z)
        for (var x = 0; x < xSize; ++x)
        {
            ref var cell         = ref compactHeightfield.cells[x + (z * zStride)];
            var     maxSpanIndex = cell.index + cell.count;

            for (var spanIndex = cell.index; spanIndex < maxSpanIndex; ++spanIndex)
            {
                ref var span = ref compactHeightfield.spans[spanIndex];

                if (compactHeightfield.areas[spanIndex] == RC_NULL_AREA)
                {
                    areas[spanIndex] = compactHeightfield.areas[spanIndex];
                    continue;
                }

                var neighborAreas = new int[9];
                for (var neighborIndex = 0; neighborIndex < 9; ++neighborIndex)
                    neighborAreas[neighborIndex] = compactHeightfield.areas[spanIndex];

                for (var dir = 0; dir < 4; ++dir)
                {
                    if (GetCon(span, dir) == RC_NOT_CONNECTED)
                        continue;

                    var aX     = x                                                   + GetDirOffsetX(dir);
                    var aZ     = z                                                   + GetDirOffsetY(dir);
                    var aIndex = compactHeightfield.cells[aX + (aZ * zStride)].index + GetCon(span, dir);
                    if (compactHeightfield.areas[aIndex] != RC_NULL_AREA)
                        neighborAreas[(dir * 2) + 0] = compactHeightfield.areas[aIndex];

                    ref var aSpan               = ref compactHeightfield.spans[aIndex];
                    var     dir2                = (dir + 1) & 0x3;
                    var     neighborConnection2 = GetCon(aSpan, dir2);

                    if (neighborConnection2 != RC_NOT_CONNECTED)
                    {
                        var bX     = aX                                                  + GetDirOffsetX(dir2);
                        var bZ     = aZ                                                  + GetDirOffsetY(dir2);
                        var bIndex = compactHeightfield.cells[bX + (bZ * zStride)].index + GetCon(aSpan, dir2);
                        if (compactHeightfield.areas[bIndex] != RC_NULL_AREA)
                            neighborAreas[(dir * 2) + 1] = compactHeightfield.areas[bIndex];
                    }
                }

                //Array.Sort(neighborAreas);
                neighborAreas.InsertSort();
                areas[spanIndex] = neighborAreas[4];
            }
        }

        compactHeightfield.areas = areas;

        return true;
    }

    /// Applies an area id to all spans within the specified bounding box. (AABB) 
    /// 
    /// @see rcCompactHeightfield, rcMedianFilterWalkableArea
    /// @ingroup recast
    /// 
    /// @param[in,out]	context				The build context to use during the operation.
    /// @param[in]		boxMinBounds		The minimum extents of the bounding box. [(x, y, z)] [Units: wu]
    /// @param[in]		boxMaxBounds		The maximum extents of the bounding box. [(x, y, z)] [Units: wu]
    /// @param[in]		areaId				The area id to apply. [Limit:
    /// <
    /// =
    /// #
    /// RC_WALKABLE_AREA
    /// ]
    /// @
    /// param
    /// [
    /// in
    /// ,
    /// out
    /// ]
    /// compactHeightfield
    /// A
    /// populated
    /// compact
    /// heightfield.
    public static void MarkBoxArea
    (
        RcContext            context,
        float[]              boxMinBounds,
        float[]              boxMaxBounds,
        RcAreaModification   areaId,
        RcCompactHeightfield compactHeightfield
    )
    {
        using var timer = context.ScopedTimer(RcTimerLabel.RC_TIMER_MARK_BOX_AREA);

        var xSize   = compactHeightfield.width;
        var zSize   = compactHeightfield.height;
        var zStride = xSize; // For readability

        // Find the footprint of the box area in grid cell coordinates. 
        var minX = (int)((boxMinBounds[0] - compactHeightfield.bmin.X) / compactHeightfield.cs);
        var minY = (int)((boxMinBounds[1] - compactHeightfield.bmin.Y) / compactHeightfield.ch);
        var minZ = (int)((boxMinBounds[2] - compactHeightfield.bmin.Z) / compactHeightfield.cs);
        var maxX = (int)((boxMaxBounds[0] - compactHeightfield.bmin.X) / compactHeightfield.cs);
        var maxY = (int)((boxMaxBounds[1] - compactHeightfield.bmin.Y) / compactHeightfield.ch);
        var maxZ = (int)((boxMaxBounds[2] - compactHeightfield.bmin.Z) / compactHeightfield.cs);

        if (maxX < 0)
            return;

        if (minX >= xSize)
            return;

        if (maxZ < 0)
            return;

        if (minZ >= zSize)
            return;

        if (minX < 0)
            minX = 0;

        if (maxX >= xSize)
            maxX = xSize - 1;

        if (minZ < 0)
            minZ = 0;

        if (maxZ >= zSize)
            maxZ = zSize - 1;

        for (var z = minZ; z <= maxZ; ++z)
        for (var x = minX; x <= maxX; ++x)
        {
            ref var cell         = ref compactHeightfield.cells[x + (z * zStride)];
            var     maxSpanIndex = cell.index + cell.count;

            for (var spanIndex = cell.index; spanIndex < maxSpanIndex; ++spanIndex)
            {
                ref var span = ref compactHeightfield.spans[spanIndex];

                // Skip if the span is outside the box extents.
                if (span.y < minY || span.y > maxY)
                    continue;

                // Skip if the span has been removed.
                if (compactHeightfield.areas[spanIndex] == RC_NULL_AREA)
                    continue;

                // Mark the span.
                compactHeightfield.areas[spanIndex] = areaId.Apply(compactHeightfield.areas[spanIndex]);
            }
        }
    }

    /// Applies the area id to the all spans within the specified convex polygon. 
    /// 
    /// The value of spacial parameters are in world units.
    /// 
    /// The y-values of the polygon vertices are ignored. So the polygon is effectively 
    /// projected onto the xz-plane, translated to @p minY, and extruded to @p maxY.
    /// 
    /// @see rcCompactHeightfield, rcMedianFilterWalkableArea
    /// @ingroup recast
    /// 
    /// @param[in,out]	context				The build context to use during the operation.
    /// @param[in]		verts				The vertices of the polygon [For: (x, y, z) * @p numVerts]
    /// @param[in]		numVerts			The number of vertices in the polygon.
    /// @param[in]		minY				The height of the base of the polygon. [Units: wu]
    /// @param[in]		maxY				The height of the top of the polygon. [Units: wu]
    /// @param[in]		areaId				The area id to apply. [Limit:
    /// <
    /// =
    /// #
    /// RC_WALKABLE_AREA
    /// ]
    /// @
    /// param
    /// [
    /// in
    /// ,
    /// out
    /// ]
    /// compactHeightfield
    /// A
    /// populated
    /// compact
    /// heightfield.
    public static void MarkConvexPolyArea
    (
        RcContext            context,
        float[]              verts,
        float                minY,
        float                maxY,
        RcAreaModification   areaId,
        RcCompactHeightfield compactHeightfield
    )
    {
        using var timer = context.ScopedTimer(RcTimerLabel.RC_TIMER_MARK_CONVEXPOLY_AREA);

        var xSize   = compactHeightfield.width;
        var zSize   = compactHeightfield.height;
        var zStride = xSize; // For readability

        // Compute the bounding box of the polygon
        var bmin = verts.ToVec3();
        var bmax = verts.ToVec3();

        for (var i = 3; i < verts.Length; i += 3)
        {
            bmin = RcVec3f.Min(bmin, verts.ToVec3(i));
            bmax = RcVec3f.Max(bmax, verts.ToVec3(i));
        }

        bmin.Y = minY;
        bmax.Y = maxY;

        // Compute the grid footprint of the polygon 
        var minx = (int)((bmin.X - compactHeightfield.bmin.X) / compactHeightfield.cs);
        var miny = (int)((bmin.Y - compactHeightfield.bmin.Y) / compactHeightfield.ch);
        var minz = (int)((bmin.Z - compactHeightfield.bmin.Z) / compactHeightfield.cs);
        var maxx = (int)((bmax.X - compactHeightfield.bmin.X) / compactHeightfield.cs);
        var maxy = (int)((bmax.Y - compactHeightfield.bmin.Y) / compactHeightfield.ch);
        var maxz = (int)((bmax.Z - compactHeightfield.bmin.Z) / compactHeightfield.cs);

        // Early-out if the polygon lies entirely outside the grid.
        if (maxx < 0)
            return;

        if (minx >= xSize)
            return;

        if (maxz < 0)
            return;

        if (minz >= zSize)
            return;

        // Clamp the polygon footprint to the grid
        if (minx < 0)
            minx = 0;

        if (maxx >= xSize)
            maxx = xSize - 1;

        if (minz < 0)
            minz = 0;

        if (maxz >= zSize)
            maxz = zSize - 1;

        // TODO: Optimize.
        for (var z = minz; z <= maxz; ++z)
        for (var x = minx; x <= maxx; ++x)
        {
            ref var cell         = ref compactHeightfield.cells[x + (z * zStride)];
            var     maxSpanIndex = cell.index + cell.count;

            for (var spanIndex = cell.index; spanIndex < maxSpanIndex; ++spanIndex)
            {
                ref var span = ref compactHeightfield.spans[spanIndex];

                // Skip if span is removed.
                if (compactHeightfield.areas[spanIndex] == RC_NULL_AREA)
                    continue;

                // Skip if y extents don't overlap.
                if (span.y < miny || span.y > maxy)
                    continue;

                var point = new RcVec3f
                (
                    compactHeightfield.bmin.X + ((x + 0.5f) * compactHeightfield.cs),
                    0,
                    compactHeightfield.bmin.Z + ((z + 0.5f) * compactHeightfield.cs)
                );

                if (PointInPoly(verts, point))
                    compactHeightfield.areas[spanIndex] = areaId.Apply(compactHeightfield.areas[spanIndex]);
            }
        }
    }


    /// Applies the area id to all spans within the specified y-axis-aligned cylinder.
    /// 
    /// @see rcCompactHeightfield, rcMedianFilterWalkableArea
    /// 
    /// @ingroup recast
    /// 
    /// @param[in,out]	context				The build context to use during the operation.
    /// @param[in]		position			The center of the base of the cylinder. [Form: (x, y, z)] [Units: wu] 
    /// @param[in]		radius				The radius of the cylinder. [Units: wu] [Limit: > 0]
    /// @param[in]		height				The height of the cylinder. [Units: wu] [Limit: > 0]
    /// @param[in]		areaId				The area id to apply. [Limit:
    /// <
    /// =
    /// #
    /// RC_WALKABLE_AREA
    /// ]
    /// @
    /// param
    /// [
    /// in
    /// ,
    /// out
    /// ]
    /// compactHeightfield
    /// A
    /// populated
    /// compact
    /// heightfield.
    public static void MarkCylinderArea
    (
        RcContext            context,
        float[]              position,
        float                radius,
        float                height,
        RcAreaModification   areaId,
        RcCompactHeightfield compactHeightfield
    )
    {
        using var timer = context.ScopedTimer(RcTimerLabel.RC_TIMER_MARK_CYLINDER_AREA);

        var xSize   = compactHeightfield.width;
        var zSize   = compactHeightfield.height;
        var zStride = xSize; // For readability

        // Compute the bounding box of the cylinder
        var cylinderBBMin = new RcVec3f
        (
            position[0] - radius,
            position[1],
            position[2] - radius
        );

        var cylinderBBMax = new RcVec3f
        (
            position[0] + radius,
            position[1] + height,
            position[2] + radius
        );

        // Compute the grid footprint of the cylinder
        var minx = (int)((cylinderBBMin.X - compactHeightfield.bmin.X) / compactHeightfield.cs);
        var miny = (int)((cylinderBBMin.Y - compactHeightfield.bmin.Y) / compactHeightfield.ch);
        var minz = (int)((cylinderBBMin.Z - compactHeightfield.bmin.Z) / compactHeightfield.cs);
        var maxx = (int)((cylinderBBMax.X - compactHeightfield.bmin.X) / compactHeightfield.cs);
        var maxy = (int)((cylinderBBMax.Y - compactHeightfield.bmin.Y) / compactHeightfield.ch);
        var maxz = (int)((cylinderBBMax.Z - compactHeightfield.bmin.Z) / compactHeightfield.cs);

        // Early-out if the cylinder is completely outside the grid bounds.
        if (maxx < 0)
            return;

        if (minx >= xSize)
            return;

        if (maxz < 0)
            return;

        if (minz >= zSize)
            return;

        // Clamp the cylinder bounds to the grid.
        if (minx < 0)
            minx = 0;

        if (maxx >= xSize)
            maxx = xSize - 1;

        if (minz < 0)
            minz = 0;

        if (maxz >= zSize)
            maxz = zSize - 1;

        var radiusSq = radius * radius;

        for (var z = minz; z <= maxz; ++z)
        for (var x = minx; x <= maxx; ++x)
        {
            ref var cell         = ref compactHeightfield.cells[x + (z * zStride)];
            var     maxSpanIndex = cell.index + cell.count;

            var cellX  = compactHeightfield.bmin.X + ((x + 0.5f) * compactHeightfield.cs);
            var cellZ  = compactHeightfield.bmin.Z + ((z + 0.5f) * compactHeightfield.cs);
            var deltaX = cellX                     - position[0];
            var deltaZ = cellZ                     - position[2];

            // Skip this column if it's too far from the center point of the cylinder.
            if (RcMath.Sqr(deltaX) + RcMath.Sqr(deltaZ) >= radiusSq)
                continue;

            // Mark all overlapping spans
            for (var spanIndex = cell.index; spanIndex < maxSpanIndex; ++spanIndex)
            {
                ref var span = ref compactHeightfield.spans[spanIndex];

                // Skip if span is removed.
                if (compactHeightfield.areas[spanIndex] == RC_NULL_AREA)
                    continue;

                // Mark if y extents overlap.
                if (span.y >= miny && span.y <= maxy)
                    compactHeightfield.areas[spanIndex] = areaId.Apply(compactHeightfield.areas[spanIndex]);
            }
        }
    }

    // public static bool PointInPoly(float[] verts, RcVec3f p)
    // {
    //     bool c = false;
    //     int i, j;
    //     for (i = 0, j = verts.Length - 3; i < verts.Length; j = i, i += 3)
    //     {
    //         int vi = i;
    //         int vj = j;
    //         if (((verts[vi + 2] > p.z) != (verts[vj + 2] > p.z))
    //             && (p.x < (verts[vj] - verts[vi]) * (p.z - verts[vi + 2]) / (verts[vj + 2] - verts[vi + 2])
    //                 + verts[vi]))
    //             c = !c;
    //     }
    //
    //     return c;
    // }

    // TODO (graham): This is duplicated in the ConvexVolumeTool in RecastDemo
    /// Checks if a point is contained within a polygon
    /// 
    /// @param[in]	numVerts	Number of vertices in the polygon
    /// @param[in]	verts		The polygon vertices
    /// @param[in]	point		The point to check
    /// @returns true if the point lies within the polygon, false otherwise.
    public static bool PointInPoly
    (
        float[] verts,
        RcVec3f point
    )
    {
        var inPoly = false;

        for (int i = 0, j = (verts.Length / 3) - 1; i < verts.Length / 3; j = i++)
        {
            var vi = new RcVec3f(verts[i * 3], verts[(i * 3) + 1], verts[(i * 3) + 2]);
            var vj = new RcVec3f(verts[j * 3], verts[(j * 3) + 1], verts[(j * 3) + 2]);
            if ((vi.Z > point.Z) == (vj.Z > point.Z))
                continue;

            if (point.X >= ((vj.X - vi.X) * (point.Z - vi.Z) / (vj.Z - vi.Z)) + vi.X)
                continue;

            inPoly = !inPoly;
        }

        return inPoly;
    }

    /// Expands a convex polygon along its vertex normals by the given offset amount.
    /// Inserts extra vertices to bevel sharp corners.
    /// 
    /// Helper function to offset convex polygons for rcMarkConvexPolyArea.
    /// 
    /// @ingroup recast
    /// 
    /// @param[in]		verts		The vertices of the polygon [Form: (x, y, z) * @p numVerts]
    /// @param[in]		numVerts	The number of vertices in the polygon.
    /// @param[in]		offset		How much to offset the polygon by. [Units: wu]
    /// @param[out]		outVerts	The offset vertices (should hold up to 2 * @p numVerts) [Form: (x, y, z) * return value]
    /// @param[in]		maxOutVerts	The max number of vertices that can be stored to @p outVerts.
    /// @returns Number of vertices in the offset polygon or 0 if too few vertices in @p outVerts.
    public static int OffsetPoly
    (
        float[] verts,
        int     numVerts,
        float   offset,
        float[] outVerts,
        int     maxOutVerts
    )
    {
        // Defines the limit at which a miter becomes a bevel.
        // Similar in behavior to https://developer.mozilla.org/en-US/docs/Web/SVG/Attribute/stroke-miterlimit
        const float MITER_LIMIT = 1.20f;

        var numOutVerts = 0;

        for (var vertIndex = 0; vertIndex < numVerts; vertIndex++)
        {
            var vertIndexA = (vertIndex + numVerts - 1) % numVerts;
            var vertIndexB = vertIndex;
            var vertIndexC = (vertIndex + 1) % numVerts;

            var vertA = verts.ToVec3(vertIndexA * 3);
            var vertB = verts.ToVec3(vertIndexB * 3);
            var vertC = verts.ToVec3(vertIndexC * 3);

            // From A to B on the x/z plane
            var prevSegmentDir = RcVec3f.Subtract(vertB, vertA);
            prevSegmentDir.Y = 0; // Squash onto x/z plane
            prevSegmentDir   = RcVec.SafeNormalize(prevSegmentDir);

            // From B to C on the x/z plane
            var currSegmentDir = RcVec3f.Subtract(vertC, vertB);
            currSegmentDir.Y = 0; // Squash onto x/z plane
            currSegmentDir   = RcVec.SafeNormalize(currSegmentDir);

            // The y component of the cross product of the two normalized segment directions.
            // The X and Z components of the cross product are both zero because the two
            // segment direction vectors fall within the x/z plane.
            var cross = (currSegmentDir.X * prevSegmentDir.Z) - (prevSegmentDir.X * currSegmentDir.Z);

            // CCW perpendicular vector to AB.  The segment normal.
            var prevSegmentNormX = -prevSegmentDir.Z;
            var prevSegmentNormZ = prevSegmentDir.X;

            // CCW perpendicular vector to BC.  The segment normal.
            var currSegmentNormX = -currSegmentDir.Z;
            var currSegmentNormZ = currSegmentDir.X;

            // Average the two segment normals to get the proportional miter offset for B.
            // This isn't normalized because it's defining the distance and direction the corner will need to be
            // adjusted proportionally to the edge offsets to properly miter the adjoining edges.
            var cornerMiterX     = (prevSegmentNormX + currSegmentNormX) * 0.5f;
            var cornerMiterZ     = (prevSegmentNormZ + currSegmentNormZ) * 0.5f;
            var cornerMiterSqMag = RcMath.Sqr(cornerMiterX) + RcMath.Sqr(cornerMiterZ);

            // If the magnitude of the segment normal average is less than about .69444,
            // the corner is an acute enough angle that the result should be beveled.
            var bevel = cornerMiterSqMag * MITER_LIMIT * MITER_LIMIT < 1.0f;

            // Scale the corner miter so it's proportional to how much the corner should be offset compared to the edges.
            if (cornerMiterSqMag > RcVec.EPSILON)
            {
                var scale = 1.0f / cornerMiterSqMag;
                cornerMiterX *= scale;
                cornerMiterZ *= scale;
            }

            if (bevel && cross < 0.0f) // If the corner is convex and an acute enough angle, generate a bevel.
            {
                if (numOutVerts + 2 > maxOutVerts)
                    return 0;

                // Generate two bevel vertices at a distances from B proportional to the angle between the two segments.
                // Move each bevel vertex out proportional to the given offset.
                var d = (1.0f - ((prevSegmentDir.X * currSegmentDir.X) + (prevSegmentDir.Z * currSegmentDir.Z))) * 0.5f;

                outVerts[(numOutVerts * 3) + 0] = vertB.X + ((-prevSegmentNormX + (prevSegmentDir.X * d)) * offset);
                outVerts[(numOutVerts * 3) + 1] = vertB.Y;
                outVerts[(numOutVerts * 3) + 2] = vertB.Z + ((-prevSegmentNormZ + (prevSegmentDir.Z * d)) * offset);
                numOutVerts++;

                outVerts[(numOutVerts * 3) + 0] = vertB.X + ((-currSegmentNormX - (currSegmentDir.X * d)) * offset);
                outVerts[(numOutVerts * 3) + 1] = vertB.Y;
                outVerts[(numOutVerts * 3) + 2] = vertB.Z + ((-currSegmentNormZ - (currSegmentDir.Z * d)) * offset);
                numOutVerts++;
            }
            else
            {
                if (numOutVerts + 1 > maxOutVerts)
                    return 0;

                // Move B along the miter direction by the specified offset.
                outVerts[(numOutVerts * 3) + 0] = vertB.X - (cornerMiterX * offset);
                outVerts[(numOutVerts * 3) + 1] = vertB.Y;
                outVerts[(numOutVerts * 3) + 2] = vertB.Z - (cornerMiterZ * offset);
                numOutVerts++;
            }
        }

        return numOutVerts;
    }
}
