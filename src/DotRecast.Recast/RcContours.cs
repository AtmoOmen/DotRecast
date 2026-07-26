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

public static class RcContours
{
    private static int GetCornerHeight
    (
        int                  x,
        int                  y,
        int                  i,
        int                  dir,
        RcCompactHeightfield chf,
        out bool             isBorderVertex
    )
    {
        isBorderVertex = false;

        ref var s    = ref chf.spans[i];
        var     ch   = s.y;
        var     dirp = (dir + 1) & 0x3;

        int[] regs =
        [
            0, 0, 0, 0
        ];

        // Combine region and area codes in order to prevent
        // border vertices which are in between two areas to be removed.
        regs[0] = chf.spans[i].reg | (chf.areas[i] << 16);

        if (GetCon(s, dir) != RC_NOT_CONNECTED)
        {
            var     ax  = x                                      + GetDirOffsetX(dir);
            var     ay  = y                                      + GetDirOffsetY(dir);
            var     ai  = chf.cells[ax + (ay * chf.width)].index + GetCon(s, dir);
            ref var @as = ref chf.spans[ai];
            ch      = Math.Max(ch, @as.y);
            regs[1] = chf.spans[ai].reg | (chf.areas[ai] << 16);

            if (GetCon(@as, dirp) != RC_NOT_CONNECTED)
            {
                var     ax2 = ax                                       + GetDirOffsetX(dirp);
                var     ay2 = ay                                       + GetDirOffsetY(dirp);
                var     ai2 = chf.cells[ax2 + (ay2 * chf.width)].index + GetCon(@as, dirp);
                ref var as2 = ref chf.spans[ai2];
                ch      = Math.Max(ch, as2.y);
                regs[2] = chf.spans[ai2].reg | (chf.areas[ai2] << 16);
            }
        }

        if (GetCon(s, dirp) != RC_NOT_CONNECTED)
        {
            var     ax  = x                                      + GetDirOffsetX(dirp);
            var     ay  = y                                      + GetDirOffsetY(dirp);
            var     ai  = chf.cells[ax + (ay * chf.width)].index + GetCon(s, dirp);
            ref var @as = ref chf.spans[ai];
            ch      = Math.Max(ch, @as.y);
            regs[3] = chf.spans[ai].reg | (chf.areas[ai] << 16);

            if (GetCon(@as, dir) != RC_NOT_CONNECTED)
            {
                var     ax2 = ax                                       + GetDirOffsetX(dir);
                var     ay2 = ay                                       + GetDirOffsetY(dir);
                var     ai2 = chf.cells[ax2 + (ay2 * chf.width)].index + GetCon(@as, dir);
                ref var as2 = ref chf.spans[ai2];
                ch      = Math.Max(ch, as2.y);
                regs[2] = chf.spans[ai2].reg | (chf.areas[ai2] << 16);
            }
        }

        // Check if the vertex is special edge vertex, these vertices will be removed later.
        for (var j = 0; j < 4; ++j)
        {
            var a = j;
            var b = (j + 1) & 0x3;
            var c = (j + 2) & 0x3;
            var d = (j + 3) & 0x3;

            // The vertex is a border vertex there are two same exterior cells in a row,
            // followed by two interior cells and none of the regions are out of bounds.
            var twoSameExts  = (regs[a] & regs[b] & RC_BORDER_REG) != 0 && regs[a] == regs[b];
            var twoInts      = ((regs[c] | regs[d]) & RC_BORDER_REG) == 0;
            var intsSameArea = regs[c] >> 16                         == regs[d] >> 16;
            var noZeros      = regs[a] != 0 && regs[b] != 0 && regs[c] != 0 && regs[d] != 0;

            if (twoSameExts && twoInts && intsSameArea && noZeros)
            {
                isBorderVertex = true;
                break;
            }
        }

        return ch;
    }

    private static void WalkContour
    (
        int                  x,
        int                  y,
        int                  i,
        RcCompactHeightfield chf,
        int[]                flags,
        List<int>            points
    )
    {
        // Choose the first non-connected edge
        var dir = 0;
        while ((flags[i] & (1 << dir)) == 0)
            dir++;

        var startDir = dir;
        var starti   = i;

        var area = chf.areas[i];

        var iter = 0;

        while (++iter < 40000)
        {
            if ((flags[i] & (1 << dir)) != 0)
            {
                // Choose the edge corner
                var isBorderVertex = false;
                var isAreaBorder   = false;
                var px             = x;
                var py             = GetCornerHeight(x, y, i, dir, chf, out isBorderVertex);
                var pz             = y;

                switch (dir)
                {
                    case 0:
                        pz++;
                        break;
                    case 1:
                        px++;
                        pz++;
                        break;
                    case 2:
                        px++;
                        break;
                }

                var     r = 0;
                ref var s = ref chf.spans[i];

                if (GetCon(s, dir) != RC_NOT_CONNECTED)
                {
                    var ax = x                                      + GetDirOffsetX(dir);
                    var ay = y                                      + GetDirOffsetY(dir);
                    var ai = chf.cells[ax + (ay * chf.width)].index + GetCon(s, dir);
                    r = chf.spans[ai].reg;
                    if (area != chf.areas[ai])
                        isAreaBorder = true;
                }

                if (isBorderVertex)
                    r |= RC_BORDER_VERTEX;
                if (isAreaBorder)
                    r |= RC_AREA_BORDER;
                points.Add(px);
                points.Add(py);
                points.Add(pz);
                points.Add(r);

                flags[i] &= ~(1 << dir);     // Remove visited edges
                dir      =  (dir + 1) & 0x3; // Rotate CW
            }
            else
            {
                var     ni = -1;
                var     nx = x + GetDirOffsetX(dir);
                var     ny = y + GetDirOffsetY(dir);
                ref var s  = ref chf.spans[i];

                if (GetCon(s, dir) != RC_NOT_CONNECTED)
                {
                    ref var nc = ref chf.cells[nx + (ny * chf.width)];
                    ni = nc.index + GetCon(s, dir);
                }

                if (ni == -1)
                {
                    // Should not happen.
                    return;
                }

                x   = nx;
                y   = ny;
                i   = ni;
                dir = (dir + 3) & 0x3; // Rotate CCW
            }

            if (starti == i && startDir == dir)
                break;
        }
    }

    private static float DistancePtSeg
    (
        int x,
        int y,
        int z,
        int px,
        int py,
        int pz,
        int qx,
        int qy,
        int qz
    )
    {
        float pqx = qx          - px;
        float pqy = qy          - py;
        float pqz = qz          - pz;
        float dx  = x           - px;
        float dy  = y           - py;
        float dz  = z           - pz;
        var   d   = (pqx * pqx) + (pqy * pqy) + (pqz * pqz);
        var   t   = (pqx * dx)  + (pqy * dy)  + (pqz * dz);
        if (d > 0)
            t /= d;
        if (t < 0)
            t = 0;
        else if (t > 1)
            t = 1;

        dx = px + (t * pqx) - x;
        dy = py + (t * pqy) - y;
        dz = pz + (t * pqz) - z;

        return (dx * dx) + (dy * dy) + (dz * dz);
    }

    private static void SimplifyContour
    (
        List<int> points,
        List<int> simplified,
        float     maxError,
        int       maxEdgeLen,
        int       buildFlags
    )
    {
        // Add initial points.
        var hasConnections = false;

        for (var i = 0; i < points.Count; i += 4)
            if ((points[i + 3] & RC_CONTOUR_REG_MASK) != 0)
            {
                hasConnections = true;
                break;
            }

        if (hasConnections)
        {
            // The contour has some portals to other regions.
            // Add a new point to every location where the region changes.
            for (int i = 0, ni = points.Count / 4; i < ni; ++i)
            {
                var ii            = (i + 1) % ni;
                var differentRegs = (points[(i * 4) + 3] & RC_CONTOUR_REG_MASK) != (points[(ii * 4) + 3] & RC_CONTOUR_REG_MASK);
                var areaBorders   = (points[(i * 4) + 3] & RC_AREA_BORDER)      != (points[(ii * 4) + 3] & RC_AREA_BORDER);

                if (differentRegs || areaBorders)
                {
                    simplified.Add(points[(i * 4) + 0]);
                    simplified.Add(points[(i * 4) + 1]);
                    simplified.Add(points[(i * 4) + 2]);
                    simplified.Add(i);
                }
            }
        }

        if (simplified.Count == 0)
        {
            // If there is no connections at all,
            // create some initial points for the simplification process.
            // Find lower-left and upper-right vertices of the contour.
            var llx = points[0];
            var lly = points[1];
            var llz = points[2];
            var lli = 0;
            var urx = points[0];
            var ury = points[1];
            var urz = points[2];
            var uri = 0;

            for (var i = 0; i < points.Count; i += 4)
            {
                var x = points[i + 0];
                var y = points[i + 1];
                var z = points[i + 2];

                if (x < llx || (x == llx && z < llz))
                {
                    llx = x;
                    lly = y;
                    llz = z;
                    lli = i / 4;
                }

                if (x > urx || (x == urx && z > urz))
                {
                    urx = x;
                    ury = y;
                    urz = z;
                    uri = i / 4;
                }
            }

            simplified.Add(llx);
            simplified.Add(lly);
            simplified.Add(llz);
            simplified.Add(lli);

            simplified.Add(urx);
            simplified.Add(ury);
            simplified.Add(urz);
            simplified.Add(uri);
        }

        // Add points until all raw points are within
        // error tolerance to the simplified shape.
        var pn = points.Count / 4;

        for (var i = 0; i < simplified.Count / 4;)
        {
            var ii = (i + 1) % (simplified.Count / 4);

            var ax = simplified[(i * 4) + 0];
            var ay = simplified[(i * 4) + 1];
            var az = simplified[(i * 4) + 2];
            var ai = simplified[(i * 4) + 3];

            var bx = simplified[(ii * 4) + 0];
            var by = simplified[(ii * 4) + 1];
            var bz = simplified[(ii * 4) + 2];
            var bi = simplified[(ii * 4) + 3];

            // Find maximum deviation from the segment.
            float maxd = 0;
            var   maxi = -1;
            int   ci, cinc, endi;

            // Traverse the segment in lexilogical order so that the
            // max deviation is calculated similarly when traversing
            // opposite segments.
            if (bx > ax || (bx == ax && bz > az))
            {
                cinc = 1;
                ci   = (ai + cinc) % pn;
                endi = bi;
            }
            else
            {
                cinc     = pn - 1;
                ci       = (bi + cinc) % pn;
                endi     = ai;
                (ax, bx) = (bx, ax);
                (ay, by) = (by, ay);
                (az, bz) = (bz, az);
            }

            // Tessellate only outer edges or edges between areas.
            //if ((points[ci * 4 + 3] & RC_CONTOUR_REG_MASK) == 0 || (points[ci * 4 + 3] & RC_AREA_BORDER) != 0)
            {
                while (ci != endi)
                {
                    var d = DistancePtSeg(points[(ci * 4) + 0], points[(ci * 4) + 1], points[(ci * 4) + 2], ax, ay, az, bx, by, bz);

                    if (d > maxd)
                    {
                        maxd = d;
                        maxi = ci;
                    }

                    ci = (ci + cinc) % pn;
                }
            }

            // If the max deviation is larger than accepted error,
            // add new point, else continue to next segment.
            if (maxi != -1 && maxd > maxError * maxError)
            {
                // Add the point.
                simplified.Insert(((i + 1) * 4) + 0, points[(maxi * 4) + 0]);
                simplified.Insert(((i + 1) * 4) + 1, points[(maxi * 4) + 1]);
                simplified.Insert(((i + 1) * 4) + 2, points[(maxi * 4) + 2]);
                simplified.Insert(((i + 1) * 4) + 3, maxi);
            }
            else
                ++i;
        }

        // Split too long edges.
        if (maxEdgeLen > 0 && (buildFlags & (RcBuildContoursFlags.RC_CONTOUR_TESS_WALL_EDGES | RcBuildContoursFlags.RC_CONTOUR_TESS_AREA_EDGES)) != 0)
        {
            for (var i = 0; i < simplified.Count / 4;)
            {
                var ii = (i + 1) % (simplified.Count / 4);

                var ax = simplified[(i * 4) + 0];
                var ay = simplified[(i * 4) + 1];
                var az = simplified[(i * 4) + 2];
                var ai = simplified[(i * 4) + 3];

                var bx = simplified[(ii * 4) + 0];
                var by = simplified[(ii * 4) + 1];
                var bz = simplified[(ii * 4) + 2];
                var bi = simplified[(ii * 4) + 3];

                // Find maximum deviation from the segment.
                var maxi = -1;
                var ci   = (ai + 1) % pn;

                // Tessellate only outer edges or edges between areas.
                var tess = false;
                // Wall edges.
                if ((buildFlags & RcBuildContoursFlags.RC_CONTOUR_TESS_WALL_EDGES) != 0 && (points[(ci * 4) + 3] & RC_CONTOUR_REG_MASK) == 0)
                    tess = true;

                // Edges between areas.
                if ((buildFlags & RcBuildContoursFlags.RC_CONTOUR_TESS_AREA_EDGES) != 0 && (points[(ci * 4) + 3] & RC_AREA_BORDER) != 0)
                    tess = true;

                if (tess)
                {
                    var dx = bx - ax;
                    var dy = by - ay;
                    var dz = bz - az;

                    if ((dx * dx) + (dy * dy) + (dz * dz) > maxEdgeLen * maxEdgeLen)
                    {
                        // Round based on the segments in lexilogical order so that the
                        // max tesselation is consistent regardless in which direction
                        // segments are traversed.
                        var n = bi < ai ?
                                    bi + pn - ai :
                                    bi      - ai;

                        if (n > 1)
                        {
                            if (bx > ax || (bx == ax && bz > az))
                                maxi = (ai + (n / 2)) % pn;
                            else
                                maxi = (ai + ((n + 1) / 2)) % pn;
                        }
                    }
                }

                // If the max deviation is larger than accepted error,
                // add new point, else continue to next segment.
                if (maxi != -1)
                {
                    // Add the point.
                    simplified.Insert(((i + 1) * 4) + 0, points[(maxi * 4) + 0]);
                    simplified.Insert(((i + 1) * 4) + 1, points[(maxi * 4) + 1]);
                    simplified.Insert(((i + 1) * 4) + 2, points[(maxi * 4) + 2]);
                    simplified.Insert(((i + 1) * 4) + 3, maxi);
                }
                else
                    ++i;
            }
        }

        for (var i = 0; i < simplified.Count / 4; ++i)
        {
            // The edge vertex flag is take from the current raw point,
            // and the neighbour region is take from the next raw point.
            var ai = (simplified[(i * 4) + 3] + 1) % pn;
            var bi = simplified[(i * 4) + 3];
            simplified[(i * 4) + 3] = (points[(ai * 4) + 3] & (RC_CONTOUR_REG_MASK | RC_AREA_BORDER)) | (points[(bi * 4) + 3] & RC_BORDER_VERTEX);
        }
    }

    private static int CalcAreaOfPolygon2D
    (
        int[] verts,
        int   nverts
    )
    {
        var area = 0;

        for (int i = 0, j = nverts - 1; i < nverts; j = i++)
        {
            var vi = i             * 4;
            var vj = j             * 4;
            area += (verts[vi + 0] * verts[vj + 2]) - (verts[vj + 0] * verts[vi + 2]);
        }

        return (area + 1) / 2;
    }

    private static bool IntersectSegContour
    (
        int   d0,
        int   d1,
        int   i,
        int   n,
        int[] verts,
        int[] d0verts,
        int[] d1verts
    )
    {
        // For each edge (k,k+1) of P
        var pverts = new int[4 * 4];

        for (var g = 0; g < 4; g++)
        {
            pverts[g] = d0verts[d0 + g];
            pverts[4               + g] = d1verts[d1 + g];
        }

        d0 = 0;
        d1 = 4;

        for (var k = 0; k < n; k++)
        {
            var k1 = RcMeshs.Next(k, n);
            // Skip edges incident to i.
            if (i == k || i == k1)
                continue;
            var p0 = k  * 4;
            var p1 = k1 * 4;

            for (var g = 0; g < 4; g++)
            {
                pverts[8  + g] = verts[p0 + g];
                pverts[12 + g] = verts[p1 + g];
            }

            p0 = 8;
            p1 = 12;
            if (RcMeshs.VEqual(pverts, d0, p0) ||
                RcMeshs.VEqual(pverts, d1, p0) ||
                RcMeshs.VEqual(pverts, d0, p1) ||
                RcMeshs.VEqual(pverts, d1, p1))
                continue;

            if (RcMeshs.Intersect(pverts, d0, d1, p0, p1))
                return true;
        }

        return false;
    }

    private static bool InCone
    (
        int   i,
        int   n,
        int[] verts,
        int   pj,
        int[] vertpj
    )
    {
        var pi     = i                  * 4;
        var pi1    = RcMeshs.Next(i, n) * 4;
        var pin1   = RcMeshs.Prev(i, n) * 4;
        var pverts = new int[4 * 4];

        for (var g = 0; g < 4; g++)
        {
            pverts[g] = verts[pi + g];
            pverts[4             + g] = verts[pi1  + g];
            pverts[8             + g] = verts[pin1 + g];
            pverts[12            + g] = vertpj[pj  + g];
        }

        pi   = 0;
        pi1  = 4;
        pin1 = 8;
        pj   = 12;
        // If P[i] is a convex vertex [ i+1 left or on (i-1,i) ].
        if (RcMeshs.LeftOn(pverts, pin1, pi, pi1))
            return RcMeshs.Left(pverts, pi, pj, pin1) && RcMeshs.Left(pverts, pj, pi, pi1);
        // Assume (i-1,i,i+1) not collinear.
        // else P[i] is reflex.
        return !(RcMeshs.LeftOn(pverts, pi, pj, pi1) && RcMeshs.LeftOn(pverts, pj, pi, pin1));
    }

    private static void RemoveDegenerateSegments
    (
        List<int> simplified
    )
    {
        // Remove adjacent vertices which are equal on xz-plane,
        // or else the triangulator will get confused.
        var npts = simplified.Count / 4;

        for (var i = 0; i < npts; ++i)
        {
            var ni = RcMeshs.Next(i, npts);

            // if (Vequal(&simplified[i*4], &simplified[ni*4]))
            if (simplified[i * 4] == simplified[ni * 4] && simplified[(i * 4) + 2] == simplified[(ni * 4) + 2])
            {
                // Degenerate segment, remove.
                simplified.RemoveAt(i * 4);
                simplified.RemoveAt(i * 4);
                simplified.RemoveAt(i * 4);
                simplified.RemoveAt(i * 4);
                npts--;
            }
        }
    }

    private static void MergeContours
    (
        RcContour ca,
        RcContour cb,
        int       ia,
        int       ib
    )
    {
        var maxVerts = ca.nverts + cb.nverts + 2;
        var verts    = new int[maxVerts * 4];

        var nv = 0;

        // Copy contour A.
        for (var i = 0; i <= ca.nverts; ++i)
        {
            var dst = nv                   * 4;
            var src = (ia + i) % ca.nverts * 4;
            verts[dst + 0] = ca.verts[src + 0];
            verts[dst + 1] = ca.verts[src + 1];
            verts[dst + 2] = ca.verts[src + 2];
            verts[dst + 3] = ca.verts[src + 3];
            nv++;
        }

        // Copy contour B
        for (var i = 0; i <= cb.nverts; ++i)
        {
            var dst = nv                   * 4;
            var src = (ib + i) % cb.nverts * 4;
            verts[dst + 0] = cb.verts[src + 0];
            verts[dst + 1] = cb.verts[src + 1];
            verts[dst + 2] = cb.verts[src + 2];
            verts[dst + 3] = cb.verts[src + 3];
            nv++;
        }

        ca.verts  = verts;
        ca.nverts = nv;

        cb.verts  = null;
        cb.nverts = 0;
    }

    // Finds the lowest leftmost vertex of a contour.
    private static int[] FindLeftMostVertex
    (
        RcContour contour
    )
    {
        var minx     = contour.verts[0];
        var minz     = contour.verts[2];
        var leftmost = 0;

        for (var i = 1; i < contour.nverts; i++)
        {
            var x = contour.verts[(i * 4) + 0];
            var z = contour.verts[(i * 4) + 2];

            if (x < minx || (x == minx && z < minz))
            {
                minx     = x;
                minz     = z;
                leftmost = i;
            }
        }

        return [minx, minz, leftmost];
    }

    private static void MergeRegionHoles
    (
        RcContext       ctx,
        RcContourRegion region
    )
    {
        // Sort holes from left to right.
        for (var i = 0; i < region.nholes; i++)
        {
            var minleft = FindLeftMostVertex(region.holes[i].contour);
            region.holes[i].minx     = minleft[0];
            region.holes[i].minz     = minleft[1];
            region.holes[i].leftmost = minleft[2];
        }

        Array.Sort(region.holes, RcContourHoleComparer.Shared);

        var maxVerts = region.outline.nverts;
        for (var i = 0; i < region.nholes; i++)
            maxVerts += region.holes[i].contour.nverts;

        var diags = new RcPotentialDiagonal[maxVerts];

        var outline = region.outline;

        // Merge holes into the outline one by one.
        for (var i = 0; i < region.nholes; i++)
        {
            var hole = region.holes[i].contour;

            var index      = -1;
            var bestVertex = region.holes[i].leftmost;

            for (var iter = 0; iter < hole.nverts; iter++)
            {
                // Find potential diagonals.
                // The 'best' vertex must be in the cone described by 3 consecutive vertices of the outline.
                // ..o j-1
                // |
                // | * best
                // |
                // j o-----o j+1
                // :
                var ndiags = 0;
                var corner = bestVertex * 4;

                for (var j = 0; j < outline.nverts; j++)
                    if (InCone(j, outline.nverts, outline.verts, corner, hole.verts))
                    {
                        var dx = outline.verts[(j * 4) + 0] - hole.verts[corner + 0];
                        var dz = outline.verts[(j * 4) + 2] - hole.verts[corner + 2];
                        diags[ndiags] = new RcPotentialDiagonal(j, (dx * dx) + (dz * dz));
                        ndiags++;
                    }

                // Sort potential diagonals by distance, we want to make the connection as short as possible.
                Array.Sort(diags, 0, ndiags, RcPotentialDiagonalComparer.Shared);

                // Find a diagonal that is not intersecting the outline not the remaining holes.
                index = -1;

                for (var j = 0; j < ndiags; j++)
                {
                    var pt = diags[j].vert * 4;
                    var intersect = IntersectSegContour
                    (
                        pt,
                        corner,
                        diags[j].vert,
                        outline.nverts,
                        outline.verts,
                        outline.verts,
                        hole.verts
                    );
                    for (var k = i; k < region.nholes && !intersect; k++)
                        intersect |= IntersectSegContour
                        (
                            pt,
                            corner,
                            -1,
                            region.holes[k].contour.nverts,
                            region.holes[k].contour.verts,
                            outline.verts,
                            hole.verts
                        );

                    if (!intersect)
                    {
                        index = diags[j].vert;
                        break;
                    }
                }

                // If found non-intersecting diagonal, stop looking.
                if (index != -1)
                    break;
                // All the potential diagonals for the current vertex were intersecting, try next vertex.
                bestVertex = (bestVertex + 1) % hole.nverts;
            }

            if (index == -1)
            {
                ctx.Warn("mergeHoles: Failed to find merge points for");
                continue;
            }

            MergeContours(region.outline, hole, index, bestVertex);
        }
    }

    /// @par
    /// 
    /// The raw contours will match the region outlines exactly. The @p maxError and @p maxEdgeLen
    /// parameters control how closely the simplified contours will match the raw contours.
    /// 
    /// Simplified contours are generated such that the vertices for portals between areas match up.
    /// (They are considered mandatory vertices.)
    /// 
    /// Setting @p maxEdgeLength to zero will disabled the edge length feature.
    /// 
    /// See the #rcConfig documentation for more information on the configuration parameters.
    /// 
    /// @see rcAllocContourSet, rcCompactHeightfield, rcContourSet, rcConfig
    public static RcContourSet BuildContours
    (
        RcContext            ctx,
        RcCompactHeightfield chf,
        float                maxError,
        int                  maxEdgeLen,
        int                  buildFlags
    )
    {
        var w          = chf.width;
        var h          = chf.height;
        var borderSize = chf.borderSize;
        var cset       = new RcContourSet();

        using var timer = ctx.ScopedTimer(RcTimerLabel.RC_TIMER_BUILD_CONTOURS);

        cset.bmin = chf.bmin;
        cset.bmax = chf.bmax;

        if (borderSize > 0)
        {
            // If the heightfield was build with bordersize, remove the offset.
            var pad = borderSize * chf.cs;
            cset.bmin.X += pad;
            cset.bmin.Z += pad;
            cset.bmax.X -= pad;
            cset.bmax.Z -= pad;
        }

        cset.cs         = chf.cs;
        cset.ch         = chf.ch;
        cset.width      = chf.width  - (chf.borderSize * 2);
        cset.height     = chf.height - (chf.borderSize * 2);
        cset.borderSize = chf.borderSize;
        cset.maxError   = maxError;

        var flags = new int[chf.spanCount];

        ctx.StartTimer(RcTimerLabel.RC_TIMER_BUILD_CONTOURS_TRACE);

        // Mark boundaries.
        for (var y = 0; y < h; ++y)
        for (var x = 0; x < w; ++x)
        {
            ref var c = ref chf.cells[x + (y * w)];

            for (int i = c.index, ni = c.index + c.count; i < ni; ++i)
            {
                var     res = 0;
                ref var s   = ref chf.spans[i];

                if (chf.spans[i].reg == 0 || (chf.spans[i].reg & RC_BORDER_REG) != 0)
                {
                    flags[i] = 0;
                    continue;
                }

                for (var dir = 0; dir < 4; ++dir)
                {
                    var r = 0;

                    if (GetCon(s, dir) != RC_NOT_CONNECTED)
                    {
                        var ax = x                              + GetDirOffsetX(dir);
                        var ay = y                              + GetDirOffsetY(dir);
                        var ai = chf.cells[ax + (ay * w)].index + GetCon(s, dir);
                        r = chf.spans[ai].reg;
                    }

                    if (r == chf.spans[i].reg)
                        res |= 1 << dir;
                }

                flags[i] = res ^ 0xf; // Inverse, mark non connected edges.
            }
        }

        ctx.StopTimer(RcTimerLabel.RC_TIMER_BUILD_CONTOURS_TRACE);

        List<int> verts      = [with(256)];
        List<int> simplified = [with(64)];

        for (var y = 0; y < h; ++y)
        for (var x = 0; x < w; ++x)
        {
            ref var c = ref chf.cells[x + (y * w)];

            for (int i = c.index, ni = c.index + c.count; i < ni; ++i)
            {
                if (flags[i] == 0 || flags[i] == 0xf)
                {
                    flags[i] = 0;
                    continue;
                }

                var reg = chf.spans[i].reg;
                if (reg == 0 || (reg & RC_BORDER_REG) != 0)
                    continue;
                var area = chf.areas[i];

                verts.Clear();
                simplified.Clear();

                ctx.StartTimer(RcTimerLabel.RC_TIMER_BUILD_CONTOURS_WALK);
                WalkContour(x, y, i, chf, flags, verts);
                ctx.StopTimer(RcTimerLabel.RC_TIMER_BUILD_CONTOURS_WALK);

                ctx.StartTimer(RcTimerLabel.RC_TIMER_BUILD_CONTOURS_SIMPLIFY);
                SimplifyContour(verts, simplified, maxError, maxEdgeLen, buildFlags);
                RemoveDegenerateSegments(simplified);
                ctx.StopTimer(RcTimerLabel.RC_TIMER_BUILD_CONTOURS_SIMPLIFY);

                // Store region->contour remap info.
                // Create contour.
                if (simplified.Count / 4 >= 3)
                {
                    var cont = new RcContour();
                    cset.conts.Add(cont);

                    cont.nverts = simplified.Count / 4;
                    cont.verts  = new int[simplified.Count];
                    for (var l = 0; l < cont.verts.Length; l++)
                        cont.verts[l] = simplified[l];

                    if (borderSize > 0)
                    {
                        // If the heightfield was build with bordersize, remove the offset.
                        for (var j = 0; j < cont.nverts; ++j)
                        {
                            cont.verts[j * 4]       -= borderSize;
                            cont.verts[(j * 4) + 2] -= borderSize;
                        }
                    }

                    cont.nrverts = verts.Count / 4;
                    cont.rverts  = new int[verts.Count];
                    for (var l = 0; l < cont.rverts.Length; l++)
                        cont.rverts[l] = verts[l];

                    if (borderSize > 0)
                    {
                        // If the heightfield was build with bordersize, remove the offset.
                        for (var j = 0; j < cont.nrverts; ++j)
                        {
                            cont.rverts[j * 4]       -= borderSize;
                            cont.rverts[(j * 4) + 2] -= borderSize;
                        }
                    }

                    cont.reg  = reg;
                    cont.area = area;
                }
            }
        }

        // Merge holes if needed.
        if (cset.conts.Count > 0)
        {
            // Calculate winding of all polygons.
            var winding = new int[cset.conts.Count];
            var nholes  = 0;

            for (var i = 0; i < cset.conts.Count; ++i)
            {
                var cont = cset.conts[i];
                // If the contour is wound backwards, it is a hole.
                winding[i] = CalcAreaOfPolygon2D(cont.verts, cont.nverts) < 0 ?
                                 -1 :
                                 1;
                if (winding[i] < 0)
                    nholes++;
            }

            if (nholes > 0)
            {
                // Collect outline contour and holes contours per region.
                // We assume that there is one outline and multiple holes.
                var nregions = chf.maxRegions + 1;
                var regions  = new RcContourRegion[nregions];
                for (var i = 0; i < nregions; i++)
                    regions[i] = new RcContourRegion();

                for (var i = 0; i < cset.conts.Count; ++i)
                {
                    var cont = cset.conts[i];

                    // Positively would contours are outlines, negative holes.
                    if (winding[i] > 0)
                    {
                        if (regions[cont.reg].outline != null)
                        {
                            throw new Exception
                            (
                                "rcBuildContours: Multiple outlines for region " + cont.reg + "."
                            );
                        }

                        regions[cont.reg].outline = cont;
                    }
                    else
                        regions[cont.reg].nholes++;
                }

                for (var i = 0; i < nregions; i++)
                    if (regions[i].nholes > 0)
                    {
                        regions[i].holes = new RcContourHole[regions[i].nholes];
                        for (var nh = 0; nh < regions[i].nholes; nh++)
                            regions[i].holes[nh] = new RcContourHole();

                        regions[i].nholes = 0;
                    }

                for (var i = 0; i < cset.conts.Count; ++i)
                {
                    var cont = cset.conts[i];
                    var reg  = regions[cont.reg];
                    if (winding[i] < 0)
                        reg.holes[reg.nholes++].contour = cont;
                }

                // Finally merge each regions holes into the outline.
                for (var i = 0; i < nregions; i++)
                {
                    var reg = regions[i];
                    if (reg.nholes == 0)
                        continue;

                    if (reg.outline != null)
                        MergeRegionHoles(ctx, reg);
                    else
                    {
                        // The region does not have an outline.
                        // This can happen if the contour becaomes selfoverlapping because of
                        // too aggressive simplification settings.
                        throw new Exception("rcBuildContours: Bad outline for region " + i + ", contour simplification is likely too aggressive.");
                    }
                }
            }
        }

        return cset;
    }
}
