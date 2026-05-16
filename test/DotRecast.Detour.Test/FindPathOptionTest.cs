using System;
using DotRecast.Core.Collections;
using DotRecast.Core.Numerics;
using DotRecast.Recast;
using DotRecast.Recast.Geom;
using NUnit.Framework;

namespace DotRecast.Detour.Test;

public class FindPathOptionTest : AbstractDetourTest
{
    [Test]
    public void FindPathNoOptionShouldMatchLegacyOverload()
    {
        IDtQueryFilter filter = new DtQueryDefaultFilter();
        RcFixedArray256<long> legacyPath = new RcFixedArray256<long>();
        RcFixedArray256<long> optionPath = new RcFixedArray256<long>();

        var legacyStatus = query.FindPath(startRefs[0], endRefs[0], startPoss[0], endPoss[0], filter, legacyPath.AsSpan(), out var legacyCount, legacyPath.Length);
        var optionStatus = query.FindPath
        (
            startRefs[0],
            endRefs[0],
            startPoss[0],
            endPoss[0],
            filter,
            optionPath.AsSpan(),
            out var optionCount,
            optionPath.Length,
            DtFindPathOption.NoOption
        );

        Assert.That(optionStatus, Is.EqualTo(legacyStatus));
        Assert.That(optionCount, Is.EqualTo(legacyCount));

        for (var i = 0; i < legacyCount; ++i)
            Assert.That(optionPath[i], Is.EqualTo(legacyPath[i]));
    }

    [Test]
    public void FindPathZeroScaleShouldAllowFixedCostOffMeshShortcut()
    {
        using var fixture = OffMeshShortcutFixture.Create();

        RcFixedArray256<long> path = new RcFixedArray256<long>();
        var status = fixture.Query.FindPath
        (
            fixture.StartRef,
            fixture.EndRef,
            fixture.StartPos,
            fixture.EndPos,
            fixture.Filter,
            path.AsSpan(),
            out var pathCount,
            path.Length,
            DtFindPathOption.ZeroScale
        );

        Assert.That(status.Succeeded(), Is.True);
        Assert.That(pathCount, Is.GreaterThanOrEqualTo(3));
        Assert.That(ContainsOffMeshPoly(fixture.Mesh, path, pathCount), Is.True, "ZeroScale 启发式应允许固定代价离网连接成为最优路径");
    }

    private static bool ContainsOffMeshPoly(DtNavMesh navMesh, RcFixedArray256<long> path, int pathCount)
    {
        for (var i = 0; i < pathCount; ++i)
        {
            var polyRef = path[i];
            navMesh.GetTileAndPolyByRefUnsafe(polyRef, out _, out var poly);
            if (poly.GetPolyType() == DtPolyTypes.DT_POLYTYPE_OFFMESH_CONNECTION)
                return true;
        }

        return false;
    }

    private sealed class OffMeshShortcutFixture : IDisposable
    {
        public required DtNavMesh      Mesh     { get; init; }
        public required DtNavMeshQuery Query    { get; init; }
        public required IDtQueryFilter Filter   { get; init; }
        public required long           StartRef { get; init; }
        public required long           EndRef   { get; init; }
        public required RcVec3f        StartPos { get; init; }
        public required RcVec3f        EndPos   { get; init; }

        public void Dispose() => Query.GetAttachedNavMesh().Release();

        public static OffMeshShortcutFixture Create()
        {
            var startPos = new RcVec3f(22.606520f, 10.197294f, -45.918674f);
            var endPos   = new RcVec3f(6.457663f, 10.197294f, -18.334061f);
            var geom     = RcSampleInputGeomProvider.LoadFile("dungeon.obj");

            var cfg = new RcConfig
            (
                RcPartition.WATERSHED,
                0.3f,
                0.2f,
                45.0f,
                2.0f,
                0.6f,
                0.9f,
                8,
                20,
                12.0f,
                1.3f,
                6,
                6.0f,
                1.0f,
                true,
                true,
                true,
                SampleAreaModifications.SAMPLE_AREAMOD_GROUND,
                true
            );
            var builderConfig = new RcBuilderConfig(cfg, geom.GetMeshBoundsMin(), geom.GetMeshBoundsMax());
            var builder       = new RcBuilder();
            var result        = builder.Build(geom, builderConfig, false);
            var pmesh         = result.Mesh;
            var dmesh         = result.MeshDetail;

            for (var i = 0; i < pmesh.npolys; ++i)
                pmesh.flags[i] = 1;

            var option = new DtNavMeshCreateParams
            {
                verts            = pmesh.verts,
                vertCount        = pmesh.nverts,
                polys            = pmesh.polys,
                polyAreas        = pmesh.areas,
                polyFlags        = pmesh.flags,
                polyCount        = pmesh.npolys,
                nvp              = pmesh.nvp,
                detailMeshes     = dmesh.meshes,
                detailVerts      = dmesh.verts,
                detailVertsCount = dmesh.nverts,
                detailTris       = dmesh.tris,
                detailTriCount   = dmesh.ntris,
                walkableHeight   = 2.0f,
                walkableRadius   = 0.6f,
                walkableClimb    = 0.9f,
                bmin             = pmesh.bmin,
                bmax             = pmesh.bmax,
                cs               = 0.3f,
                ch               = 0.2f,
                buildBvTree      = true,
                offMeshConVerts  = [startPos.X, startPos.Y, startPos.Z, endPos.X, endPos.Y, endPos.Z],
                offMeshConRad    = [0.6f],
                offMeshConDir    = [0],
                offMeshConAreas  = [1],
                offMeshConFlags  = [2],
                offMeshConUserID = [0x1234],
                offMeshConCount  = 1
            };

            var meshData = DtNavMeshBuilder.CreateNavMeshData(option);
            Assert.That(meshData, Is.Not.Null);

            var navMesh = new DtNavMesh();
            navMesh.Init(new DtNavMeshParams
            {
                orig       = option.bmin,
                tileWidth  = option.bmax.X - option.bmin.X,
                tileHeight = option.bmax.Z - option.bmin.Z,
                maxTiles   = 1,
                maxPolys   = 1 << 14
            }, 6);
            navMesh.AddTile(meshData!, 0, 0, out _);

            var query = new DtNavMeshQuery(navMesh);
            query.FindNearestPoly(startPos, new RcVec3f(2, 4, 2), new DtQueryDefaultFilter(3, 0, [1f]), out var startRef, out _, out _);
            query.FindNearestPoly(endPos,   new RcVec3f(2, 4, 2), new DtQueryDefaultFilter(3, 0, [1f]), out var endRef,   out _, out _);

            Assert.That(startRef, Is.Not.EqualTo(0));
            Assert.That(endRef,   Is.Not.EqualTo(0));

            return new OffMeshShortcutFixture
            {
                Mesh     = navMesh,
                Query    = query,
                Filter   = new FixedCostOffMeshFilter(),
                StartRef = startRef,
                EndRef   = endRef,
                StartPos = startPos,
                EndPos   = endPos
            };
        }
    }

    private sealed class FixedCostOffMeshFilter : IDtQueryFilter
    {
        private readonly DtQueryDefaultFilter inner = new DtQueryDefaultFilter(3, 0, [1f, 1f]);

        public bool PassFilter(long refs, DtMeshTile tile, DtPoly poly) => inner.PassFilter(refs, tile, poly);

        public float GetCost
        (
            RcVec3f    pa,
            RcVec3f    pb,
            long       prevRef,
            DtMeshTile prevTile,
            DtPoly     prevPoly,
            long       curRef,
            DtMeshTile curTile,
            DtPoly     curPoly,
            long       nextRef,
            DtMeshTile nextTile,
            DtPoly     nextPoly
        )
            => curPoly.GetPolyType() == DtPolyTypes.DT_POLYTYPE_OFFMESH_CONNECTION ? 1f : RcVec3f.Distance(pa, pb);
    }
}
