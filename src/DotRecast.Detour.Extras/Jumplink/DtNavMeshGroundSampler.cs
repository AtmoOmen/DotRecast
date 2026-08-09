using DotRecast.Core;
using DotRecast.Core.Numerics;
using DotRecast.Recast;

namespace DotRecast.Detour.Extras.Jumplink;

public class DtNavMeshGroundSampler : DtAbstractGroundSampler
{
    public override void Sample
    (
        DtJumpLinkBuilderConfig acfg,
        RcBuilderResult         result,
        DtEdgeSampler           es
    )
    {
        var navMeshQuery = CreateQuery(result, acfg.agentRadius, acfg.agentHeight, acfg.agentClimb);
        Sample(acfg, es, navMeshQuery);
    }

    public void Sample
    (
        DtJumpLinkBuilderConfig acfg,
        DtEdgeSampler           es,
        DtNavMeshQuery          navMeshQuery
    )
    {
        if (navMeshQuery == null)
            return;

        SampleGround(acfg, es, (pt, heightRange, out height) => GetNavMeshHeight(navMeshQuery, pt, acfg.cellSize, heightRange, out height));
    }

    public DtNavMeshQuery CreateQuery
    (
        RcBuilderResult r,
        float           agentRadius,
        float           agentHeight,
        float           agentClimb
    )
    {
        var option = new DtNavMeshCreateParams();
        option.verts            = r.Mesh.verts;
        option.vertCount        = r.Mesh.nverts;
        option.polys            = r.Mesh.polys;
        option.polyAreas        = r.Mesh.areas;
        option.polyFlags        = r.Mesh.flags;
        option.polyCount        = r.Mesh.npolys;
        option.nvp              = r.Mesh.nvp;
        option.detailMeshes     = r.MeshDetail.meshes;
        option.detailVerts      = r.MeshDetail.verts;
        option.detailVertsCount = r.MeshDetail.nverts;
        option.detailTris       = r.MeshDetail.tris;
        option.detailTriCount   = r.MeshDetail.ntris;
        option.walkableRadius   = agentRadius;
        option.walkableHeight   = agentHeight;
        option.walkableClimb    = agentClimb;
        option.bmin             = r.Mesh.bmin;
        option.bmax             = r.Mesh.bmax;
        option.cs               = r.Mesh.cs;
        option.ch               = r.Mesh.ch;
        option.buildBvTree      = true;
        var mesh   = new DtNavMesh();
        var status = mesh.Init(DtNavMeshBuilder.CreateNavMeshData(option), option.nvp, 0);
        if (status.Failed())
            return null;

        return new DtNavMeshQuery(mesh);
    }


    private bool GetNavMeshHeight
    (
        DtNavMeshQuery navMeshQuery,
        RcVec3f        pt,
        float          cs,
        float          heightRange,
        out float      height
    )
    {
        height = 0;

        var halfExtents = new RcVec3f(cs, heightRange, cs);
        var maxHeight   = pt.Y + heightRange;
        var found       = new RcAtomicBoolean();
        var minHeight   = new RcAtomicFloat(pt.Y);

        void UpdateMinHeight
        (
            DtMeshTile tile,
            DtPoly     poly,
            long       refs
        )
        {
            var status = navMeshQuery.GetPolyHeight(refs, pt, out var h);

            if (status.Succeeded())
            {
                if (h > minHeight.Get() && h < maxHeight)
                {
                    minHeight.Exchange(h);
                    found.Set(true);
                }
            }
        }

        var query = new DtCallbackPolyQuery(UpdateMinHeight);

        navMeshQuery.QueryPolygons(pt, halfExtents, DtQueryNoOpFilter.Shared, query);

        if (found.Get())
        {
            height = minHeight.Get();
            return true;
        }

        height = pt.Y;
        return false;
    }
}
