using System;
using DotRecast.Core;
using DotRecast.Core.Numerics;

namespace DotRecast.Detour.Extras.Jumplink;

public class DtBallisticTrajectory : IDtTrajectory
{
    private readonly float verticalSpeed;
    private readonly float gravity;

    public DtBallisticTrajectory
    (
        float verticalSpeed,
        float gravity
    )
    {
        this.verticalSpeed = verticalSpeed;
        this.gravity       = gravity;
    }

    public RcVec3f Apply
    (
        RcVec3f start,
        RcVec3f end,
        float   u
    )
    {
        var t = u * TimeToReach(start, end);

        return new()
        {
            X = RcMath.Lerp(start.X, end.X, u),
            Y = start.Y + (verticalSpeed * t) - (0.5f * gravity * t * t),
            Z = RcMath.Lerp(start.Z, end.Z, u)
        };
    }

    public float TimeToReach
    (
        RcVec3f start,
        RcVec3f end
    )
    {
        var drop         = start.Y                         - end.Y;
        var discriminant = (verticalSpeed * verticalSpeed) + (2 * gravity * drop);

        return discriminant < 0 ?
                   -1 :
                   (verticalSpeed + MathF.Sqrt(discriminant)) / gravity;
    }
}
