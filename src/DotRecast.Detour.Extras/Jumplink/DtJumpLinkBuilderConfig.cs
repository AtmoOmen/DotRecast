using System;

namespace DotRecast.Detour.Extras.Jumplink;

public class DtJumpLinkBuilderConfig
{
    public readonly float cellSize;
    public readonly float cellHeight;
    public readonly float agentClimb;
    public readonly float agentRadius;
    public readonly float groundTolerance;
    public readonly float agentHeight;
    public readonly float startDistance;
    public readonly float endDistance;
    public readonly float minHorizontalSpeed;
    public readonly float maxHorizontalSpeed;
    public readonly float verticalSpeed;
    public readonly float gravity;
    public readonly float minHeight;
    public readonly float heightRange;

    public DtJumpLinkBuilderConfig
    (
        float cellSize,
        float cellHeight,
        float agentRadius,
        float agentHeight,
        float agentClimb,
        float groundTolerance,
        float startDistance,
        float minHorizontalSpeed,
        float maxHorizontalSpeed,
        float verticalSpeed,
        float gravity,
        float minHeight,
        float maxHeight
    )
    {
        this.cellSize           = cellSize;
        this.cellHeight         = cellHeight;
        this.agentRadius        = agentRadius;
        this.agentHeight        = agentHeight;
        this.agentClimb         = agentClimb;
        this.groundTolerance    = groundTolerance;
        this.startDistance      = startDistance;
        this.minHorizontalSpeed = minHorizontalSpeed;
        this.maxHorizontalSpeed = maxHorizontalSpeed;
        this.verticalSpeed      = verticalSpeed;
        this.gravity            = gravity;
        this.minHeight          = minHeight;
        heightRange             = maxHeight - minHeight;
        endDistance             = maxHorizontalSpeed * TimeToFall(verticalSpeed, gravity, -minHeight);
    }

    private static float TimeToFall
    (
        float verticalSpeed,
        float gravity,
        float drop
    ) =>
        (verticalSpeed + MathF.Sqrt((verticalSpeed * verticalSpeed) + (2 * gravity * drop))) / gravity;
}
