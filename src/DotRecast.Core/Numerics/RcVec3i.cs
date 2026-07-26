using System;

namespace DotRecast.Core.Numerics;

public struct RcVec3i : IEquatable<RcVec3i>
{
    public int X;
    public int Y;
    public int Z;

    public static RcVec3i Zero  => new(0, 0, 0);
    public static RcVec3i UnitX => new(1, 0, 0);
    public static RcVec3i UnitY => new(0, 1, 0);
    public static RcVec3i UnitZ => new(0, 1, 1);

    // Comparison Operators
    public static bool operator ==
    (
        RcVec3i left,
        RcVec3i right
    ) => left.Equals(right);

    public static bool operator !=
    (
        RcVec3i left,
        RcVec3i right
    ) => !left.Equals(right);

    // Arithmetic Operators
    public static RcVec3i operator +
    (
        RcVec3i a,
        RcVec3i b
    ) => new(a.X + b.X, a.Y + b.Y, a.Z + b.Z);

    public static RcVec3i operator -
    (
        RcVec3i a,
        RcVec3i b
    ) => new(a.X - b.X, a.Y - b.Y, a.Z - b.Z);

    public static RcVec3i operator *
    (
        RcVec3i a,
        int     scalar
    ) => new(a.X * scalar, a.Y * scalar, a.Z * scalar);

    public RcVec3i
    (
        int x,
        int y,
        int z
    )
    {
        X = x;
        Y = y;
        Z = z;
    }

    public int this
    [
        int index
    ] =>
        index switch
        {
            0 => X,
            1 => Y,
            2 => Z,
            _ => throw new IndexOutOfRangeException()
        };


    public bool Equals
    (
        RcVec3i other
    ) =>
        X == other.X && Y == other.Y && Z == other.Z;

    public override bool Equals
    (
        object obj
    ) =>
        obj is RcVec3i other && Equals(other);

    public override int GetHashCode() =>
        HashCode.Combine(X, Y, Z);

    public override string ToString() =>
        $"({X}, {Y}, {Z})";
}
