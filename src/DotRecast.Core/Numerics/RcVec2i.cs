using System;

namespace DotRecast.Core.Numerics;

public struct RcVec2i : IEquatable<RcVec2i>
{
    public int X;
    public int Y;

    public static RcVec2i Zero  => new(0, 0);
    public static RcVec2i UnitX => new(1, 0);
    public static RcVec2i UnitY => new(0, 1);

    // Comparison Operators
    public static bool operator ==
    (
        RcVec2i left,
        RcVec2i right
    ) => left.Equals(right);

    public static bool operator !=
    (
        RcVec2i left,
        RcVec2i right
    ) => !left.Equals(right);

    // Arithmetic Operators
    public static RcVec2i operator +
    (
        RcVec2i a,
        RcVec2i b
    ) => new(a.X + b.X, a.Y + b.Y);

    public static RcVec2i operator -
    (
        RcVec2i a,
        RcVec2i b
    ) => new(a.X - b.X, a.Y - b.Y);

    public static RcVec2i operator *
    (
        RcVec2i a,
        int     scalar
    ) => new(a.X * scalar, a.Y * scalar);

    public RcVec2i
    (
        int x,
        int y
    )
    {
        X = x;
        Y = y;
    }

    public int this
    [
        int index
    ] =>
        index switch
        {
            0 => X,
            1 => Y,
            _ => throw new IndexOutOfRangeException()
        };


    public bool Equals
    (
        RcVec2i other
    ) =>
        X == other.X && Y == other.Y;

    public override bool Equals
    (
        object obj
    ) =>
        obj is RcVec2i other && Equals(other);

    public override int GetHashCode() =>
        HashCode.Combine(X, Y);

    public override string ToString() =>
        $"({X}, {Y})";
}
