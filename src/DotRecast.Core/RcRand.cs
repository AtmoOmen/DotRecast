using System;

namespace DotRecast.Core;

public class RcRand : IRcRand
{
    private readonly Random _r;

    public RcRand() : this(new Random())
    {
    }

    public RcRand
    (
        Random r
    ) =>
        _r = r;

    public RcRand
    (
        long seed
    ) =>
        _r = new Random((int)seed); // TODO: @ikpil, check random seed value

    public float Next() =>
        (float)_r.NextDouble();

    public double NextDouble() =>
        _r.NextDouble();

    public int NextInt32() =>
        _r.Next();
}
