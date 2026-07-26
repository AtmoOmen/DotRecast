using System;
using System.Threading;

namespace DotRecast.Core;

public class RcAtomicLong : IComparable<RcAtomicLong>
{
    private long _location;

    public RcAtomicLong() : this(0)
    {
    }

    public RcAtomicLong
    (
        long location
    ) =>
        _location = location;

    public int CompareTo
    (
        RcAtomicLong other
    ) =>
        Read().CompareTo(other.Read());

    public long IncrementAndGet() =>
        Interlocked.Increment(ref _location);

    public long DecrementAndGet() =>
        Interlocked.Decrement(ref _location);

    public long Read() =>
        Interlocked.Read(ref _location);

    public long Exchange
    (
        long exchange
    ) =>
        Interlocked.Exchange(ref _location, exchange);

    public long Decrease
    (
        long value
    ) =>
        Interlocked.Add(ref _location, -value);

    public long CompareExchange
    (
        long value,
        long comparand
    ) =>
        Interlocked.CompareExchange(ref _location, value, comparand);

    public long AddAndGet
    (
        long value
    ) =>
        Interlocked.Add(ref _location, value);
}
