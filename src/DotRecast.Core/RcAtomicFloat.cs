using System.Threading;

namespace DotRecast.Core;

public class RcAtomicFloat
{
    private volatile float _location;

    public RcAtomicFloat
    (
        float location
    ) =>
        _location = location;

    public float Get() =>
        _location;

    public float Exchange
    (
        float exchange
    ) =>
        Interlocked.Exchange(ref _location, exchange);

    public float CompareExchange
    (
        float value,
        float comparand
    ) =>
        Interlocked.CompareExchange(ref _location, value, comparand);
}
