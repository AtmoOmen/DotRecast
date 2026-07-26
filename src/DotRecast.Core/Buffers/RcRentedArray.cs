using System;
using System.Buffers;
using System.Runtime.CompilerServices;

namespace DotRecast.Core.Buffers;

public class RcRentedArray
{
    public static readonly RcRentedArray Shared = new();

    private RcRentedArray()
    {
    }

    public RcRentedArray<T> Rent<T>
    (
        int minimumLength
    ) =>
        new(minimumLength);

    public void Return<T>
    (
        RcRentedArray<T> array
    )
    {
        if (array.IsDisposed)
            return;

        array.Dispose();
    }
}

public ref struct RcRentedArray<T>
{
    private readonly T[] _items;
    public readonly  int Length;

    public bool IsDisposed { get; private set; }

    internal RcRentedArray
    (
        int length
    )
    {
        Length     = length;
        _items     = ArrayPool<T>.Shared.Rent(length);
        IsDisposed = false;
    }

    public void Dispose()
    {
        if (IsDisposed)
            return;

        IsDisposed = true;
        ArrayPool<T>.Shared.Return(_items, true);
    }

    public ref T this
    [
        int index
    ]
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        get
        {
            if (0 > index || Length <= index)
                RcThrowHelper.ThrowExceptionIfIndexOutOfRange(index, Length);

            if (IsDisposed)
                RcThrowHelper.ThrowNullReferenceException("already disposed");

            return ref _items[index];
        }
    }

    public Span<T> AsSpan()
    {
        if (IsDisposed)
            RcThrowHelper.ThrowNullReferenceException("already disposed");

        return new Span<T>(_items, 0, Length);
    }
}
