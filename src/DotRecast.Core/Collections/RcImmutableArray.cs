namespace DotRecast.Core.Collections;

public static class RcImmutableArray
{
    public static RcImmutableArray<T> Create<T>() =>
        RcImmutableArray<T>.Empty;

    public static RcImmutableArray<T> Create<T>
    (
        T item1
    )
    {
        T[] array = [item1];
        return [with(array)];
    }

    public static RcImmutableArray<T> Create<T>
    (
        T item1,
        T item2
    )
    {
        T[] array = [item1, item2];
        return [with(array)];
    }

    public static RcImmutableArray<T> Create<T>
    (
        T item1,
        T item2,
        T item3
    )
    {
        T[] array = [item1, item2, item3];
        return [with(array)];
    }

    public static RcImmutableArray<T> Create<T>
    (
        T item1,
        T item2,
        T item3,
        T item4
    )
    {
        T[] array = [item1, item2, item3, item4];
        return [with(array)];
    }

    public static RcImmutableArray<T> Create<T>
    (
        params T[] items
    )
    {
        if (items == null || items.Length == 0)
            return RcImmutableArray<T>.Empty;

        var tmp = new T[items.Length];
        RcArrays.Copy(items, tmp, items.Length);
        return [with(tmp)];
    }
}
