using System;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;

public static class Helpers
{
    public static Quaternion ScalarMultiply(this Quaternion input, float scalar)
    {
        return new Quaternion(input.x * scalar, input.y * scalar, input.z * scalar, input.w * scalar);
    }

    public static void Resize<T>(this List<T> list, int size, T element = default(T), bool clone = false)
    {
        int count = list.Count;

        if (size < count)
        {
            list.RemoveRange(size, count - size);
        }
        else if (size > count)
        {
            if (size > list.Capacity)   // Optimization
                list.Capacity = size;
            if(!clone)
                list.AddRange(Enumerable.Repeat(element, size - count));
            else
                list.AddRange(Enumerable.Range(0, size - count).Select(i => Activator.CreateInstance<T>()).ToArray());
        }
    }


}
