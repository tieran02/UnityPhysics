using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public struct SPHKernal
{
    float h, h2, h3;

    public SPHKernal(float kernalRadius)
    {
        h = kernalRadius;
        h2 = h * h;
        h3 = h2 * h;
    }

    public float Value(float distance)
    {
        if (distance * distance >= h2)
            return 0.0f;
        else
        {
            float x = 1.0f - distance * distance / h2;
            return 315.0f / (64.0f * Mathf.PI * h3) * x * x * x;
        }
    }
}
