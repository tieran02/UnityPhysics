using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public struct SPHKernal
{
    float h, h2, h3, h5;

    public SPHKernal(float kernalRadius)
    {
        h = kernalRadius;
        h2 = h * h;
        h3 = h2 * h;
        h5 = h2 * h3;
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

    public float FirstDerivative(float distance)
    {
        if(distance >= h)
        {
            return 0.0f;
        }
        else
        {
            float x = 1.0f - distance * distance / h2;
            return -945.0f / (32.0f * Mathf.PI * h5) * distance * x * x;
        }
    }

    public Vector3 Gradiant(float distance, Vector3 direction)
    {
        return -FirstDerivative(distance) * direction;
    }

    public float SecondDerivative(float distance)
    {
        if (distance * distance >= h2)
            return 0.0f;
        else
        {
            float x = distance * distance / h2;
            return 945.0f / (32.0f * Mathf.PI * h5) * (1 - x) * (5 * x - 1);
        }
    }


}
