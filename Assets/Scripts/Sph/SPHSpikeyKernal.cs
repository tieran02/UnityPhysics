using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public struct SPHSpikeyKernal
{
    float h, h2, h3, h4, h5;

    public SPHSpikeyKernal(float kernalRadius)
    {
        h = kernalRadius;
        h2 = h * h;
        h3 = h2 * h;
        h4 = h3 * h;
        h5 = h4 * h;
    }

    public float Value(float distance)
    {
        if (distance >= h)
            return 0.0f;
        else
        {
            float x = 1.0f - distance / h;
            return 15.0f / (Mathf.PI * h3) * x * x * x;
        }
    }

    public float FirstDerivative(float distance)
    {
        if (distance >= h)
        {
            return 0.0f;
        }
        else
        {
            float x = 1.0f - distance  / h;
            return -45.0f / (Mathf.PI * h4)  * x * x;
        }
    }

    public Vector3 Gradiant(float distance, Vector3 direction)
    {
        return -FirstDerivative(distance) * direction;
    }

    public float SecondDerivative(float distance)
    {
        if (distance >= h)
            return 0.0f;
        else
        {
            float x = 1.0f - distance / h;
            return 90.0f / (Mathf.PI * h5) * x;
        }
    }


}
