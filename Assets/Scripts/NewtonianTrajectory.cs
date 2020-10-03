using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class NewtonianTrajectory : MonoBehaviour
{
    public Vector3 InitialVelocity;

    private Vector3 StartPosition;

    private const float GRAVITY = -9.8f;
    private const float TIME_STEP = 1 / 60.0f;
    private float nextUpdateTime = 0.0f;



    // Start is called before the first frame update
    void Start()
    {
        StartPosition = transform.position;
    }

    // Update is called once per frame
    void Update()
    {
        // If the next update is reached
        if (Time.time >= nextUpdateTime)
        {
            transform.position = calculatePosition(nextUpdateTime);

            nextUpdateTime += TIME_STEP; 
        }
    }

    Vector3 calculatePosition(float t)
    {
        float xt = StartPosition.x + (InitialVelocity.x * t);
        float yt = StartPosition.y + (InitialVelocity.y * t) + (GRAVITY * (t * t)) / 2.0f;
        float zt = StartPosition.z + (InitialVelocity.z * t);

        return new Vector3(xt,yt,zt);
    }
}
