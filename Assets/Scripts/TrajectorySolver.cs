using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class TrajectorySolver : MonoBehaviour
{
    public Vector3 InitialVelocity;

    private Vector3 GRAVITY_FORCE = new Vector3(0.0f, -9.8f,0.0f);
    private Vector3 StartPosition;

    private Vector3 velocity;
    private Vector3 position;

    private const float TIME_STEP = 1 / 60.0f;
    private float accumulator;
    private float lastTime;

    // Start is called before the first frame update
    void Start()
    {
        StartPosition = transform.position;
        position = StartPosition;
        velocity = InitialVelocity;
    }

    // Update is called once per frame
    void Update()
    {
        accumulator += Time.deltaTime;

        //update 60 times per second
        while (accumulator > TIME_STEP)
        {
            float deltaTime = Time.time - lastTime;
            if (deltaTime <= 0.0f) deltaTime = TIME_STEP;
            lastTime = Time.time;

            Solver(deltaTime);

            accumulator -= TIME_STEP;
        }
    }

    void Solver(float deltaTime)
    {
        velocity += (GRAVITY_FORCE * deltaTime);
        position += (velocity * deltaTime);
        transform.position = position;
    }
}
