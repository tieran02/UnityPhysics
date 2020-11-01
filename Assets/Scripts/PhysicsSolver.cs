using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using UnityEngine;

public class PhysicsSolver : MonoBehaviour
{
    public PhysicsRigidBody[] RigidBodies;
    public BaseCollider[] BaseColliders;
    public PlaneCollider PlaneCollider;

    private const float TIME_STEP = 1 / 60.0f;
    private float accumulator;
    private float lastTime;

    void Awake()
    {
        RigidBodies = FindObjectsOfType<PhysicsRigidBody>();
        BaseColliders = FindObjectsOfType<BaseCollider>();
    }

    // Update is called once per frame
    void Update()
    {
        accumulator += Time.deltaTime;

        //update 60 times per second (this limits the physic time step)
        //to make the movement look smoother we can always use linear interpolation
        while (accumulator > TIME_STEP)
        {
            float deltaTime = Time.time - lastTime;
            if (deltaTime <= 0.0f) deltaTime = TIME_STEP;
            lastTime = Time.time;

            foreach (var rigidBody in RigidBodies)
            {
                Solver(rigidBody, deltaTime);
            }
            

            accumulator -= TIME_STEP;
        }
    }

    void Solver(PhysicsRigidBody rigidBody, float deltaTime)
    {
        rigidBody.ApplyForces(deltaTime);
        rigidBody.Collisions(BaseColliders, deltaTime);
        rigidBody.Integrate(deltaTime);
    }
}
