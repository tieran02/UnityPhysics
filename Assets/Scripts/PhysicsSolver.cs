using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using UnityEngine;

public class PhysicsSolver : MonoBehaviour
{
    public PhysicsRigidBody[] RigidBodies;
    public PlaneCollider PlaneCollider;

    private Vector3 GRAVITY_FORCE = new Vector3(0.0f, -9.8f, 0.0f);
    private const float TIME_STEP = 1 / 60.0f;
    private float accumulator;
    private float lastTime;

    void Awake()
    {
        RigidBodies = FindObjectsOfType<PhysicsRigidBody>();
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
        rigidBody.Velocity += (GRAVITY_FORCE * deltaTime);

        //check for collision
        SphereCollider sphereCollider = rigidBody.GetComponent<SphereCollider>();
        if (sphereCollider != null && PlaneCollider != null)
        {
            float vc = 1.0f;
            //check if on collision course with plane
            if (PlaneCollider.SphereCollisionOccured(sphereCollider, deltaTime, ref vc))
            {
                Vector3 newVelocity = rigidBody.Velocity.normalized * vc;
                rigidBody.Velocity = newVelocity;

                //todo project velocity along the plane
            }
        }

        rigidBody.TranslatePosition(rigidBody.Velocity * deltaTime);
    }
}
