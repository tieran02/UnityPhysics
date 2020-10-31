using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using UnityEngine;

public class PhysicsRigidBody : MonoBehaviour
{
    public Vector3 Position => transform.position;
    public Vector3 Velocity;
    public Vector3 InitialImpulse;

    public float Mass = 1.0f;
    public Vector3 CenterOfMass;

    private readonly Vector3 GRAVITY_FORCE = new Vector3(0.0f, -9.8f, 0.0f);

    private Vector3 deltaVelocity;
    private Vector3 deltaMomentum;

    private Vector3 externalForces;

    //net force, change of momentum in respects to time (deltaMomentum / delta)
    private Vector3 Fnet;

    private void Awake()
    {
        AddLinearImpulse(InitialImpulse);
    }

    public void SetPosition(Vector3 Position)
    {
        transform.position = Position;
    }

    public void TranslatePosition(Vector3 translation)
    {
        transform.position += translation;
    }

    /// <summary>
    /// Amount of energy needed to bring a moving object to rest
    /// </summary>
    /// <returns></returns>
    public Vector3 Momentum()
    {
        return Mass * Velocity;
    }

    /// <summary>
    /// The acceleration of the object (based on the last steps velocity)
    /// </summary>
    /// <returns></returns>
    public Vector3 Acceleration()
    {
        return Velocity - deltaVelocity;
    }

    /// <summary>
    /// How much force the current rigidbody has in respects to the force and acceleration
    /// </summary>
    /// <returns></returns>
    public Vector3 Force()
    {
        return Mass * Acceleration();
    }


    public void ApplyForces(float deltaTime)
    {
        deltaVelocity = Velocity;
        deltaMomentum = Momentum();
        Fnet = deltaMomentum / deltaTime;


        externalForces = GRAVITY_FORCE * Mass;

        Velocity += externalForces * deltaTime;
    }

    public void Collisions(PlaneCollider planeCollider, SphereCollider[] sphereColliders, float deltaTime)
    {
        //check for collision
        SphereCollider sphereCollider = GetComponent<SphereCollider>();
        if (sphereCollider != null && planeCollider != null)
        {
            float vc = 1.0f;
            //check if on collision course with plane
            if (planeCollider.SphereCollisionOccured(sphereCollider, deltaTime, ref vc))
            {
                Vector3 newVelocity = Velocity.normalized * vc;
                Velocity = newVelocity;

                //todo project velocity along the plane
            }

            //check if sphere collides with other spheres
            foreach (var otherSphereCollider in sphereColliders)
            {
                if (sphereCollider.RigidBody == null || sphereCollider == otherSphereCollider)
                    continue;

                if (sphereCollider.SphereCollisionOccured(otherSphereCollider, deltaTime, ref vc))
                {
                    Vector3 newVelocity = Velocity.normalized * vc;
                    Velocity = newVelocity;
                }
            }
        }
    }

    public void Integrate(float deltaTime)
    {
        deltaVelocity = Velocity;
        deltaMomentum = Momentum();
        Fnet = deltaMomentum / deltaTime;

        TranslatePosition(Velocity * deltaTime);
    }

    public void AddLinearImpulse(Vector3 impulse)
    {
        Velocity += impulse;
    }
}
