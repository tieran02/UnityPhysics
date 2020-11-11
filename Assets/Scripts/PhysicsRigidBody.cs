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

    //RestitutionCoefficient: 0 = Perfectly Inelastic, RestitutionCoefficient: 1 = Elastic
    public float RestitutionCoefficient = 0.6f;

    private readonly Vector3 GRAVITY_FORCE = new Vector3(0.0f, -9.8f, 0.0f);

    private Vector3 lastPosition;
    public Vector3 LastPosition => lastPosition;
    private Vector3 lastVelocity;
    public Vector3 LastVelocity => lastVelocity;
    private Vector3 lastMomentum;

    private Vector3 externalForces;

    //net force, change of momentum in respects to time (lastMomentum / delta)
    private Vector3 Fnet;

    private void Awake()
    {
        AddLinearImpulse(InitialImpulse);
        lastPosition = Position;
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
        return Velocity - lastVelocity;
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
        externalForces = GRAVITY_FORCE * Mass;
        Velocity += externalForces * deltaTime;
    }

    public void Collisions(BaseCollider[] colliders, float deltaTime)
    {
        //check for collision
        BaseCollider collider = GetComponent<BaseCollider>();
        if (collider != null)
        {
            float vc = 1.0f;

            //check if sphere collides with other spheres
            foreach (var otherCollider in colliders)
            {
                if (collider.RigidBody == null || collider == otherCollider)
                    continue;

                if (collider.CollisionOccured(otherCollider, deltaTime, ref vc))
                {
                    //Vector3 newVelocity = Velocity.normalized * vc;
                    //Velocity = newVelocity;

                    if(otherCollider.RigidBody)
                        ApplyLinearResponse(otherCollider.RigidBody);
                    else
                        ApplyLinearResponse();
                }
            }
        }
    }

    public void Integrate(float deltaTime)
    {
        lastPosition = Position;
        lastVelocity = Velocity;
        lastMomentum = Momentum();
        Fnet = lastMomentum / deltaTime;

        TranslatePosition(Velocity * deltaTime);
    }

    public void AddLinearImpulse(Vector3 impulse)
    {
        Velocity += impulse;
    }

    private void ApplyLinearResponse(PhysicsRigidBody other)
    {
        // Relative velocity
        Vector3 approachVelocity = Velocity - other.Velocity;

        //Restitution calculation (RestitutionCoefficient: 0 = Perfectly Inelastic, RestitutionCoefficient: 1 = Elastic)
        Vector3 J = (-approachVelocity * (RestitutionCoefficient + 1)) / ((1 / Mass) + (1 / other.Mass));

        Vector3 V1 = (J / Mass) + Velocity;
        Vector3 V2 = (-J / other.Mass) + other.Velocity;

        Velocity = V1;
        other.Velocity = V2;
    }

    private void ApplyLinearResponse()
    {
        // Relative velocity
        Vector3 approachVelocity = Velocity;

        Vector3 J = (-approachVelocity * (RestitutionCoefficient + 1)) / ((1 / Mass));

        Vector3 V1 = (J / Mass) + Velocity;

        Velocity = V1;
    }
}
