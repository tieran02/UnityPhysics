using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Assets.Scripts;
using UnityEngine;

public class EulerSolver : Solver
{
    private PhysicsRigidBody[] RigidBodies;
    private RigidStateVector[] States;

    private BaseCollider[] BaseColliders;

    private readonly Vector3 GRAVITY_FORCE = new Vector3(0.0f, -9.8f, 0.0f);
    private const float frictionCoeffient = 0.1f;

    private Dictionary<PhysicsRigidBody, float> idleTime = new Dictionary<PhysicsRigidBody, float>();

    public EulerSolver(PhysicsRigidBody[] rigidBodies, BaseCollider[] baseColliders) : base(1.0f/60.0f)
    {
        RigidBodies = rigidBodies;
        BaseColliders = baseColliders;

        States = new RigidStateVector[rigidBodies.Length];
        for (int i = 0; i < rigidBodies.Length; i++)
        {
            States[i] = new RigidStateVector(RigidBodies[i].Data);
            idleTime[rigidBodies[i]] = 0.0f;
        }
    }

    protected override void NextStep()
    {
        for (int i = 0; i < States.Length; i++)
        {
            PhysicsRigidBody rigidBody = RigidBodies[i];
            RigidStateVector state = States[i];
            state.Position = rigidBody.Position;
            state.Momentum = rigidBody.Momentum();
            state.Orientation = rigidBody.Orientation;

            //apply gravity
            ApplyExternalForces(rigidBody, ref state, deltaTime);
            ApplyPositionState(rigidBody, ref state, deltaTime);

            Collisions(rigidBody.Collider, ref state);

            rigidBody.Integrate(ref state, deltaTime);
        }
    }

    void ApplyPositionState(PhysicsRigidBody rigidBody, ref RigidStateVector rigidState, float deltaTime)
    {
        rigidState.Position += rigidBody.InverseMass * rigidState.Momentum * deltaTime;
    }

    private void ApplyExternalForces(PhysicsRigidBody rigidBody, ref RigidStateVector rigidState, float deltaTime)
    {
        //onlu apply gravity if active
        if (rigidBody.State != PhysicsRigidBody.RigidbodyState.Active)
            return;

        Vector3 GravityForce = GRAVITY_FORCE * rigidBody.Mass;
        rigidState.Momentum += GravityForce * deltaTime;
    }

    public void Collisions(BaseCollider collider, ref RigidStateVector rigidState)
    {
        //check for collision
        if (collider != null)
        {
            //check if sphere collides with other spheres
            foreach (var otherCollider in BaseColliders)
            {
                if (collider.RigidBody == null || collider == otherCollider)
                    continue;

                CollisionData collisionData;
                if (collider.CollisionOccured(otherCollider, deltaTime, out collisionData))
                {
                    if (otherCollider.RigidBody != null && otherCollider.RigidBody.State != PhysicsRigidBody.RigidbodyState.Sliding)
                    {
                        //if collided with other and the other object is not sliding set the other rigid body to awake
                        otherCollider.RigidBody.State = PhysicsRigidBody.RigidbodyState.Active;
                    }

                    ApplyLinearResponse(collider.RigidBody, ref rigidState, collisionData, otherCollider.RigidBody);
                }
            }
        }
    }

    public void ApplyLinearResponse(PhysicsRigidBody rigidBody, ref RigidStateVector rigidState, CollisionData collisionData, PhysicsRigidBody other)
    {
        Vector3 responseA;
        Vector3 responseB;

        Vector3 angularVelocity;
        if (!other)
            LinearResponse(ref rigidState, collisionData, rigidBody.Data.RestitutionCoefficient, out responseA, out angularVelocity);
        else
        {
            LinearResponse(ref rigidState, collisionData, other, rigidBody.Data.RestitutionCoefficient, out responseA, out responseB);
            other.AddLinearImpulse(responseB * other.Mass);
        }

        rigidState.Momentum = responseA * rigidState.Mass;
    }

    private void LinearResponse(ref RigidStateVector rigidState, CollisionData collisionData, float restitution, out Vector3 response, out Vector3 angularVelocity)
    {
        Vector3 targetNormal = collisionData.CollisionNormal;
        //Vector3 targetPoint = newPosition + newVelocity.normalized * collisionData.VC;
        //here we can have the colliders velocity, for now all colliders are static, thus have no velocity
        Vector3 colliderVelocityAtTargetPoint = Vector3.zero;

        Vector3 relativeVelocity = rigidState.Velocity() - colliderVelocityAtTargetPoint;
        float normalDotRelativeVelocity = Vector3.Dot(targetNormal, relativeVelocity);
        Vector3 relativeVelocityNormal = normalDotRelativeVelocity * targetNormal;
        Vector3 relativeVelocityT = relativeVelocity - relativeVelocityNormal;

        // Check if the velocity is facing opposite direction of the surface
        // normal
        if (normalDotRelativeVelocity < 0.0)
        {
            //Apply restitution coefficient to the surface normal component of the velocity
            Vector3 deltaRelativeVelocityNormal = ((-restitution - 1.0f)) * relativeVelocityNormal;
            relativeVelocityNormal *= -restitution;

            // Apply friction to the tangential component of the velocity
            // http://graphics.stanford.edu/papers/cloth-sig02/cloth.pdf
            if (relativeVelocityT.sqrMagnitude > 0.0f)
            {
                float frictionScale = Mathf.Max(1.0f - frictionCoeffient *
                    deltaRelativeVelocityNormal.magnitude / relativeVelocityT.magnitude, 0.0f);
                relativeVelocityT *= frictionScale;
            }

            relativeVelocityT = Vector3.ProjectOnPlane(relativeVelocityT, targetNormal);

        }
        
        response = relativeVelocityNormal + relativeVelocityT + colliderVelocityAtTargetPoint;
        angularVelocity = Vector3.zero;
        rigidState.Position = collisionData.ResolutionPoint + (response * deltaTime);
    }

    private void LinearResponse(ref RigidStateVector rigidState, CollisionData collisionData, PhysicsRigidBody other, float restitution, out Vector3 responseA, out Vector3 responseB)
    {
        // Relative velocity
        Vector3 r1 = collisionData.ContactPoint - rigidState.Position;
        Vector3 r2 = collisionData.ContactPoint - other.Position;

        Vector3 approachVelocity = (rigidState.Velocity()) - (other.Velocity);
        Vector3 V1norm = approachVelocity.normalized;
        Vector3 Vb = 2 * collisionData.CollisionNormal * Vector3.Dot(collisionData.CollisionNormal, -V1norm) + V1norm;
        Vector3 newVelocity = Vb * approachVelocity.magnitude;

        //Restitution calculation (RestitutionCoefficient: 0 = Perfectly Inelastic, RestitutionCoefficient: 1 = Elastic)
        Vector3 J = (-approachVelocity * (restitution + 1)) / ((1 / rigidState.Mass) + (1 / other.Mass));

        Vector3 V1 = (J / rigidState.Mass) + newVelocity;
        Vector3 V2 = (-J / other.Mass) - newVelocity;

        responseA = V1;
        responseB = V2;

        rigidState.Position += V1.normalized * collisionData.VC;
    }
}
