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

    public EulerSolver(PhysicsRigidBody[] rigidBodies, BaseCollider[] baseColliders) : base(1.0f/60.0f)
    {
        RigidBodies = rigidBodies;
        BaseColliders = baseColliders;

        States = new RigidStateVector[rigidBodies.Length];
        for (int i = 0; i < rigidBodies.Length; i++)
        {
            States[i] = new RigidStateVector(RigidBodies[i].Data);
        }
    }

    protected override void NextStep()
    {
        /*foreach (var rigidBody in RigidBodies)
        {
            if (rigidBody.State == PhysicsRigidBody.RigidbodyState.Sleep)
                continue;

            applyForce(rigidBody);
            Collisions(rigidBody);
            Integrate(rigidBody);
        }*/

        for (int i = 0; i < States.Length; i++)
        {
            PhysicsRigidBody rigidBody = RigidBodies[i];
            RigidStateVector state = States[i];


            ApplyMomentumState(rigidBody, ref state);
            ApplyAngularMomentumState(rigidBody, ref state);

            //ApplyPositionState(rigidBody, ref state);
            //ApplyRotation(rigidBody, ref state);

            //apply gravity
            ApplyExternalForces(rigidBody, ref state, deltaTime);

            //todo collisions
            Collisions(rigidBody, ref state);

            ApplyConstraints(rigidBody, ref state);

            //todo intergrate
            rigidBody.Integrate(ref state, deltaTime);
        }
    }

    void ApplyPositionState(PhysicsRigidBody rigidBody, ref RigidStateVector rigidState)
    {
        rigidState.Position += rigidBody.InverseMass * rigidState.Momentum;
    }

    void ApplyRotation(PhysicsRigidBody rigidBody, ref RigidStateVector rigidState)
    {
        //TODO angular velocity relative to radius
    }

    void ApplyMomentumState(PhysicsRigidBody rigidBody, ref RigidStateVector rigidState)
    {
        Vector3 acceleration = rigidBody.Velocity - rigidState.Velocity();
        Vector3 force = rigidBody.Mass * acceleration;
        rigidState.Momentum = force;
    }

    void ApplyAngularMomentumState(PhysicsRigidBody rigidBody, ref RigidStateVector rigidState)
    {
        //TODO angular momentum
        //rigidState.AngularMomentum = rigidBody.
    }

    private void ApplyExternalForces(PhysicsRigidBody rigidBody, ref RigidStateVector rigidState, float deltaTime)
    {
        Vector3 GravityForce = GRAVITY_FORCE * rigidBody.Mass;
        rigidState.Momentum += GravityForce * deltaTime;
    }

    private void ApplyConstraints(PhysicsRigidBody rigidBody, ref RigidStateVector rigidState)
    {
        if (rigidState.Momentum.sqrMagnitude < 0.01)
        {
            rigidState.Momentum = Vector3.zero;
        }
    }
    /*private void applyForce(PhysicsRigidBody rigidBody)
    {
        rigidBody.ExternalForces = GRAVITY_FORCE * rigidBody.Mass;
        rigidBody.Velocity += rigidBody.ExternalForces * deltaTime;

        rigidBody.AngularVelocity = rigidBody.InverseTensor().MultiplyVector(rigidBody.AngularMomentum);
        Quaternion q = new Quaternion(rigidBody.AngularVelocity.x, rigidBody.AngularVelocity.y,
            rigidBody.AngularVelocity.z, 0.0f);

        rigidBody.Spin = q.ScalarMultiply(0.5f) * rigidBody.Orientation;


    }*/

    public void Collisions(PhysicsRigidBody rigidBody, ref RigidStateVector rigidState)
    {
        //check for collision
        BaseCollider collider = rigidBody.GetComponent<BaseCollider>();
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

                    //check if the rigid body has no velocity relative to the surface normal (not moving away/towards the surface)
                    //TODO check if both velocities are identical if so then both objects are at rest relative to another
                    //TODO check if the angle between n and both/one the velocity are 90 (cos of angle = 0) then the two points are sliding

                    //Vector3 velocityWithoutExternalFoces = rigidBody.Velocity - (rigidBody.ExternalForces * deltaTime);
                    float angleN = Vector3.Angle(rigidBody.Velocity, collisionData.CollisionNormal.normalized);
                    if (angleN == 0.0f)
                    {
                        //rigidBody.State = PhysicsRigidBody.RigidbodyState.Sleep;
                        rigidBody.State = PhysicsRigidBody.RigidbodyState.Sliding;
                    }

                    switch (rigidBody.State)
                    {
                        case PhysicsRigidBody.RigidbodyState.Active:
                            if(otherCollider.RigidBody?.State != PhysicsRigidBody.RigidbodyState.Sliding)
                                ApplyLinearResponse(rigidBody, ref rigidState, collisionData,otherCollider.RigidBody);
                            else
                                ApplyLinearResponse(rigidBody, ref rigidState, collisionData, null);
                            break;
                        case PhysicsRigidBody.RigidbodyState.Sliding:
                            rigidState.Momentum = Vector3.ProjectOnPlane(rigidBody.Velocity, collisionData.CollisionNormal);
                            break;
                        case PhysicsRigidBody.RigidbodyState.Sleep:
                            break;
                        default:
                            throw new ArgumentOutOfRangeException();
                    }
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
            LinearResponse(rigidBody,ref rigidState, collisionData, out responseA, out angularVelocity);
        else
        {
            //other.State = RigidbodyState.Active;
            LinearResponse(rigidBody, ref rigidState, collisionData, other, out responseA, out responseB);
            if (responseB.sqrMagnitude > 0.5f)
            {
                other.AddLinearImpulse(responseB);
                //other.Velocity = responseB;
            }
            else
            {
                other.AddLinearImpulse(Vector3.ProjectOnPlane(other.Velocity, collisionData.CollisionNormal));
            }
        }

        if (responseA.sqrMagnitude > 0.5f)
            rigidState.Momentum = responseA;
        else
            rigidState.Momentum = Vector3.ProjectOnPlane(rigidState.Velocity(), collisionData.CollisionNormal);
    }

    private void LinearResponse(PhysicsRigidBody rigidBody, ref RigidStateVector rigidState, CollisionData collisionData, out Vector3 response, out Vector3 angularVelocity)
    {
        // Relative velocity
        Vector3 approachVelocity = rigidState.Velocity();
        Vector3 V1norm = approachVelocity.normalized;
        Vector3 Vb = 2 * collisionData.CollisionNormal * Vector3.Dot(collisionData.CollisionNormal, -V1norm) + V1norm;
        Vector3 newVelocity = Vb * approachVelocity.magnitude;

        Vector3 J = (newVelocity * (rigidBody.Data.RestitutionCoefficient + 1)) / ((1 / rigidBody.Mass));

        Vector3 V1 = (J / rigidBody.Mass) - newVelocity;

        //angularVelocity = Vector3.Cross(AngularVelocity * Time.fixedDeltaTime, ((collisionData.CollisionPoint - transform.position) - rigidbodyData.CenterOfMass));
        angularVelocity = Vector3.zero;

        response = V1;
    }

    private void LinearResponse(PhysicsRigidBody rigidBody, ref RigidStateVector rigidState, CollisionData collisionData, PhysicsRigidBody other, out Vector3 responseA, out Vector3 responseB)
    {
        // Relative velocity
        Vector3 r1 = collisionData.CollisionPoint - rigidState.Position;
        Vector3 r2 = collisionData.CollisionPoint - other.Position;

        Vector3 approachVelocity = (rigidState.Velocity()) - (other.Velocity);

        //Restitution calculation (RestitutionCoefficient: 0 = Perfectly Inelastic, RestitutionCoefficient: 1 = Elastic)
        Vector3 J = (-approachVelocity * (rigidBody.Data.RestitutionCoefficient + 1)) / ((1 / rigidBody.Mass) + (1 / other.Mass));

        Vector3 V1 = (J / rigidBody.Mass) + rigidState.Velocity();
        Vector3 V2 = (-J / other.Mass) + other.Velocity;

        responseA = V1;
        responseB = V2;
    }


    /*public void Integrate(PhysicsRigidBody rigidBody)
    {
        rigidBody.TranslatePosition(rigidBody.Velocity * deltaTime);
        rigidBody.transform.rotation = rigidBody.Spin;
    }*/
}
