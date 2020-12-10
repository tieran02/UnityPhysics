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

            IdleState(rigidBody, ref state);
            if(rigidBody.State == PhysicsRigidBody.RigidbodyState.Sleep)
            {
                continue;
            }

            ApplyMomentumState(rigidBody, ref state);
            ApplyAngularMomentumState(rigidBody, ref state);

            //apply gravity
            ApplyExternalForces(rigidBody, ref state, deltaTime);

            //ApplyPositionState(rigidBody, ref state, deltaTime);
            //ApplyRotation(rigidBody, ref state);

            ApplyPositionState(rigidBody, ref state, deltaTime);

            ApplyConstraints(rigidBody, ref state);


            //TODO apply state to rigidbody before collision check
            rigidBody.ApplyState(state);

            //todo collisions
            Collisions(rigidBody, ref state);

            //todo intergrate
            rigidBody.Integrate(ref state, deltaTime);
        }
    }

    void ApplyPositionState(PhysicsRigidBody rigidBody, ref RigidStateVector rigidState, float deltaTime)
    {
        rigidState.Position += rigidBody.InverseMass * rigidState.Momentum * deltaTime;
    }

    void ApplyRotation(PhysicsRigidBody rigidBody, ref RigidStateVector rigidState)
    {
        //TODO angular velocity relative to radius
    }

    void ApplyMomentumState(PhysicsRigidBody rigidBody, ref RigidStateVector rigidState)
    {
        Vector3 acceleration = rigidBody.Velocity;
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
        //onlu apply gravity if active
        if (rigidBody.State != PhysicsRigidBody.RigidbodyState.Active)
            return;

        Vector3 GravityForce = GRAVITY_FORCE * rigidBody.Mass;
        rigidState.Momentum += GravityForce * deltaTime;
    }

    private void ApplyConstraints(PhysicsRigidBody rigidBody, ref RigidStateVector rigidState)
    {
        if (rigidState.Momentum.sqrMagnitude < 0.02)
        {
            //rigidState.Momentum = Vector3.zero;
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
    private void IdleState(PhysicsRigidBody rigidBody, ref RigidStateVector rigidState)
    {
        const float sleepThreshold = 0.02f;
        const float slideThreshold = 0.03f;
        const float awakeThreshold = 0.08f;

        const float idleFrameTime = 0.4f;

        switch (rigidBody.State)
        {
            case PhysicsRigidBody.RigidbodyState.Active:
                if (rigidState.Momentum.sqrMagnitude < slideThreshold)
                {
                    idleTime[rigidBody] += deltaTime;
                    if (idleTime[rigidBody] >= idleFrameTime)
                    {
                        rigidBody.State = PhysicsRigidBody.RigidbodyState.Sliding;
                        idleTime[rigidBody] = 0.0f;
                    }
                }
                break;
            case PhysicsRigidBody.RigidbodyState.Sliding:
                if (rigidState.Momentum.sqrMagnitude < sleepThreshold)
                {
                    idleTime[rigidBody] += deltaTime;
                    if (idleTime[rigidBody] >= idleFrameTime)
                    {
                        rigidBody.State = PhysicsRigidBody.RigidbodyState.Sleep;
                        idleTime[rigidBody] = 0.0f;
                    }
                }
                break;
            case PhysicsRigidBody.RigidbodyState.Sleep:
                if (rigidState.Momentum.sqrMagnitude > awakeThreshold)
                {
                    rigidBody.State = PhysicsRigidBody.RigidbodyState.Active;
                    idleTime[rigidBody] = 0.0f;
                }
                break;
            default:
                break;
        }
    }

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
                    float angleN = Vector3.Angle(rigidBody.Velocity, collisionData.CollisionNormal);
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
                            rigidState.Momentum = Vector3.ProjectOnPlane(rigidState.Momentum, collisionData.CollisionNormal);
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
            other.AddLinearImpulse(responseB * other.Mass);
        }

        rigidState.Momentum = responseA * rigidBody.Mass;
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
        //rigidState.Position = collisionData.ResolutionPoint;
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
}
