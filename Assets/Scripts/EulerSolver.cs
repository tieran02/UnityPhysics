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
    private BaseCollider[] BaseColliders;

    private readonly Vector3 GRAVITY_FORCE = new Vector3(0.0f, -9.8f, 0.0f);

    public EulerSolver(PhysicsRigidBody[] rigidBodies, BaseCollider[] baseColliders) : base(1.0f/60.0f)
    {
        RigidBodies = rigidBodies;
        BaseColliders = baseColliders;
    }

    protected override void NextStep()
    {
        foreach (var rigidBody in RigidBodies)
        {
            if (rigidBody.State == PhysicsRigidBody.RigidbodyState.Sleep)
                continue;

            applyForce(rigidBody);
            Collisions(rigidBody);
            Integrate(rigidBody);
        }
    }

    private void applyForce(PhysicsRigidBody rigidBody)
    {
        rigidBody.ExternalForces = GRAVITY_FORCE * rigidBody.Mass;
        rigidBody.Velocity += rigidBody.ExternalForces * deltaTime;

        rigidBody.AngularVelocity = rigidBody.InverseTensor().MultiplyVector(rigidBody.AngularMomentum);
        Quaternion q = new Quaternion(rigidBody.AngularVelocity.x, rigidBody.AngularVelocity.y,
            rigidBody.AngularVelocity.z, 0.0f);

        rigidBody.Spin = q.ScalarMultiply(0.5f) * rigidBody.Orientation;


    }

    public void Collisions(PhysicsRigidBody rigidBody)
    {
        //check for collision
        BaseCollider collider = rigidBody.GetComponent<BaseCollider>();
        if (collider != null)
        {
            float vc = 1.0f;

            //check if sphere collides with other spheres
            foreach (var otherCollider in BaseColliders)
            {
                if (collider.RigidBody == null || collider == otherCollider)
                    continue;

                CollisionData collisionData;
                if (collider.CollisionOccured(otherCollider, deltaTime, out collisionData))
                {
                    //collider.transform.position = collisionData.ResolutionPoint;
                    Vector3 relativeVelocity = otherCollider.RigidBody
                        ? otherCollider.RigidBody.Velocity - rigidBody.Velocity
                        : rigidBody.Velocity;


                    collider.transform.position = collisionData.ResolutionPoint;
                    if (otherCollider.RigidBody)
                        rigidBody.ApplyLinearResponse(otherCollider.RigidBody);
                    else
                    {
                        rigidBody.ApplyLinearResponse(collisionData);
                    }
                }
            }
        }
    }

    public void Integrate(PhysicsRigidBody rigidBody)
    {
        rigidBody.TranslatePosition(rigidBody.Velocity * deltaTime);
        rigidBody.transform.rotation = rigidBody.Spin;
    }
}
