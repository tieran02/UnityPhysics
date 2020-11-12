using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
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
            applyForce(rigidBody);
            Collisions(rigidBody);
            Integrate(rigidBody);
        }
    }

    private void applyForce(PhysicsRigidBody rigidBody)
    {
        rigidBody.externalForces = GRAVITY_FORCE * rigidBody.Mass;
        rigidBody.Velocity += rigidBody.externalForces * deltaTime;
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
                    if (otherCollider.RigidBody)
                        rigidBody.ApplyLinearResponse(otherCollider.RigidBody);
                    else
                        rigidBody.ApplyLinearResponse(collisionData.CollisionNormal);
                }
            }
        }
    }

    public void Integrate(PhysicsRigidBody rigidBody)
    {
        rigidBody.TranslatePosition(rigidBody.Velocity * deltaTime);
    }
}
