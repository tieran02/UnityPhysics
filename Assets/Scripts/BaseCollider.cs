using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using UnityEngine;

public struct CollisionData
{
    public Vector3 CollisionPoint;
    public Vector3 CollisionNormal;
}

public abstract class BaseCollider : MonoBehaviour
{
    public PhysicsRigidBody RigidBody;

    public abstract bool CollisionOccured(SphereCollider collider, float deltaTime, out CollisionData collisionData);
    public abstract bool CollisionOccured(PlaneCollider collider, float deltaTime, out CollisionData collisionData);

    protected virtual void Awake()
    {
        RigidBody = GetComponent<PhysicsRigidBody>();
    }

    public bool CollisionOccured(BaseCollider collider, float deltaTime, ref float vcMagnitude)
    {
        switch (collider)
        {
            case SphereCollider sphereCollider:
                return CollisionOccured(sphereCollider, deltaTime, ref vcMagnitude);
                break;
            case PlaneCollider planeCollider:
                return CollisionOccured(planeCollider, deltaTime, ref vcMagnitude);
                break;
        }

        return false;
    }

    public bool CollisionOccured(BaseCollider collider, float deltaTime, out CollisionData collisionData)
    {
        switch (collider)
        {
            case SphereCollider sphereCollider:
                return CollisionOccured(sphereCollider, deltaTime, out collisionData);
            case PlaneCollider planeCollider:
                return CollisionOccured(planeCollider, deltaTime, out collisionData);
        }
        collisionData = new CollisionData();
        return false;
    }
}