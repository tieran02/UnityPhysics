using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using UnityEngine;

public struct CollisionData
{
    public Vector3 ResolutionPoint;
    public Vector3 CollisionNormal;
    public Vector3 ContactPoint;
    public float PenetrationDepth;
    public float Angle;
    public float VC;
}

public abstract class BaseCollider : MonoBehaviour
{
    public enum ColliderShape
    {
        PLANE,
        SPHERE,
        POINT,
    }

    public PhysicsRigidBody RigidBody;

    public ColliderShape Shape { get; protected set; }

    public abstract bool CollisionOccured(SphereCollider collider, float deltaTime, out CollisionData collisionData);
    public abstract bool CollisionOccured(PlaneCollider collider, float deltaTime, out CollisionData collisionData);
    public abstract bool CollisionOccured(Vector3 point, Vector3 velocity, float deltaTime, out CollisionData collisionData);

    protected virtual void Awake()
    {
        RigidBody = GetComponent<PhysicsRigidBody>();
    }

    public bool CollisionOccured(BaseCollider collider, float deltaTime, ref float vcMagnitude)
    {
        switch (Shape)
        {
            case ColliderShape.SPHERE:
                return CollisionOccured((SphereCollider)collider, deltaTime, ref vcMagnitude);
            case ColliderShape.PLANE:
                return CollisionOccured((PlaneCollider)collider, deltaTime, ref vcMagnitude);
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