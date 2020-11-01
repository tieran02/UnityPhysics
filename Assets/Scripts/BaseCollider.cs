using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using UnityEngine;

public abstract class BaseCollider : MonoBehaviour
{
    public PhysicsRigidBody RigidBody;

    public abstract bool CollisionOccured(SphereCollider collider, float deltaTime, ref float vcMagnitude);
    public abstract bool CollisionOccured(PlaneCollider collider, float deltaTime, ref float vcMagnitude);

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
}