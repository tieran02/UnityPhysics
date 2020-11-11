using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class SphereCollider : BaseCollider
{
    public float Radius = 0.5f;

    void OnDrawGizmos()
    {
        Gizmos.DrawWireSphere(transform.position, Radius);
    }

    public override bool CollisionOccured(SphereCollider sphereCollider, float deltaTime, ref float vcMagnitude)
    {
        //first check if both colliders are in motion
        if (RigidBody != null && sphereCollider.RigidBody != null && RigidBody.Velocity.sqrMagnitude != 0.0f &&
            sphereCollider.RigidBody.Velocity.sqrMagnitude != 0.0f)
        {
            return motionCollision(sphereCollider, deltaTime, ref vcMagnitude);
        }


        Vector3 A = sphereCollider.transform.position - transform.position; // the vector from the centre of sphere one to center of sphere 2
        Vector3 V = RigidBody.Velocity; // the vector of motion

        float aMagnitude = A.magnitude;
        float q = Mathf.Acos(Vector3.Dot(A.normalized, V.normalized)) * Mathf.Deg2Rad; //angle between A and V in radians
        float d = Mathf.Sin(q) * aMagnitude;

        //e = the distance along v between point of closest approach(d) and the point of collision 
        float radiusSq = (Radius + sphereCollider.Radius) * (Radius + sphereCollider.Radius);
        float dSq = d * d;
        float e = Mathf.Sqrt(radiusSq - dSq);

        vcMagnitude = Mathf.Cos(q) * aMagnitude - e;

        //Debug.Log($"Distance: {d}  vc:{vcMagnitude}  v:{V.magnitude}  q:{q}  e:{e}");

        return vcMagnitude <= V.magnitude * deltaTime;
    }

    public override bool CollisionOccured(PlaneCollider collider, float deltaTime, ref float vcMagnitude)
    {
        return collider.CollisionOccured(this,deltaTime, ref vcMagnitude);
    }

    private bool motionCollision(SphereCollider sphereCollider, float deltaTime, ref float vcMagnitude)
    {
        PhysicsRigidBody otherRigidBody = sphereCollider.RigidBody;
        if (RigidBody == null || otherRigidBody == null)
            return false;

        float deltaXPosition = sphereCollider.transform.position.x - transform.position.x;
        float deltaYPosition = sphereCollider.transform.position.y - transform.position.y;
        float deltaZPosition = sphereCollider.transform.position.z - transform.position.z;

        float deltaXVelocity = (otherRigidBody.Velocity.x - RigidBody.Velocity.x);
        float deltaYVelocity = (otherRigidBody.Velocity.y - RigidBody.Velocity.y);
        float deltaZVelocity = (otherRigidBody.Velocity.z - RigidBody.Velocity.z);

        float A = deltaXVelocity * deltaXVelocity + deltaYVelocity * deltaYVelocity +
                  deltaZVelocity * deltaZVelocity;

        float B = 2 * deltaXPosition * deltaXVelocity +
                  2 * deltaYPosition * deltaYVelocity +
                  2 * deltaZPosition * deltaZVelocity;

        float C = deltaXPosition * deltaXPosition + deltaYPosition * deltaYPosition +
                  deltaZPosition * deltaZPosition +
                  (Radius + sphereCollider.Radius) * (Radius + sphereCollider.Radius);

        float preRoot = B * B - 4 * A * C;
        if (preRoot < 0.0f)
            return false;


        double sqr = Mathf.Sqrt(preRoot);
        double t1 = (-B + sqr) / 2 * A;
        double t2 = (-B - sqr) / 2 * A;

        float collisionPoint = Mathf.Min((float)t1, (float)t2);

        return true;
    }

    private bool QuadraticFormular(float A, float B, float C, out float result1, out float result2)
    {
        float preroot = B * B - 4 * A * C;

        if(preroot > 0.0f)
        {
            float sqr = Mathf.Sqrt(preroot);
            float d = 1.0f / (2 * A);
            result1 = (-B + sqr) * d;
            result2 = (-B - sqr) * d;
            return true;
        }

        result1 = 0;
        result2 = 0;
        return false;
    }
}
