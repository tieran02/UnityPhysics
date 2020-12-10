using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class SphereCollider : BaseCollider
{
    public float Radius = 0.5f;

    protected override void Awake()
    {
        Shape = ColliderShape.SPHERE;

        base.Awake();
    }

    public override bool CollisionOccured(PlaneCollider collider, float deltaTime, out CollisionData collisionData)
    {
        return collider.CollisionOccured(this, deltaTime, out collisionData);
    }

    public override bool CollisionOccured(SphereCollider sphereCollider, float deltaTime, out CollisionData collisionData)
    {
        collisionData = new CollisionData();
        float vcMagnitude = 0.0f;

        Vector3 A = sphereCollider.transform.position - transform.position; // the vector from the centre of sphere one to center of sphere 2
        Vector3 V = RigidBody.Velocity; // the vector of motion

        //first check if both colliders are in motion
        if (RigidBody != null && sphereCollider.RigidBody != null)
        {
            if (motionCollision(sphereCollider, deltaTime, ref vcMagnitude))
            {
                collisionData.ResolutionPoint = transform.position + (A.normalized * vcMagnitude);
                Vector3 N = (collisionData.ResolutionPoint - sphereCollider.transform.position).normalized;
                collisionData.CollisionNormal = N;
                collisionData.VC = vcMagnitude;
                return true;
            }
            return false;
        }


        float aMagnitude = A.magnitude;
        float q = Mathf.Acos(Vector3.Dot(A.normalized, V.normalized)) * Mathf.Deg2Rad; //angle between A and V in radians
        float d = Mathf.Sin(q) * aMagnitude;

        //e = the distance along v between point of closest approach(d) and the point of collision 
        float radiusSq = (Radius + sphereCollider.Radius) * (Radius + sphereCollider.Radius);
        float dSq = d * d;
        float e = Mathf.Sqrt(radiusSq - dSq);

        vcMagnitude = Mathf.Cos(q) * aMagnitude - e;

        //Debug.Log($"Distance: {d}  vc:{vcMagnitude}  v:{V.magnitude}  q:{q}  e:{e}");

        if (vcMagnitude <= V.magnitude * deltaTime)
        {
            collisionData.ResolutionPoint = transform.position + (A.normalized * vcMagnitude);
            Vector3 N = (collisionData.ResolutionPoint - sphereCollider.transform.position).normalized;
            collisionData.CollisionNormal = N;
            collisionData.VC = vcMagnitude;
            return true;
        }

        return false;
    }

    //TODO convert to use collisionData
    private bool motionCollision(SphereCollider sphereCollider, float deltaTime, ref float vcMagnitude)
    {
        PhysicsRigidBody otherRigidBody = sphereCollider.RigidBody;
        if (RigidBody == null || otherRigidBody == null)
            return false;

        //vector from last positon to other sphers last positon
        Vector3 AB = otherRigidBody.Position - RigidBody.Position;

        //relative velocity from last time step
        Vector3 vab = (otherRigidBody.Velocity - RigidBody.Velocity) * deltaTime;

        float radiusAB = Radius + sphereCollider.Radius;

        float a = Vector3.Dot(vab, vab);
        float b = 2 * Vector3.Dot(AB, vab);
        float c = Vector3.Dot(AB, AB) - (radiusAB * radiusAB);

        //first check if the two spheres are currently overlapping
        if(Vector3.Dot(AB,AB) <= (radiusAB * radiusAB))
        {
            vcMagnitude = 0.0f;
            return true;
        }

        //check if the two sphere hit each other this frame
        float result1;
        float result2;
        if(QuadraticFormular(a,b,c,out result1,out result2))
        {
            if (result1 < result2)
                vcMagnitude = result1;
            else
                vcMagnitude = result2;

            if(vcMagnitude <= radiusAB && vcMagnitude > 0.0f)
                return true;
        }


        return false;
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

    void OnDrawGizmos()
    {
        Gizmos.DrawWireSphere(transform.position, Radius);
    }
}
