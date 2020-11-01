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
}
