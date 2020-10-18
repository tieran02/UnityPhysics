using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class SphereCollider : MonoBehaviour
{
    public float Radius = 0.5f;
    public PhysicsRigidBody RigidBody;

    void Awake()
    {
        RigidBody = GetComponent<PhysicsRigidBody>();
    }

    void OnDrawGizmos()
    {
        Gizmos.DrawWireSphere(transform.position, Radius);
    }

    public bool Intersect(SphereCollider other)
    {
        //avoid using square root operation
        Vector3 s = this.transform.position - other.transform.position;
        return s.sqrMagnitude <= ((this.Radius + other.Radius) * (this.Radius + other.Radius));
    }

    public bool Intersect(PlaneCollider other)
    {
        float VC = 0.0f;
        return other.SphereCollisionOccured(this, 1.0f / 60.0f, ref VC);
    }

    public bool SphereCollisionOccured(SphereCollider otherSphereCollider, float deltaTime, ref float vcMagnitude)
    {
        Vector3 A = otherSphereCollider.transform.position - transform.position; // the vector from the centre of sphere one to center of sphere 2
        Vector3 V = RigidBody.Velocity; // the vector of motion

        float aMagnitude = A.magnitude;
        float q = Mathf.Acos(Vector3.Dot(A.normalized, V.normalized)) * Mathf.Deg2Rad; //angle between A and V in radians
        float d = Mathf.Sin(q) * aMagnitude;

        //e = the distance along v between point of closest approach(d) and the point of collision 
        float radiusSq = (Radius + otherSphereCollider.Radius) * (Radius + otherSphereCollider.Radius);
        float dSq = d * d;
        float e = Mathf.Sqrt(radiusSq - dSq);

        vcMagnitude = Mathf.Cos(q) * aMagnitude - e;

        Debug.Log($"Distance: {d}  vc:{vcMagnitude}  v:{V.magnitude}  q:{q}  e:{e}");

        return vcMagnitude <= V.magnitude * deltaTime;

    }
}
