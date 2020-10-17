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

}
