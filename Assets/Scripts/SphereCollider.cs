using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class SphereCollider : MonoBehaviour
{
    public float Radius = 0.5f;
    // Start is called before the first frame update
    void Start()
    {
        
    }

    // Update is called once per frame
    void Update()
    {
        
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
        float distance = Vector3.Dot((transform.position - other.plane.A), other.plane.Normal());
        return distance <= Radius;
    }
}
