using System.Collections;
using System.Collections.Generic;
using UnityEngine;

[System.Serializable]
public struct Plane
{
    Plane(Vector3 a, Vector3 b, Vector3 c)
    {
        A = a;
        B = b;
        C = c;
    }

    public Vector3 A, B, C;

    public Vector3 Normal()
    {
        return Vector3.Cross(AC(),AB()).normalized;
    }

    public Vector3 AB()
    {
        return B - A;
    }

    public Vector3 AC()
    {
        return C - A;
    }

    public Vector3 Center()
    {
        return (A + B + C) / 3.0f;
    }
}

public class PlaneCollider : MonoBehaviour
{
    public Plane plane;
    public bool IsInfinite = true;

    void Awake()
    {
        var meshFilter = GetComponent<MeshFilter>();
        if (meshFilter != null)
        {
            var mesh = meshFilter.mesh;
            if(mesh == null)
                return;

            plane.A = transform.TransformPoint(mesh.vertices[0]);
            plane.B = transform.TransformPoint(mesh.vertices[1]);
            plane.C = transform.TransformPoint(mesh.vertices[2]);
        }
    }

    public bool SphereCollisionOccured(SphereCollider sphereCollider, float deltaTime, ref float vcMagnitude)
    {
        Vector3 N = plane.Normal();
        Vector3 V = sphereCollider.RigidBody.Velocity;

        float angle = Vector3.Angle(N, -V);

        if (angle <= 90.0f)
        {
            Vector3 k = plane.A; // an arbitrary point on the plane
            Vector3 P = sphereCollider.transform.position - k; //a vector from k to the start of the sphere

            float q1 = Vector3.Angle(P, N);
            float q2 = (90.0f - q1) * Mathf.Deg2Rad;

            float s = Vector3.Angle(V, -N) * Mathf.Deg2Rad;

            float distance = Mathf.Sin(q2) * P.magnitude;

            //If the plane is finite, check if the colliers point is within the finite plane
            if (!IsInfinite && !IsPointOnFinitePlane(sphereCollider.transform.position,distance,N))
            {
                return false;
            }

            vcMagnitude = (distance - sphereCollider.Radius) / Mathf.Cos(s);


            return vcMagnitude <= (V.magnitude * deltaTime);
        }

        return false;
    }

    private bool IsPointOnFinitePlane(Vector3 point, float distance, Vector3 normal)
    {
        //get the coplanar point
        Vector3 pl_cp = point - (distance * normal);
        Vector3 pa = pl_cp - plane.A;

        Vector3 AB = plane.AB();
        Vector3 AC = plane.AC();

        //Get the U,V positions local to the plane by using the AB and AC vectors of the plane
        //If U or V are less than 0 or greater than 1 the point is outside of the finite plane
        float u = Vector3.Dot(pa, AB) / Vector3.Dot(AB, AB);
        float v = Vector3.Dot(pa, AC) / Vector3.Dot(AC, AC);

        if (u < 0.0f || u > 1.0f || v < 0.0f || v > 1.0f)
            return false;

        return true;
    }

    void OnDrawGizmos()
    {
        var worldPlane = plane;
        DrawPlane(worldPlane, Color.red);
    }

    public static void DrawPlane(Plane plane, Color color)
    {

        Gizmos.color = Color.red;
        Gizmos.DrawWireSphere(plane.A,.25f);
        Gizmos.color = Color.green;
        Gizmos.DrawWireSphere(plane.B, .25f);
        Gizmos.color = Color.blue;
        Gizmos.DrawWireSphere(plane.C, .25f);

        Gizmos.color = Color.white;
        Gizmos.DrawLine(plane.Center(), plane.Center() + plane.Normal() * 2.0f);

        //
        Gizmos.color = Color.red;
        Gizmos.DrawLine(plane.A, plane.B);
        Gizmos.DrawLine(plane.C, plane.A);
    }


}
