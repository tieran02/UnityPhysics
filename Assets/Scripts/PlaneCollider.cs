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

public class PlaneCollider : BaseCollider
{
    public Plane plane;
    public bool IsInfinite = true;

    protected override void Awake()
    {        
        Shape = ColliderShape.PLANE;

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
        base.Awake();
    }

    public override bool CollisionOccured(SphereCollider collider, float deltaTime, out CollisionData collisionData)
    {
        collisionData = new CollisionData();

        Vector3 N = plane.Normal();
        Vector3 V = collider.RigidBody.Velocity;

        float angle = Vector3.Angle(N, -V);

        if (angle < 90.0f)
        {
            Vector3 k = plane.A; // an arbitrary point on the plane
            Vector3 P = collider.transform.position - k; //a vector from k to the start of the sphere

            float q1 = Vector3.Angle(P, N);
            float q2 = (90.0f - q1) * Mathf.Deg2Rad;

            float s = Vector3.Angle(V, -N) * Mathf.Deg2Rad;

            float distance = Mathf.Sin(q2) * P.magnitude;

            //If the plane is finite, check if the colliers point is within the finite plane
            if (!IsInfinite && !IsPointOnFinitePlane(collider.transform.position, distance, N))
            {
                return false;
            }

            float vcMagnitude = (distance - collider.Radius) / Mathf.Cos(s);


            if (vcMagnitude <= (V.magnitude * deltaTime))
            {
                Vector3 pNorm = P.normalized;
                collisionData.ResolutionPoint = collider.transform.position + (pNorm * vcMagnitude);
                collisionData.CollisionNormal = N;
                collisionData.CollisionPoint = collisionData.ResolutionPoint + (V.normalized * collider.Radius);
                collisionData.Angle = angle;
                collisionData.VC = vcMagnitude;
                return true;
            };
        }

        return false;
    }

    //TODO implement plane to plane collision
    public override bool CollisionOccured(PlaneCollider collider, float deltaTime, out CollisionData collisionData)
    {
        throw new System.NotImplementedException();
    }

    private bool IsPointOnFinitePlane(Vector3 point, float distance, Vector3 normal)
    {
        //get the coplanar point
        Vector3 pl_cp = point - (distance * normal);
        Vector3 pa = pl_cp - plane.A;

        Vector3 AB = plane.AB();
        Vector3 AC = plane.AC();

        float dotAB = Vector3.Dot(AB, AB);
        float dotAC = Vector3.Dot(AC, AC);

        //Get the U,V positions local to the plane by using the AB and AC vectors of the plane
        //If U or V are less than 0 or greater than 1 the point is outside of the finite plane
        float u = Vector3.Dot(pa, AB) / dotAB;
        float v = Vector3.Dot(pa, AC) / dotAC;

        float radius = 0.5f;

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
