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
