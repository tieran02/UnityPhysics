using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using UnityEngine;

public class PhysicsRigidBody : MonoBehaviour
{
    public Vector3 Position => transform.position;
    public Vector3 Velocity;
    public Vector3 InitialVelocity;

    private void Awake()
    {
        Velocity = InitialVelocity;
    }

    public void SetPosition(Vector3 Position)
    {
        transform.position = Position;
    }

    public void TranslatePosition(Vector3 translation)
    {
        transform.position += translation;
    }
}
