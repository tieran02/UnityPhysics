using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using UnityEngine;

public class PhysicsRigidBodyVolume
{
    public Vector3 Position;
    public Vector3 Velocity;
    public Vector3 Forces; //Sum of all forces being applied to this volume

    public float Mass = 1.0f;
    public float Friction = 0.6f;

    //TODO add a base collider, for now just use a sphere
    public SphereCollider Collider;

    public void Update(float deltaTime)
    {

    }

    public void ApplyForces()
    {

    }
}
