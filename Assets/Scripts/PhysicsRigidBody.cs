﻿using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using UnityEngine;

public struct RigidStateVector
{
    public Vector3 Position;
    public Quaternion Rotation;
    public Vector3 Momentum;
    public Quaternion Orientation;
    public Vector3 AngularMomentum;

    //secondary vars
    public readonly float Mass;
    public readonly float InverseMass;

    public readonly Matrix4x4 Inertia;
    public readonly Matrix4x4 Inverseinertia;

    public RigidStateVector(RigidbodyData data)
    {
        Position = data.Position;
        Rotation = default;
        Momentum = default;
        AngularMomentum = default;
        Orientation = data.Orientation;

        Inertia = data.InertiaTensor;
        Inverseinertia = data.InverseInertiaTensor;

        Mass = data.Mass;
        InverseMass = Mass == 0 ? 0 : 1.0f / Mass;
    }

    public Vector3 Velocity()
    {
        return Momentum * InverseMass;
    }

    public Vector3 AngularVelocity()
    {
        return Inverseinertia.MultiplyVector(AngularMomentum);
    }

    public Quaternion Spin()
    {
        var angularVelocity = AngularVelocity();
        Quaternion q = new Quaternion(angularVelocity.x, angularVelocity.y, angularVelocity.z, 1.0f);
        return q.ScalarMultiply(0.5f) * Orientation;
    }
}

[System.Serializable]
public struct RigidbodyData
{
    public float Mass;
    public float InverseMass;
    public Matrix4x4 InertiaTensor;
    public Matrix4x4 InverseInertiaTensor;

    public Vector3 Position;
    public Vector3 Velocity;
    public Vector3 Acceleration;
    public Vector3 AngularMomentum;
    public Vector3 AngularVelocity;
    public Quaternion Orientation;
    public Vector3 Forces;
    public Vector3 Torque;

    public Vector3 CenterOfMass;
    public float RestitutionCoefficient;

    public BaseCollider collider;

    public float CalculateInverseMass()
    {
        return Mass == 0 ? 0 : 1.0f / Mass;
    }
}

public class PhysicsRigidBody : MonoBehaviour
{
    public enum RigidbodyState
    {
        Active,
        Sliding,
        Sleep
    }

    [SerializeField]
    private RigidbodyData rigidbodyData;

    public Vector3 InitialImpulse;
    public Vector3 InitialAngularImpulse;
    #region Properties

    public float Mass => rigidbodyData.Mass;
    public float InverseMass => rigidbodyData.InverseMass;
    public Vector3 Velocity => rigidbodyData.Velocity;
    public Vector3 AngularVelocity => rigidbodyData.AngularVelocity;
    public Vector3 Position => rigidbodyData.Position;
    public Vector3 Torque => rigidbodyData.Torque;
    public Quaternion Orientation => rigidbodyData.Orientation;
    public BaseCollider Collider => rigidbodyData.collider;
    public RigidbodyState State {get ;set; }
    public RigidbodyData Data => rigidbodyData;
    #endregion

    private void Awake()
    {
        rigidbodyData.collider = GetComponent<BaseCollider>();
        rigidbodyData.InertiaTensor = Tensor();
        rigidbodyData.InverseInertiaTensor = rigidbodyData.InertiaTensor.inverse;
        rigidbodyData.Orientation = transform.rotation;
        rigidbodyData.InverseMass = rigidbodyData.CalculateInverseMass();
        rigidbodyData.Position = transform.position;
    }

    void Start()
    {
        AddLinearImpulse(InitialImpulse);
        AddRotationalImpulse(Vector3.up, InitialAngularImpulse);
    }

    public void ApplyState(RigidStateVector state)
    {
        rigidbodyData.Position = state.Position;
        rigidbodyData.Velocity = state.Velocity();
        transform.position = rigidbodyData.Position;
    }

    public void SetPosition(Vector3 Position)
    {
        rigidbodyData.Position = Position;
        transform.position = Position;
    }

    public void TranslatePosition(Vector3 translation)
    {
       SetPosition(Position + translation);
    }

    /// <summary>
    /// Amount of energy needed to bring a moving object to rest
    /// </summary>
    /// <returns></returns>
    public Vector3 Momentum()
    {
        return Mass * Velocity;
    }

    /// <summary>
    /// The acceleration of the object (based on the last steps velocity)
    /// </summary>
    /// <returns></returns>
    public Vector3 Acceleration()
    {
        //TODO get acceleration from physics state vector
        return Velocity; //- LastVelocity;
    }

    /// <summary>
    /// How much force the current rigidbody has in respects to the force and acceleration
    /// </summary>
    /// <returns></returns>
    public Vector3 Force()
    {
        return Mass * Acceleration();
    }

    public float KineticEnergy()
    {
        return 0.5f * Mass * Vector3.Dot(Velocity, Velocity);
    }

    public void AddLinearImpulse(Vector3 impulse)
    {
        rigidbodyData.Velocity += impulse;
    }

    public void Integrate(ref RigidStateVector state, float deltaTime)
    {
        rigidbodyData.Velocity = state.Velocity();
        rigidbodyData.Position = state.Position;

        transform.position = rigidbodyData.Position;

        //TODO add angular velocity
        rigidbodyData.Orientation = Quaternion.Euler(rigidbodyData.Orientation.eulerAngles + (state.Spin().eulerAngles * deltaTime));
        transform.rotation = rigidbodyData.Orientation;

        //apply angulat velocity
        rigidbodyData.Torque = state.AngularMomentum * rigidbodyData.Mass;
        rigidbodyData.AngularMomentum = state.AngularMomentum;
        rigidbodyData.AngularVelocity = Vector3.Cross(state.AngularVelocity() * deltaTime, transform.position - rigidbodyData.CenterOfMass);

        rigidbodyData.Velocity += rigidbodyData.AngularVelocity;
    }

    public void ApplyLinearResponse(CollisionData collisionData , PhysicsRigidBody other)
    {
        Vector3 responseA;
        Vector3 responseB;

        Vector3 angularVelocity;
        if (!other)
            LinearResponse(collisionData, out responseA, out angularVelocity);
        else
        {
            //other.State = RigidbodyState.Active;
            LinearResponse(collisionData, other, out responseA, out responseB);
            if (responseB.sqrMagnitude > 0.5f)
                other.rigidbodyData.Velocity = responseB;
            else
                other.rigidbodyData.Velocity = Vector3.ProjectOnPlane(other.Velocity, collisionData.CollisionNormal);
        }

        if (responseA.sqrMagnitude > 0.5f)
            rigidbodyData.Velocity = responseA;
        else
            rigidbodyData.Velocity = Vector3.ProjectOnPlane(Velocity, collisionData.CollisionNormal);
    }

    private void LinearResponse(CollisionData collisionData, out Vector3 response, out Vector3 angularVelocity)
    {
        // Relative velocity
        Vector3 approachVelocity = Velocity;
        Vector3 V1norm = approachVelocity.normalized;
        Vector3 Vb = 2 * collisionData.CollisionNormal * Vector3.Dot(collisionData.CollisionNormal, -V1norm) + V1norm;
        Vector3 newVelocity = Vb * approachVelocity.magnitude;

        Vector3 J = (newVelocity * (rigidbodyData.RestitutionCoefficient + 1)) / ((1 / Mass));

        Vector3 V1 = (J / Mass) - newVelocity;

        angularVelocity = Vector3.Cross(AngularVelocity * Time.fixedDeltaTime,
            ((collisionData.ContactPoint - transform.position) - rigidbodyData.CenterOfMass));

        response =  V1 ;
    }

    private void LinearResponse(CollisionData collisionData, PhysicsRigidBody other, out Vector3 responseA, out Vector3 responseB)
    {
        // Relative velocity
        Vector3 r1 = collisionData.ContactPoint - Position;
        Vector3 r2 = collisionData.ContactPoint - other.Position;

        Vector3 approachVelocity = (Velocity) - (other.Velocity);

        //Restitution calculation (RestitutionCoefficient: 0 = Perfectly Inelastic, RestitutionCoefficient: 1 = Elastic)
        Vector3 J = (-approachVelocity * (rigidbodyData.RestitutionCoefficient + 1)) / ((1 / Mass) + (1 / other.Mass));

        Vector3 V1 = (J / Mass) + Velocity;
        Vector3 V2 = (-J / other.Mass) + other.Velocity;

        responseA = V1;
        responseB = V2;
    }

    public Matrix4x4 Tensor()
    {
        float ix = 0.0f;
        float iy = 0.0f;
        float iz = 0.0f;
        float iw = 0.0f;

        if (Mass > 0.0f)
        {
            float fraction;
            switch (Collider.Shape)
            {
                case BaseCollider.ColliderShape.PLANE:
                    PlaneCollider planeCollider = (PlaneCollider)Collider;
                    Vector3 size = planeCollider.GetComponent<Renderer>().bounds.size;
                    fraction = (1.0f / 12.0f);
                    float x2 = size.x * size.x;
                    float y2 = size.y * size.y;
                    float z2 = size.z * size.z;
                    ix = (y2 + z2) * Mass * fraction;
                    iy = (x2 + z2) * Mass * fraction;
                    iz = (x2 + y2) * Mass * fraction;
                    iw = 1.0f;
                    break;
                case BaseCollider.ColliderShape.SPHERE:
                    SphereCollider sphereCollider = (SphereCollider)Collider;
                    float r2 = sphereCollider.Radius * sphereCollider.Radius;
                    fraction = (2.0f / 5.0f);
                    ix = r2 * Mass * fraction;
                    iy = r2 * Mass * fraction;
                    iz = r2 * Mass * fraction;
                    iw = 1.0f;
                    break;
                default:
                    throw new ArgumentOutOfRangeException();
            }
        }

        return new Matrix4x4
        {
            m00 = ix,
            m01 = 0,
            m02 = 0,
            m03 = 0,
            m10 = 0,
            m11 = ix,
            m12 = 0,
            m13 = 0,
            m20 = 0,
            m21 = 0,
            m22 = iz,
            m23 = 0,
            m30 = 0,
            m31 = 0,
            m32 = 0,
            m33 = iw,
        };
    }

    public Matrix4x4 InverseTensor()
    {
        return Tensor().inverse;
    }

    public void AddRotationalImpulse(Vector3 point, Vector3 impulse)
    {
        rigidbodyData.Torque += Vector3.Cross(point - rigidbodyData.CenterOfMass, impulse);
        //rigidbodyData.AngularVelocity = InverseTensor().MultiplyVector(torque);
        //AngularMomentum += AngularVelocity;
    }
}
