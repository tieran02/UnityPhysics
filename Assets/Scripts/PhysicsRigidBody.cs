using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using UnityEngine;

public class PhysicsRigidBody : MonoBehaviour
{
    public enum RigidbodyState
    {
        Active,
        Sleep
    }

    public Vector3 Velocity;
    public Vector3 InitialImpulse;
    public Vector3 InitialAngularImpulse;

    public Vector3 AngularVelocity;
    public Vector3 Torques;

    public float Mass = 1.0f;
    public Vector3 CenterOfMass;

    //RestitutionCoefficient: 0 = Perfectly Inelastic, RestitutionCoefficient: 1 = Elastic
    public float RestitutionCoefficient = 0.6f;

    private float potentialSleepTime;


    #region Properties
    public Vector3 Position => transform.position;
    public Vector3 Orientation => transform.eulerAngles;
    public Vector3 LastPosition { get; private set; }
    public Vector3 LastVelocity { get; private set; }
    public Vector3 ExternalForces { get; set; }
    public RigidbodyState State { get; private set; }
    public BaseCollider Collider { get; private set; }
    #endregion

    private void Awake()
    {
        Collider = GetComponent<BaseCollider>();
    }

    void Start()
    {
        AddLinearImpulse(InitialImpulse);
        RotationalImpulse(Vector3.up, InitialAngularImpulse);
        LastPosition = Position;
    }

    void FixedUpdate()
    {
        float kineticEnergy = KineticEnergy();
        if (kineticEnergy < 0.001f)
        {
            potentialSleepTime += Time.fixedDeltaTime;

            if (potentialSleepTime > 5.0f && State == RigidbodyState.Active)
            {
                State = RigidbodyState.Sleep;
                potentialSleepTime = 0.0f;
            }
        }
    }

    public void SetPosition(Vector3 Position)
    {
        LastVelocity = Velocity;
        LastPosition = transform.position;
        transform.position = Position;
    }

    public void TranslatePosition(Vector3 translation)
    {
        LastVelocity = Velocity;
        LastPosition = transform.position;
        transform.position += translation;
    }

    public void TranslateOrientation(Vector3 orientation)
    {
        transform.eulerAngles += orientation;
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
        return Velocity - LastVelocity;
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
        Velocity += impulse;
    }

    public void ApplyLinearResponse(PhysicsRigidBody other)
    {
        // Relative velocity
        Vector3 approachVelocity = Velocity - other.Velocity;

        //Restitution calculation (RestitutionCoefficient: 0 = Perfectly Inelastic, RestitutionCoefficient: 1 = Elastic)
        Vector3 J = (-approachVelocity * (RestitutionCoefficient + 1)) / ((1 / Mass) + (1 / other.Mass));

        Vector3 V1 = (J / Mass) + Velocity;
        Vector3 V2 = (-J / other.Mass) + other.Velocity;

        Velocity = V1;
        other.Velocity = V2;
    }

    public void ApplyLinearResponse(CollisionData collisionData)
    {
        // Relative velocity
        Vector3 approachVelocity = Velocity;
        Vector3 V1norm = approachVelocity.normalized;
        Vector3 Vb = 2 * collisionData.CollisionNormal * Vector3.Dot(collisionData.CollisionNormal, -V1norm) + V1norm;
        Vector3 newVelocity = Vb * approachVelocity.magnitude;

        Vector3 J = (newVelocity * (RestitutionCoefficient + 1)) / ((1 / Mass));

        Vector3 V1 = (J / Mass) - newVelocity;

        Vector3 ang = Vector3.Cross(AngularVelocity * Time.fixedDeltaTime, (collisionData.CollisionPoint - CenterOfMass));
        Velocity = V1 + ang ;

        //angular velocity

    }

    public Matrix4x4 InverseTensor()
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
                    PlaneCollider planeCollider = (PlaneCollider) Collider;
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
                    SphereCollider sphereCollider = (SphereCollider) Collider;
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
            m00 = ix, m01 = 0, m02 = 0, m03 = 0,
            m10 = 0, m11 = ix, m12 = 0, m13 = 0,
            m20 = 0, m21 = 0, m22 = iz, m23 = 0,
            m30 = 0, m31 = 0, m32 = 0, m33 = iw,
        }.inverse;
    }

    public void RotationalImpulse(Vector3 point, Vector3 impulse)
    {
        Vector3 torque = Vector3.Cross(point - CenterOfMass, impulse);
        Vector3 angularAcceleration = InverseTensor().MultiplyVector(torque);
        AngularVelocity += angularAcceleration;
    }
}
