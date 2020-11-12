using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using UnityEngine;

public class PhysicsSolver : MonoBehaviour
{
    public EulerSolver eulerSolver;
    public PhysicsRigidBody[] RigidBodies;
    public BaseCollider[] BaseColliders;

    void Awake()
    {
        RigidBodies = FindObjectsOfType<PhysicsRigidBody>();
        BaseColliders = FindObjectsOfType<BaseCollider>();
        eulerSolver = new EulerSolver(RigidBodies, BaseColliders);
    }

    void FixedUpdate()
    {
        eulerSolver.Update(Time.fixedDeltaTime);
    }
}
