using System.Collections;
using System.Collections.Generic;
using System.Linq;
using System.Threading.Tasks;
using UnityEngine;

public class SPHSystemSolver : Solver
{
    private readonly ParticleSystemData particleData;

    private const float dragCoefficient = 1e-4f;
    private readonly Vector3 GRAVITY_FORCE = new Vector3(0.0f, -9.8f, 0.0f);

    //current state of positions and velocites
    private List<Vector3> newPositions;
    private List<Vector3> newVelocities;

    private BaseCollider[] colliders;

    public ParticleSystemData ParticleData => particleData;

    public SPHSystemSolver(int numberOfParticles) : base(1.0f / 60.0f)
    {
        particleData = new ParticleSystemData(numberOfParticles);
        newPositions = new List<Vector3>(Enumerable.Repeat(Vector3.zero, numberOfParticles));
        newVelocities = new List<Vector3>(Enumerable.Repeat(Vector3.zero, numberOfParticles));

        for (int i = 0; i < numberOfParticles; i++)
        {
            particleData.particleSet.Positions[i] = new Vector3(i*3.0f, 0, 0);
        }
    }

    protected override void NextStep()
    {
        beginTimeStep();

        accumulateForces(deltaTime);

        timeIntegration(deltaTime);

        resolveCollisions();

        endTimeStep();
    }

    public void SetColliders(BaseCollider[] colliders)
    {
        this.colliders = colliders;
    }

    void accumulateForces(float deltaTime)
    {
        accumlateExternalForces(deltaTime);
    }

    void resolveCollisions()
    {
        float mass = particleData.Mass;
        const float RestitutionCoefficient = 0.6f;

        Parallel.For(0, particleData.Size, index =>
        {
            Vector3 position = newPositions[index];
            Vector3 velocity = newVelocities[index];

            foreach (var collider in colliders)
            {
                CollisionData collisionData;
                if(collider.CollisionOccured(position, velocity, deltaTime, out collisionData))
                {
                    // Relative velocity
                    Vector3 approachVelocity = velocity;
                    Vector3 V1norm = approachVelocity.normalized;
                    Vector3 Vb = 2 * collisionData.CollisionNormal * Vector3.Dot(collisionData.CollisionNormal, -V1norm) + V1norm;
                    Vector3 newVelocity = Vb * approachVelocity.magnitude;

                    Vector3 J = (newVelocity * (RestitutionCoefficient + 1)) / ((1 / mass));

                    Vector3 V1 = (J / mass) - newVelocity;

                    newVelocities[index] = V1 * mass;
                    newPositions[index] += V1norm * collisionData.VC;
                }
            }
        });
    }

    private void beginTimeStep()
    {
        particleData.BuildNeighborSearcher(1.0f);
        particleData.BuildNeighborLists(1.0f);
        particleData.UpdateDensities();

        //reset forces, pos, velocities and densities in parallel
        Parallel.For(0, particleData.Size, (index, loopState) =>
        {
            particleData.particleSet.Forces[index] = Vector3.zero;
            newPositions[index] = Vector3.zero;
            newVelocities[index] = Vector3.zero;
            particleData.particleSet.Densities[index] = 0.0f;
        });

        //TODO: update collider position
    }

    private void endTimeStep()
    {
        //update data

        Parallel.For(0,particleData.Size, index => 
        {
            particleData.particleSet.Positions[index] = newPositions[index];
            particleData.particleSet.Velocities[index] = newVelocities[index];
        });
    }

    private void timeIntegration(float deltaTime)
    {
        var forces = particleData.particleSet.Forces;
        var velocities = particleData.particleSet.Velocities;
        var positions = particleData.particleSet.Positions;

        Parallel.For(0, particleData.Size, index =>
        {
            //integrate velocity
            Vector3 newVelocity = newVelocities[index];
            newVelocity = velocities[index] + deltaTime * forces[index] / particleData.Mass; //todo inverse mass
            newVelocities[index] = newVelocity;

            //integrate position
            Vector3 newPosition = newPositions[index];
            newPosition = positions[index] + deltaTime * newVelocity;
            newPositions[index] = newPosition;
        });
    }

    private void accumlateExternalForces(float deltaTime)
    {
        var forces = particleData.particleSet.Forces;
        var velocities = particleData.particleSet.Velocities;
        var positions = particleData.particleSet.Positions;
        var mass = particleData.Mass;

        Parallel.For(0, particleData.Size, index =>
        {
            Vector3 force = mass * GRAVITY_FORCE;

            //TODO here we can add other forces such as wind and drag
            Vector3 relativeVel = velocities[index];
            force += -dragCoefficient * relativeVel;

            forces[index] += force;
        });
    }
}
