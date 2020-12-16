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
    //private const float speedOfSound = 100.0f;
    private const float speedOfSound = 5.0f;
    // Exponent component of equation-of-state (or Tait's equation).
    private const float eosExponent = 7.0f;
    private const float viscosityCoefficient = 0.1f;

    //current state of positions and velocites
    private List<Vector3> newPositions;
    private List<Vector3> newVelocities;

    private BaseCollider[] colliders;

    public ParticleSystemData ParticleData => particleData;

    public SPHSystemSolver(int numberOfParticles, float targetSpacing) : base(1.0f / 60.0f)
    {
        particleData = new ParticleSystemData(numberOfParticles, targetSpacing);

        newPositions = new List<Vector3>(Enumerable.Repeat(Vector3.zero, numberOfParticles));
        newVelocities = new List<Vector3>(Enumerable.Repeat(Vector3.zero, numberOfParticles));

        const int perRow = 10;
        float spacing = targetSpacing * 2.0f;
        for (int i = 0; i < numberOfParticles; i++)
        {
            float x = ((i % perRow) * spacing) + Random.Range(-spacing, spacing);
            float y = (2 * spacing) + (float)(((i / perRow) / perRow) * spacing) * 1.1f;
            float z = (((i /perRow) % perRow) * spacing) + Random.Range(-spacing, spacing);
            particleData.particleSet.Positions[i] = new Vector3(x,y,z);
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
        accumlateNonPressureForces(deltaTime);
        accumlatePressureForces(deltaTime);

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

                    Vector3 relativeVelN = Vb * approachVelocity.magnitude;
                    Vector3 relativeVelT = approachVelocity - relativeVelN;

                    Vector3 newVelocity = relativeVelN + relativeVelT;

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
        Parallel.For(0, particleData.Size, (index, loopState) =>
        {
            particleData.particleSet.Forces[index] = Vector3.zero;
        });


        particleData.BuildNeighborSearcher(particleData.KernalRadius);
        particleData.BuildNeighborLists(particleData.KernalRadius);
        particleData.UpdateDensities();

        //reset forces, pos, velocities and densities in parallel


        //TODO: update collider position
    }

    private void endTimeStep()
    {
        Parallel.For(0, particleData.Size, index =>
        {
            particleData.particleSet.Positions[index] = newPositions[index];
            particleData.particleSet.Velocities[index] = newVelocities[index];
        });

        //update data
        computePseudoViscosity();

        var particles = particleData.particleSet;
        var x = particles.Positions;
        var d = particles.Densities;
        var p = particles.Pressures;
        var f = particles.Forces;

        float maxDensity = 0.0f;
        for (int i = 0; i < particleData.Size; ++i)
        {
            maxDensity = Mathf.Max(maxDensity, d[i]);
        }
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

    private void accumlateNonPressureForces(float deltaTime)
    {
        accumlateExternalForces(deltaTime);
        accumulateViscosityForce();
    }

    private void accumlatePressureForces(float deltaTime)
    {
        var particles = particleData.particleSet;
        var x = particles.Positions;
        var d = particles.Densities;
        var p = particles.Pressures;
        var f = particles.Forces;

        computePressure();
        accumlatePressureForce(x, d, p, f);
    }

    private void accumlatePressureForce(List<Vector3> positions, List<float> densities, List<float> pressures, List<Vector3> pressureForces)
    {
        var particles = particleData;
        int numberOfParticles = particles.Size;

        float massSquared = particles.Mass * particles.Mass;
        SPHSpikeyKernal kernal = new SPHSpikeyKernal(particles.KernalRadius);

        Parallel.For(0, numberOfParticles, (index) =>
        {
        var neigbors = particles.Neighbors[index];
        foreach (var neighbor in neigbors)
        {
            float distance = Vector3.Distance(positions[index], positions[neighbor]);

            if (distance > 0.0f)
            {
                Vector3 direction = (positions[neighbor] - positions[index]) / distance;
                    Vector3 force = massSquared
                        * (pressures[index] / (densities[index] * densities[index])
                        + pressures[neighbor] / (densities[neighbor] * densities[neighbor]))
                        * kernal.Gradiant(distance, direction);
                pressureForces[index] -= force;
                }
            }
        });
    }

    private void computePressure()
    {
        var particles = particleData.particleSet;
        var numberOfParticles = particleData.Size;
        var d = particles.Densities;
        var p = particles.Pressures;

        float targetDensity = particleData.TargetDensitiy;
        float eosScale = targetDensity * (speedOfSound * speedOfSound);

        Parallel.For(0, numberOfParticles, (index) =>
        {
            p[index] = computePressure(d[index], targetDensity, eosScale, eosExponent, 0.0f);
            //p[index] = 50.0f * (d[index] - 82.0f);
        });
    }

    private void accumulateViscosityForce()
    {
        var particles = particleData;
        int numberOfParticles = particleData.Size;
        var x = particles.particleSet.Positions;
        var d = particles.particleSet.Densities;
        var v = particles.particleSet.Velocities;
        var f = particles.particleSet.Forces;

        float massSquared = particles.Mass * particles.Mass;
        SPHSpikeyKernal kernal = new SPHSpikeyKernal(particles.KernalRadius);

        Parallel.For(0, numberOfParticles, (index) =>
        {
            var neighbors = particles.Neighbors[index];
            foreach (var neighbor in neighbors)
            {
                float distance = Vector3.Distance(x[index], x[neighbor]);

                f[index] += viscosityCoefficient * massSquared
                    * (v[neighbor] - v[index]) / d[neighbor]
                    * kernal.SecondDerivative(distance);
            }
        });
    }

    private void computePseudoViscosity()
    {

    }

    float computePressure(float density, float targetDensity, float eosScale, float eosExponent, float negativePressureScale)
    {
        float p = eosScale / eosExponent
            * (Mathf.Pow((density / targetDensity), eosExponent) - 1.0f);

        //negative pressue scaling
        if (p < 0)
            p *= negativePressureScale;
        return p;
    }
}
