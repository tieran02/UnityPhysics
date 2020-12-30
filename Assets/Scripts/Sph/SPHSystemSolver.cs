using System.Collections;
using System.Collections.Generic;
using System.Linq;
using System.Threading.Tasks;
using UnityEngine;

public class SPHSystemSolver : Solver
{
    private readonly ParticleSystemData particleData;

    private const float dragCoefficient = 2e-4f;
    private readonly Vector3 GRAVITY_FORCE = new Vector3(0.0f, -9.8f, 0.0f);
    private const float speedOfSound = 1000.0f;
    //private const float speedOfSound = 500.0f;
    // Exponent component of equation-of-state (or Tait's equation).
    private const float eosExponent = 7.0f;
    private const float viscosityCoefficient = 0.0074f;
    private const float pseudoViscosityCoefficient = 20.0f;

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

        const int perRow = 64;
        float spacing = targetSpacing * 2.0f;
        for (int i = 0; i < numberOfParticles; i++)
        {
            float x = ((i % perRow) * spacing) + Random.Range(-spacing, spacing);
            float y = (20 * spacing) + (float)(((i / perRow) / perRow) * spacing) * 1.1f;
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
    }

    void resolveCollisions()
    {
        float mass = particleData.Mass;
        const float RestitutionCoefficient = 0.1f;
        const float frictionCoeffient = float.Epsilon;

        Parallel.For(0, particleData.Size, index =>
        {
            Vector3 newPosition = newPositions[index];
            Vector3 newVelocity = newVelocities[index];

            foreach (var collider in colliders)
            {
                CollisionData collisionData;
                if(collider.CollisionOccured(newPosition, newVelocity, deltaTime, out collisionData))
                {
                    ////Relative velocity
                    //Vector3 approachVelocity = newVelocity;
                    //Vector3 V1norm = approachVelocity.normalized;
                    //Vector3 Vb = 2 * collisionData.CollisionNormal * Vector3.Dot(collisionData.CollisionNormal, -V1norm) + V1norm;

                    //Vector3 relativeVelN = Vb * approachVelocity.magnitude;
                    //Vector3 relativeVelT = approachVelocity - relativeVelN;

                    //Vector3 actualVelocity = relativeVelN + relativeVelT;

                    //Vector3 J = (actualVelocity * (RestitutionCoefficient + 1)) / ((1 / mass));

                    //Vector3 V1 = (J / mass) - actualVelocity;

                    //newVelocities[index] = V1 * mass;
                    //newPositions[index] += V1norm * collisionData.VC;

                    Vector3 targetNormal = collisionData.CollisionNormal;
                    //Vector3 targetPoint = newPosition + newVelocity.normalized * collisionData.VC;
                    //here we can have the colliders velocity, for now all colliders are static, thus have no velocity
                    Vector3 colliderVelocityAtTargetPoint = Vector3.zero;

                    Vector3 relativeVelocity = newVelocity - colliderVelocityAtTargetPoint;
                    float normalDotRelativeVelocity = Vector3.Dot(targetNormal, relativeVelocity);
                    Vector3 relativeVelocityNormal = normalDotRelativeVelocity * targetNormal;
                    Vector3 relativeVelocityT = relativeVelocity - relativeVelocityNormal;

                    // Check if the velocity is facing opposite direction of the surface
                    // normal
                    if (normalDotRelativeVelocity < 0.0)
                    {
                        //Apply restitution coefficient to the surface normal component of the velocity
                        Vector3 deltaRelativeVelocityNormal = (-RestitutionCoefficient - 1.0f) * relativeVelocityNormal;
                        relativeVelocityNormal *= -RestitutionCoefficient;

                        // Apply friction to the tangential component of the velocity
                        // http://graphics.stanford.edu/papers/cloth-sig02/cloth.pdf
                        if (relativeVelocityT.sqrMagnitude > 0.0f)
                        {
                            float frictionScale = Mathf.Max(1.0f - frictionCoeffient *
                                deltaRelativeVelocityNormal.magnitude / relativeVelocityT.magnitude, 0.0f);
                            relativeVelocityT *= frictionScale;
                        }

                        relativeVelocityT = Vector3.ProjectOnPlane(relativeVelocityT, targetNormal);

                        // Reassemble the components
                        newVelocities[index] = relativeVelocityNormal + relativeVelocityT + colliderVelocityAtTargetPoint;
                    }

                    newPositions[index] = collisionData.ContactPoint + (newVelocities[index] * deltaTime);
                    //newPositions[index] += newVelocity.normalized * (collisionData.VC - particleData.KernalRadius);
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
            Vector3 force = mass * GRAVITY_FORCE * deltaTime;

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
        var particles = particleData;
        int numberOfParticles = particleData.Size;
        var x = particles.particleSet.Positions;
        var d = particles.particleSet.Densities;
        var v = particles.particleSet.Velocities;

        float mass = particles.Mass;
        SPHSpikeyKernal kernel = new SPHSpikeyKernal(particles.KernalRadius);

        Vector3[] smoothedVelocities = new Vector3[numberOfParticles];

        Parallel.For(0, numberOfParticles, (index) =>
        {
            float weightSum = 0.0f;
            Vector3 smoothedVelocity = Vector3.zero;

            var neighbors = particles.Neighbors[index];
            foreach (var neighbor in neighbors)
            {
                float distance = Vector3.Distance(x[index], x[neighbor]);
                float wj = mass / d[neighbor] * kernel.Value(distance);
                weightSum += wj;
                smoothedVelocity += wj * v[neighbor];
            }

            float wi = mass / d[index];
            weightSum += wi;
            smoothedVelocity += wi * v[index];

            if (weightSum > 0.0)
            {
                smoothedVelocity /= weightSum;
            }

            smoothedVelocities[index] = smoothedVelocity;
        });

        float factor = deltaTime * pseudoViscosityCoefficient;
        factor = Mathf.Clamp(factor, 0.0f, 1.0f);
        Parallel.For(0, numberOfParticles, (index) =>
        {
            v[index] = Vector3.Lerp(v[index], smoothedVelocities[index], factor);
        });
    }

    float computePressure(float density, float targetDensity, float eosScale, float eosExponent, float negativePressureScale)
    {
        float p = eosScale / eosExponent
            * (Mathf.Pow((density / targetDensity), eosExponent) - 1.0f);

        //float p = 10.0f * (density - targetDensity);

        //negative pressue scaling
        if (p < 0)
            p *= negativePressureScale;
        return p;
    }
}
