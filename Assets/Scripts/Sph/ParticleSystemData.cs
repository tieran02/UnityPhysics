using System.Collections;
using System.Collections.Generic;
using System.Linq;
using System.Threading.Tasks;
using UnityEngine;

[System.Serializable]
public struct ParticleSet
{
    public List<Vector3> Positions;
    public List<Vector3> Velocities;
    public List<Vector3> Forces;
    public List<float> Densities;
}

public class ParticleSystemData
{
    public ParticleSet particleSet;

    private float radius = 1e-3f;
    private float mass = 1e-3f;

    //Target spacing of this particle system in meters.
    float targetSpacing = 0.1f;

    //Relative radius of SPH kernel.
    //SPH kernel radius divided by target spacing.
    float kernelRadiusOverTargetSpacing = 1.8f;

    float kernalRadius;

    private IPointHash neighborSearcher;
    private List<List<int>> neighborLists;

    public int Size => particleSet.Positions.Count;
    public float Mass => mass;
    public float Radius => radius;
    public float KernalRadius => kernalRadius;

    public ParticleSystemData(int particleCount)
    {
        resize(particleCount);
        neighborLists = new List<List<int>>();

        kernalRadius = kernelRadiusOverTargetSpacing * targetSpacing;
    }

    public void resize(int numberOfParticles)
    {
        particleSet = new ParticleSet
        {
            Positions = new List<Vector3>(Enumerable.Repeat(Vector3.zero,numberOfParticles)),
            Velocities = new List<Vector3>(Enumerable.Repeat(Vector3.zero, numberOfParticles)),
            Forces = new List<Vector3>(Enumerable.Repeat(Vector3.zero, numberOfParticles)),
            Densities = new List<float>(Enumerable.Repeat(0.0f, numberOfParticles))
        };
    }

    public void BuildNeighborSearcher(float maxSearchRadius)
    {
        neighborSearcher = new PointHashSearcher(Vector3Int.one * 128, 2.0f * maxSearchRadius);
        neighborSearcher.Build(particleSet.Positions);
    }

    public void BuildNeighborLists(float maxSearchRadius)
    {
        neighborLists.Resize(particleSet.Positions.Count, new List<int>());

        for (int i = 0; i < particleSet.Positions.Count; i++)
        {

            Vector3 origin = particleSet.Positions[i];
            neighborLists[i].Clear();

            neighborSearcher.ForEachNearbyPoint(origin, maxSearchRadius, (index, point) =>
            {
                neighborLists[i].Add(index);
            });
        }
    }

    public void UpdateDensities()
    {
        var p = particleSet.Positions;
        var d = particleSet.Densities;

        Parallel.For(0, particleSet.Positions.Count, (index) =>
        {
            float sum = SumOfKernelNearby(p[index]);
            d[index] = mass * sum;
        });

        //for (int index = 0; index < particleSet.Positions.Count; index++)
        //{
        //    float sum = SumOfKernelNearby(p[index]);
        //    d[index] = mass * sum;
        //}
    }

    public float SumOfKernelNearby(Vector3 origin)
    {
        float sum = 0.0f;
        SPHKernal kernal = new SPHKernal(kernalRadius);

        neighborSearcher.ForEachNearbyPoint(origin, kernalRadius, (index, neighborPosition) =>
        {
            float dist = Vector3.Distance(origin, neighborPosition);
            float weight = mass * kernal.Value(dist);
            sum += weight;
        });

        return sum;
    }

    Vector3 GradiantAt(int i, List<float> values)
    {
        Vector3 sum = Vector3.zero;
        var p = particleSet.Positions;
        var d = particleSet.Densities;
        var neighbors = neighborLists[i];
        Vector3 origin = p[i];
        SPHSpikeyKernal kernal = new SPHSpikeyKernal(KernalRadius);

        foreach (var neighbor in neighbors)
        {
            Vector3 neighborPosition = p[neighbor];
            float distance = Vector3.Distance(origin, neighborPosition);
            if(distance > 0.0f)
            {
                Vector3 direction = (neighborPosition - origin) / distance;
                sum += d[i] * mass * (values[i] / square(d[i]) + values[neighbor] / square(d[neighbor])) * kernal.Gradiant(distance, direction);
            }
        }
        return sum;
    }

    float LaplacianAt(int i, List<float> values)
    {
        float sum = 0.0f;
        var p = particleSet.Positions;
        var d = particleSet.Densities;
        var neighbors = neighborLists[i];
        Vector3 origin = p[i];
        SPHSpikeyKernal kernal = new SPHSpikeyKernal(KernalRadius);

        foreach (var neighbor in neighbors)
        {
            Vector3 neighborPosition = p[neighbor];
            float distance = Vector3.Distance(origin, neighborPosition);
            sum += mass * (values[neighbor] - values[i]) / d[neighbor] * kernal.SecondDerivative(distance);
        }
        return sum;
    }

    float square(float value)
    {
        return value * value;
    }
}