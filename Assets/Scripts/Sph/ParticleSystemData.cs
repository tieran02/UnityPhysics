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
        neighborSearcher = new PointHashSearcher(Vector3Int.one * 64, 2.0f * maxSearchRadius);
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
}