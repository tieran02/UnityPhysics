using System.Collections;
using System.Collections.Generic;
using System.Linq;
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

    public int Size => particleSet.Positions.Count;
    public float Mass => mass;
    public float Radius => radius;

    public ParticleSystemData(int particleCount)
    {
        resize(particleCount);
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
}