using System.Collections;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;

public class SPHSolver : MonoBehaviour
{
    public int NumberOfParticles = 1000;
    public float TargetSpacing = 0.02f;
    public PlaneCollider[] planeColliders;
    SPHSystemSolver solver;
    public Material material;
    public float MaxDensitity = 1000.0f;
    List<Material> mats;


    public Transform SpawnPos;

    List<GameObject> particles;

    void Awake()
    {
        solver = new SPHSystemSolver(NumberOfParticles, TargetSpacing);
        particles = new List<GameObject>(NumberOfParticles);
        solver.SetColliders(planeColliders);
        MaxDensitity = solver.ParticleData.TargetDensitiy;

        GameObject sphereGameobject = GameObject.CreatePrimitive(PrimitiveType.Sphere);
        sphereGameobject.transform.localScale = Vector3.one * TargetSpacing;

        mats = new List<Material>();
        for (int i = 0; i < NumberOfParticles; i++)
        {
            particles.Add(Instantiate(sphereGameobject, Vector3.zero, Quaternion.identity));
            mats.Add(Instantiate<Material>(material));
            particles[i].GetComponent<MeshRenderer>().material = mats[i];
        }
        Destroy(sphereGameobject);
    }

    // Update is called once per frame
    void FixedUpdate()
    {
        solver.Update(Time.fixedDeltaTime);

        for (int i = 0; i < particles.Count; i++)
        {
            particles[i].transform.position = solver.ParticleData.particleSet.Positions[i];
            mats[i].color = Color.Lerp(Color.white, Color.red, solver.ParticleData.particleSet.Densities[i] / MaxDensitity);
        }
    }

    private void OnDrawGizmos()
    {
        float kernalRadius = solver.ParticleData.KernalRadius;
        for (int i = 0; i < particles.Count; i++)
        {
            Vector3 pos = solver.ParticleData.particleSet.Positions[i];
            Gizmos.DrawWireSphere(pos, kernalRadius);
        }
    }
}
