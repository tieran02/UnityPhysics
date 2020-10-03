using UnityEngine;

public class TrajectorySolver : MonoBehaviour
{
    public Vector3 InitialVelocity;
    public SphereCollider SphereCollider;
    public PlaneCollider PlaneCollider;

    private Vector3 GRAVITY_FORCE = new Vector3(0.0f, -9.8f,0.0f);
    private Vector3 StartPosition;

    private Vector3 velocity;
    private Vector3 position;

    private const float TIME_STEP = 1 / 60.0f;
    private float accumulator;
    private float lastTime;

    void Awake()
    {
        SphereCollider = GetComponent<SphereCollider>();
    }

    // Start is called before the first frame update
    void Start()
    {
        StartPosition = transform.position;
        position = StartPosition;
        velocity = InitialVelocity;
    }

    // Update is called once per frame
    void Update()
    {
        accumulator += Time.deltaTime;

        //update 60 times per second (this limits the physic time step)
        //to make the movement look smoother we can always use linear interpolation
        while (accumulator > TIME_STEP)
        {
            float deltaTime = Time.time - lastTime;
            if (deltaTime <= 0.0f) deltaTime = TIME_STEP;
            lastTime = Time.time;

            Vector3 oldPos = transform.position;
            Solver(deltaTime);

            if (SphereCollider.Intersect(PlaneCollider))
            {
                velocity = Vector3.zero;
                transform.position = oldPos;
            }

            accumulator -= TIME_STEP;
        }
    }

    void Solver(float deltaTime)
    {
        velocity += (GRAVITY_FORCE * deltaTime);
        position += (velocity * deltaTime);
        transform.position = position;
    }
}
