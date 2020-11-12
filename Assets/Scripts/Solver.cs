using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

public abstract class Solver
{
    protected float TimeStepSeconds;
    protected float deltaTime;

    private float timeSinceLastStep;

    protected Solver(float timeStepSeconds)
    {
        TimeStepSeconds = timeStepSeconds;
    }

    public void Update(float deltaTime)
    {
        this.deltaTime = deltaTime;
        timeSinceLastStep += deltaTime;

        if (timeSinceLastStep >= TimeStepSeconds)
        {
            NextStep();
            timeSinceLastStep = 0.0f;
        }
    }

    protected abstract void NextStep();
}