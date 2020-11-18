using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class StopSleep : MonoBehaviour
{
    // Start is called before the first frame update
    void Awake()
    {
        var m_Rigidbody = GetComponent<Rigidbody>();
        m_Rigidbody.sleepThreshold = 0.0f;
    }

    // Update is called once per frame
    void Update()
    {
        
    }
}
