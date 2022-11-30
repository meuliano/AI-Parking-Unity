using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Sensors;
using UnityStandardAssets.Vehicles.Car;
using static UnityEngine.GraphicsBuffer;

public class MoveToGoalAgent : Agent
{
    [SerializeField] private Material winMaterial;
    [SerializeField] private Material loseMaterial;
    [SerializeField] private MeshRenderer floorMeshRenderer;
    [SerializeField] private bool particles = false;
    [SerializeField] private GameObject Explosion;

    private SimpleCarController carController;
    EnvironmentParameters defaultParameters;
    private Rigidbody rb;

    public float spawnRadiusX = 2f;
    public float spawnRadiusZ = 2f;

    private int steps = 0;
    private int goalsCollided = 0;
    private bool inTarget = false;

    private Vector3 startPosition;
    private Quaternion startRotation;
    private Vector3 startPositionTarget;
    private Quaternion startRotationTarget;
    private Vector3 lastPosition;

    private void FixedUpdate()
    {
        RequestDecision();
    }

    public override void OnEpisodeBegin()
    {
        // Spawn randomly in defined range
        float spawnX = Random.Range(startPosition.x - spawnRadiusX, startPosition.x + spawnRadiusX);
        float spawnZ = Random.Range(startPosition.z - spawnRadiusZ, startPosition.z + spawnRadiusZ);
        Vector3 spawnPosition = new Vector3(spawnX, startPosition.y, spawnZ);

        rb.transform.localPosition = spawnPosition;
        rb.transform.rotation = startRotation;
        rb.velocity = Vector3.zero;
        rb.angularVelocity = Vector3.zero;
        steps = 0;
    }
    public override void Initialize()
    {
        carController = GetComponent<SimpleCarController>();
        rb = GetComponent<Rigidbody>();

        defaultParameters = Academy.Instance.EnvironmentParameters;

        startPosition = transform.localPosition;
        startRotation = transform.rotation;

        lastPosition = startPosition;

        OnEpisodeBegin();
    }


    public override void CollectObservations(VectorSensor sensor)
    {
        sensor.AddObservation(transform.localPosition.x);
        sensor.AddObservation(transform.localPosition.z);
        sensor.AddObservation(transform.rotation.y);
        sensor.AddObservation(transform.InverseTransformDirection(rb.velocity).z);
    }
    public override void OnActionReceived(ActionBuffers actions)
    {
        float steering = actions.ContinuousActions[0];
        float accel = actions.ContinuousActions[1];

        carController.Move(steering, accel, 0f, 0f);
        steps++;

        if (goalsCollided > 0) { floorMeshRenderer.material = winMaterial; }
        else { floorMeshRenderer.material = loseMaterial;}

        if (goalsCollided == 2)
        {
            floorMeshRenderer.material = winMaterial;
            if (rb.velocity.magnitude < 0.2f)
            {
                AddReward(+100f);
                Debug.Log("Successful Park"+ GetCumulativeReward());
                EndEpisode();
            }
        }

        if (steps == MaxStep)
        {
            AddReward(-5f);
            Debug.Log("Failed To Reach"+GetCumulativeReward());
            floorMeshRenderer.material = loseMaterial;
        }
        AddReward(-0.05f); //Negative reward per time step
    }

    public override void Heuristic(in ActionBuffers actionsOut)
    {
        ActionSegment<float> continuousActionsOut = actionsOut.ContinuousActions;

        float steering = Input.GetAxis("Horizontal");
        float accel = Input.GetAxis("Vertical");

        continuousActionsOut[0] = steering;
        continuousActionsOut[1] = accel;
    }

    private void OnTriggerEnter(Collider other)
    {
        // Collide with Goal
        if (other.TryGetComponent<GoalCollide>(out GoalCollide goal))
        {
            inTarget = true;
            goalsCollided += 1;
        }
        //Collide with Wall
        if (other.TryGetComponent<WallCollide>(out WallCollide wall))
        {
            AddReward(-10f);
            if(particles == true) { Explode(); }
            //Debug.Log("Wall Collision"+GetCumulativeReward());
            EndEpisode();
        }
        //Collide with Car
        if (other.tag == "Car")
        {
            AddReward(-10f);
            if (particles == true) { Explode(); }
            //Debug.Log("Car Collision" + GetCumulativeReward());
            EndEpisode();
        }
    }
 
    void OnTriggerExit(Collider other)
    {
        // Exit goal collider
        if (other.TryGetComponent<GoalCollide>(out GoalCollide goal))
        {
            inTarget = false;
            goalsCollided -= 1;
        }
    }

    
    //public GameObject Explosion;
    void Explode()
    {
        GameObject explosion = Instantiate(Explosion, rb.position, Quaternion.identity);
        explosion.transform.localScale = new Vector3Int(3, 3, 3);
        explosion.GetComponent<ParticleSystem>().Play();
    }

}


