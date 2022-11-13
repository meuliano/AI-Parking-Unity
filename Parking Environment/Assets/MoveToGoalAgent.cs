using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Sensors;
using UnityStandardAssets.Vehicles.Car;

public class MoveToGoalAgent : Agent
{

    [SerializeField] private Transform targetTransform;
    [SerializeField] private Material winMaterial;
    [SerializeField] private Material loseMaterial;
    [SerializeField] private MeshRenderer floorMeshRenderer;

    private CarController carController;
    EnvironmentParameters defaultParameters;
    private Rigidbody rb;

    public float spawnRadiusX = 2f;
    public float spawnRadiusZ = 2f;
    public float spawnRadiusTargetX = 2f;
    public float spawnRadiusTargetZ = 2f;

    private int steps = 0;

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
        // Spawn randomly target position
        
        float spawnXtarget = Random.Range(startPositionTarget.x - spawnRadiusTargetX, startPositionTarget.x + spawnRadiusTargetX);
        float spawnZtarget = Random.Range(startPositionTarget.z - spawnRadiusTargetZ, startPositionTarget.z + spawnRadiusTargetZ);
        targetTransform.localPosition = new Vector3(spawnXtarget, startPositionTarget.y, spawnZtarget);

        // Spawn randomly in defined range
        float spawnX = Random.Range(startPosition.x - spawnRadiusX, startPosition.x + spawnRadiusX);
        float spawnZ = Random.Range(startPosition.z - spawnRadiusZ, startPosition.z + spawnRadiusZ);
        Vector3 spawnPosition = new Vector3(spawnX, startPosition.y, spawnZ);

        rb.transform.localPosition = spawnPosition;
        rb.transform.localRotation = startRotation;
        rb.velocity = Vector3.zero;
        rb.angularVelocity = Vector3.zero;

        steps = 0;
    }
    public override void Initialize()
    {
        carController = GetComponent<CarController>();
        rb = GetComponent<Rigidbody>();

        defaultParameters = Academy.Instance.EnvironmentParameters;

        startPosition = transform.localPosition;
        startRotation = transform.localRotation;

        startPositionTarget = targetTransform.localPosition;
        startRotationTarget = targetTransform.localRotation;

        lastPosition = startPosition;

        OnEpisodeBegin();
    }


    public override void CollectObservations(VectorSensor sensor)
    {
        sensor.AddObservation(carController.CurrentSpeed);
        sensor.AddObservation(transform.localPosition);
        sensor.AddObservation(targetTransform.localPosition);
    }
    public override void OnActionReceived(ActionBuffers actions)
    {
        float steering = actions.ContinuousActions[0];
        float accel = actions.ContinuousActions[1];
        float reverse = actions.ContinuousActions[2];

        // Input is from -1 to 1, map values accordingly
        accel = (accel + 1) / 2;
        reverse = (reverse + 1) / 2;
        accel = accel - reverse;

        carController.Move(steering, accel, 0f, 0f);

        steps++;
    }

    public override void Heuristic(in ActionBuffers actionsOut)
    {
        ActionSegment<float> continuousActionsOut = actionsOut.ContinuousActions;

        float steering = Input.GetAxis("Horizontal"); //-1 to 1
        float accel = Input.GetAxis("Accelerate");  //0 to 1
        float reverse = Input.GetAxis("Reverse");   //0 to 1

        // Input from network is between -1 to 1, map values accordingly
        accel = accel * 2 - 1;
        reverse = reverse * 2 - 1;

        continuousActionsOut[0] = steering;
        continuousActionsOut[1] = accel;
        continuousActionsOut[2] = reverse;
    }

    private void OnTriggerEnter(Collider other)
    {
        if (other.TryGetComponent<GoalCollide>(out GoalCollide goal))
        {
            SetReward(+10f);
            floorMeshRenderer.material = winMaterial;
            EndEpisode();
        }
        if (other.TryGetComponent<WallCollide>(out WallCollide wall))
        {
            SetReward(-1f);
            floorMeshRenderer.material = loseMaterial;
            EndEpisode();
        }
    }
}
