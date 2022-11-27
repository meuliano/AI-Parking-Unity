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

    [SerializeField] private Transform targetTransform;
    [SerializeField] private Material winMaterial;
    [SerializeField] private Material loseMaterial;
    [SerializeField] private MeshRenderer floorMeshRenderer;

    private SimpleCarController carController;
    EnvironmentParameters defaultParameters;
    private Rigidbody rb;

    public float spawnRadiusX = 2f;
    public float spawnRadiusZ = 2f;
    public float spawnRadiusTargetX = 2f;
    public float spawnRadiusTargetZ = 2f;

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
        // Spawn randomly target position
        
        float spawnXtarget = Random.Range(startPositionTarget.x - spawnRadiusTargetX, startPositionTarget.x + spawnRadiusTargetX);
        float spawnZtarget = Random.Range(startPositionTarget.z - spawnRadiusTargetZ, startPositionTarget.z + spawnRadiusTargetZ);
        targetTransform.localPosition = new Vector3(spawnXtarget, startPositionTarget.y, spawnZtarget);

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

        startPositionTarget = targetTransform.localPosition;
        startRotationTarget = targetTransform.rotation;

        lastPosition = startPosition;

        OnEpisodeBegin();
    }


    public override void CollectObservations(VectorSensor sensor)
    {
        //sensor.AddObservation(carController.CurrentSpeed);
        sensor.AddObservation(transform.localPosition.x);
        sensor.AddObservation(transform.localPosition.z);
        sensor.AddObservation(transform.rotation.y);
        //sensor.AddObservation(targetTransform.localPosition);
        //sensor.AddObservation(targetTransform.rotation);
        sensor.AddObservation(transform.InverseTransformDirection(rb.velocity).z);
    }
    public override void OnActionReceived(ActionBuffers actions)
    {
        float steering = actions.ContinuousActions[0];
        float accel = actions.ContinuousActions[1];
        //float reverse = actions.ContinuousActions[2];

        // Input is from -1 to 1, map values accordingly
        //accel = (accel + 1) / 2;
        //reverse = (reverse + 1) / 2;
        //accel = accel - reverse;

        carController.Move(steering, accel, 0f, 0f);
        //var localVelocity = transform.InverseTransformDirection(rb.velocity).z;

        //Debug.Log("A"+transform.localPosition.x +" B"+ transform.localPosition.z + " C"+transform.rotation.y);

        steps++;

        if (goalsCollided > 0) { floorMeshRenderer.material = winMaterial; }
        else { floorMeshRenderer.material = loseMaterial;}

        if (goalsCollided == 2)
        {
            floorMeshRenderer.material = winMaterial;
            if (rb.velocity.magnitude < 0.2f)
            {
                AddReward(+100f);
                //Debug.Log(GetCumulativeReward());
                Debug.Log("Successful Park");
                EndEpisode();
            }
        }
        //float reward = CalculateReward();
        //AddReward(reward);

        if (steps == MaxStep)
        {
            AddReward(-5f);
            floorMeshRenderer.material = loseMaterial;
        }
        AddReward(-0.05f);
    }

    public override void Heuristic(in ActionBuffers actionsOut)
    {
        ActionSegment<float> continuousActionsOut = actionsOut.ContinuousActions;

        float steering = Input.GetAxis("Horizontal"); //-1 to 1
        float accel = Input.GetAxis("Vertical");  //0 to 1
        //float reverse = Input.GetAxis("Reverse");   //0 to 1

        // Input from network is between -1 to 1, map values accordingly
        //accel = accel * 2 - 1;
        //reverse = reverse * 2 - 1;

        continuousActionsOut[0] = steering;
        continuousActionsOut[1] = accel;
        //continuousActionsOut[2] = reverse;
    }

    private void OnTriggerEnter(Collider other)
    {
        if (other.TryGetComponent<GoalCollide>(out GoalCollide goal))
        {
            //AddReward(5f);
            inTarget = true;
            goalsCollided += 1;
            //Debug.Log(GetCumulativeReward());

            //EndEpisode();
        }
        if (other.TryGetComponent<WallCollide>(out WallCollide wall))
        {
            AddReward(-10f);
            //Debug.Log(GetCumulativeReward());
            EndEpisode();
        }
        if (other.tag == "Car")
        {
            AddReward(-10f);
            //Debug.Log(GetCumulativeReward());
            EndEpisode();
        }
    }
    private void OnCollisionEnter(Collision other)
    {
        AddReward(-10f);
        Debug.Log("Ouchie");
        
    }

    void OnTriggerExit(Collider other)
    {
        if (other.TryGetComponent<GoalCollide>(out GoalCollide goal))
        {
            inTarget = false;
            goalsCollided -= 1;
        }
    }

    /*
    private float CalculateReward()
    {

        // Compare the difference of the previous distance to target to the current one
        // If the agent got closer, reward it. Else penalize it.
        float reward = 0f;

        float totDirectionChangeReward = 0f;
        float totAngleChangeReward = 0f;
        float totDistanceReward = 0f;

        if (lastPosition != Vector3.zero)
        {
            float distanceToTargetX = Mathf.Abs(transform.position.x - targetTransform.position.x);
            float distanceToTargetZ = Mathf.Abs(transform.position.z - targetTransform.position.z);

            float lastDistanceToTargetX = Mathf.Abs(lastPosition.x - targetTransform.position.x);
            float lastDistanceToTargetZ = Mathf.Abs(lastPosition.z - targetTransform.position.z);

            float directionChangeX = lastDistanceToTargetX - distanceToTargetX;
            float directionChangeZ = lastDistanceToTargetZ - distanceToTargetZ;

            totDirectionChangeReward = (directionChangeX + directionChangeZ) * 10f;
            totDirectionChangeReward = Mathf.Clamp(totDirectionChangeReward, -0.5f, 0.5f);

            float distanceRewardX = (1f - distanceToTargetX / 8f);
            float distanceRewardZ = (1f - distanceToTargetZ / 8f);

            totDistanceReward = (distanceRewardX + distanceRewardZ) / 20f;

            reward += totDirectionChangeReward + totDistanceReward;
        }

        
        if (inTarget)
        {
            float angleToTarget = Vector3.Angle(transform.forward, targetTransform.forward);
            // When driving in the spot backwards, the angle to target is 180 degrees
            if (angleToTarget > 90f)
            {
                angleToTarget = 180f - angleToTarget;
            }

            angleToTarget = Mathf.Clamp(angleToTarget, 0f, 90f);
            float angleReward = (-(1f / 45f) * angleToTarget) + 1f;

            totAngleChangeReward = angleReward + 1f;

            // Reward for minimising the angle to the target
            reward += totAngleChangeReward;

            float distanceToTarget = Vector3.Distance(transform.position, targetTransform.position);

            // Check if car was able to park and reward it accordingly
            if (angleToTarget < 2.5f && distanceToTarget < 1f && Mathf.Abs(rb.velocity.magnitude) < 2f)
            {
                Debug.Log("Car parked!");
                reward += 100f;
                EndEpisode();
            }

        }

        lastPosition = transform.position;
        return reward;
    }*/

}


