using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Sensors;


public class MoveToGoal : Agent
{
    [SerializeField] private Transform targetTransform;
    [SerializeField] private ParticleSystem wind;
    [SerializeField] private Transform headSail;
    [SerializeField] private Transform mainSail;
    [SerializeField] private Transform rudder;

    public Rigidbody yachtRigidBody;

    public float rotationThreshold = 20f;

    public float prevRemainingDist;
    public float remainingDist;

    public float angle;

    public Vector3 targetDir;

    public IYachtControls yachtControls;

    private void Awake()
    {
        yachtControls = GetComponent<IYachtControls>();
    }

    public override void OnEpisodeBegin()
    {
        wind.Stop();

        yachtRigidBody.velocity = new Vector3(0,0,0);

        angle = 0;

        targetDir = targetTransform.localPosition - transform.localPosition;

        //transform.localPosition = new Vector3(0, 0, -51);
        transform.localPosition = new Vector3(Random.Range(-50f, 20f), 0, -51);
        transform.localEulerAngles = new Vector3(0, 0, 0);

        headSail.localPosition = new Vector3((0f), 2.03f, (2.1f));
        headSail.localEulerAngles = new Vector3(4.24f, 0, 0);

        mainSail.localPosition = new Vector3((-0.1f), 2.5f, (-2.9f));
        mainSail.localEulerAngles = new Vector3(-1.18f, 0, 0);

        rudder.localPosition = new Vector3(5.2154f, 0, -5.15f);
        rudder.localEulerAngles = new Vector3(0, 0, -90f);

        //BoatForces.rotateRudderMLAgent(0);

        //targetTransform.localPosition = new Vector3(-34f, 0, 101f);
        targetTransform.localPosition = new Vector3(Random.Range(-58f, 28f), 0, Random.Range(100f, 128f));

        wind.Clear();
        wind.Play();
        //remainingDist = Vector3.Distance(targetTransform.localPosition, transform.localPosition);
        //prevRemainingDist = Vector3.Distance(targetTransform.localPosition, transform.localPosition);
        //Debug.Log("Target Postion: " + targetTransform.localPosition);
    }

    public override void CollectObservations(VectorSensor sensor)
    {
        sensor.AddObservation(transform.localPosition);
        sensor.AddObservation(targetTransform.localPosition);

        sensor.AddObservation(angle);

        sensor.AddObservation(rudder.localEulerAngles);

        sensor.AddObservation(headSail.localEulerAngles);
        sensor.AddObservation(mainSail.localEulerAngles);

    }
    public override void OnActionReceived(ActionBuffers actions)
    {
        Debug.Log("action: " + actions.DiscreteActions[0]);
        //Discreate Values
        float turnAmount = 0f;
        float headSailTurnAmount = 0f;
        float mainSailTurnAmount = 0f;

        float rotationDegree = transform.rotation.eulerAngles.z;

        if (rotationDegree > 180f)
        {
            rotationDegree -= 360f;
        }
        
        
        if (rotationDegree > rotationThreshold || rotationDegree < -rotationThreshold)
        {
            SetReward(-10f);
            EndEpisode();
        }

        //remainingDist = Vector3.Distance(targetTransform.localPosition, transform.localPosition);
        //Debug.Log("Remaining Distance: " + remainingDist);
        //targetDir = targetTransform.position - transform.position;
        //angle = (int)Vector3.SignedAngle(targetDir, transform.forward, Vector3.up);
        //Debug.Log("Angle Towards Goal: " + angle);

        //if (remainingDist < prevRemainingDist && angle < 10)
        //{
        //    prevRemainingDist = remainingDist;
        //    AddReward(+0.2f);
        //}
        //else
        //{
        //    AddReward(-0.1f);
        //}

        targetDir = targetTransform.position - transform.position;
        angle = (int)Vector3.SignedAngle(targetDir, transform.forward, Vector3.up);
        Debug.Log("Angle Towards Goal: " + angle);
        if (angle > 10)
        {
            AddReward(-0.1f);
            //yachtControls.rotateRudder(1);
        }
        else if (angle < -10)
        {
            AddReward(-0.1f);
            //yachtControls.rotateRudder(-1);
        }
        else
        {
            AddReward(+0.2f);
        }

        //targetDir = targetTransform.position - transform.position;
        //angle = Vector3.Angle(targetDir, transform.forward);
        ////Debug.Log("target: " + targetDir);
        //Debug.Log("angle: " + angle);


        //if (angle > -0.05 && angle < 0.05)
        //{
        //    Debug.Log("reward+5");
        //    AddReward(+5f);
        //}
        //else
        //{
        //    float rewardfunction = (float)(System.Math.Log(1 / System.Math.Abs(angle)) + 2);
        //    Debug.Log("Reward: " + rewardfunction);
        //    AddReward(rewardfunction);
        //}


        switch (actions.DiscreteActions[0])
        {
            case 0: 
                turnAmount = 0f;
                //rudder.localRotation = Quaternion.Euler(rudder.localRotation.x, rudder.localRotation.y + turnAmount, rudder.localRotation.z);
                //yachtControls.rotateRudder(System.Convert.ToInt32(turnAmount));
                Debug.Log("No Turn");
                break;
            case 1: 
                turnAmount = +1f;
                //rudder.localRotation = Quaternion.Euler(rudder.localRotation.x, rudder.localRotation.y + turnAmount, rudder.localRotation.z );
                //yachtControls.rotateRudder(System.Convert.ToInt32(turnAmount));
                Debug.Log("Left Turn");

                break;
            case 2: 
                turnAmount = -1f;
                //rudder.localRotation = Quaternion.Euler(rudder.localRotation.x, rudder.localRotation.y + turnAmount, rudder.localRotation.z);
                //transform.localPosition += new Vector3(turnAmount, 0, 0) * Time.deltaTime;
                Debug.Log("Right Turn");

                break;
        }

        switch (actions.DiscreteActions[1])
        {
            case 0:
                headSailTurnAmount = 0f;
                //headSail.localRotation = Quaternion.Euler(headSail.localRotation.x, headSail.localRotation.y + headSailTurnAmount, headSail.localRotation.z);
                //yachtControls.rotateHeadSail(System.Convert.ToInt32(headSailTurnAmount));
                break;
            case 1:
                headSailTurnAmount = +1f;
                //headSail.localRotation = Quaternion.Euler(headSail.localRotation.x, headSail.localRotation.y + headSailTurnAmount, headSail.localRotation.z);
                //yachtControls.rotateHeadSail(System.Convert.ToInt32(headSailTurnAmount));

                break;
            case 2:
                headSailTurnAmount = -1f;
                //headSail.localRotation = Quaternion.Euler(headSail.localRotation.x, headSail.localRotation.y + headSailTurnAmount, headSail.localRotation.z);
                break;
        }

        switch (actions.DiscreteActions[2])
        {
            case 0:
                mainSailTurnAmount = 0f;
                //mainSail.localRotation = Quaternion.Euler(mainSail.localRotation.x, mainSail.localRotation.y + mainSailTurnAmount, mainSail.localRotation.z);
                //yachtControls.rotateMainSail(System.Convert.ToInt32(mainSailTurnAmount));
                break;
            case 1:
                mainSailTurnAmount = +1f;
                //mainSail.localRotation = Quaternion.Euler(mainSail.localRotation.x, mainSail.localRotation.y + mainSailTurnAmount, mainSail.localRotation.z);
                //yachtControls.rotateMainSail(System.Convert.ToInt32(mainSailTurnAmount));
                break;
            case 2:
                mainSailTurnAmount = -1f;
                //mainSail.localRotation = Quaternion.Euler(mainSail.localRotation.x, mainSail.localRotation.y + mainSailTurnAmount, mainSail.localRotation.z);
                break;
                //Debug.Log(mainSailTurnAmount);
        }

        yachtControls.rotateRudder(System.Convert.ToInt32(turnAmount));
        yachtControls.rotateHeadSail(System.Convert.ToInt32(headSailTurnAmount));
        yachtControls.rotateMainSail(System.Convert.ToInt32(mainSailTurnAmount));


        //ContinousActions
        //Debug.Log(actions.DiscreteActions[0]);
        //float moveX = actions.ContinuousActions[0];
        //float moveZ = actions.ContinuousActions[1];

        //transform.localPosition += new Vector3(moveX, 0, 0) * Time.deltaTime;
    }

    public override void Heuristic(in ActionBuffers actionsOut)
    {
        //Continous Value
        //ActionSegment<float> continousActions = actionsOut.ContinuousActions;
        //continousActions[0] = Input.GetAxisRaw("Horizontal");
        //continousActions[1] = Input.GetAxisRaw("Vertical");

        //Discrete Value
        int turnAction = 0;
        if (Input.GetKey(KeyCode.A)) turnAction = 1;
        if (Input.GetKey(KeyCode.D)) turnAction = 2;

        int headSailTurnAction = 0;
        if (Input.GetKey(KeyCode.Q)) headSailTurnAction = 1;
        if (Input.GetKey(KeyCode.E)) headSailTurnAction = 2;

        int mainSailTurnAction = 0;
        if (Input.GetKey(KeyCode.Z)) mainSailTurnAction = 1;
        if (Input.GetKey(KeyCode.C)) mainSailTurnAction = 2;

        ActionSegment<int> discreteActions = actionsOut.DiscreteActions;
        Debug.Log("Turn: "+discreteActions[0]);
        discreteActions[0] = turnAction;
        discreteActions[1] = headSailTurnAction;
        discreteActions[2] = mainSailTurnAction;
    }

    private void OnTriggerEnter(Collider other)
    {
        if(other.TryGetComponent<Goal>(out Goal goal))
        {
            SetReward(+100f);
            EndEpisode();
        }
        if (other.TryGetComponent<Wall>(out Wall wall))
        {
            SetReward(-10f);
            EndEpisode();
        }
    }
}
