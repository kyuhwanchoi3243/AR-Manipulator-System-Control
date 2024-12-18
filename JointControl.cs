using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.Robotics.UrdfImporter;

public class JointControl : MonoBehaviour
{
    Unity.Robotics.UrdfImporter.Control.Controller controller;

    public Unity.Robotics.UrdfImporter.Control.RotationDirection direction;
    public Unity.Robotics.UrdfImporter.Control.ControlType controltype;
    public float speed ;
    public float torque ;
    public float acceleration;
    public ArticulationBody joint;
    private float targetPosition; // 목표 위치를 저장할 변수 추가
    private Queue<float> trajectoryPoints;
    private bool isTrajectoryMode = false;
    private static bool startTrajectory = false;
    private float lastFixedUpdateTime;
    private float fixedUpdateInterval;
    private int fixedUpdateCount = 0;
    private float elapsedTime = 0f;
    private bool isTrajectoryControlScene = false;

    void Start()
    {
        direction = 0;
        controller = (Unity.Robotics.UrdfImporter.Control.Controller)this.GetComponentInParent(typeof(Unity.Robotics.UrdfImporter.Control.Controller));
        joint = this.GetComponent<ArticulationBody>();
        controller.UpdateControlType(this);
        speed = controller.speed;
        torque = controller.torque;
        acceleration = controller.acceleration;
        targetPosition = joint.xDrive.target; // 초기 목표 위치 설정
        trajectoryPoints = new Queue<float>(); // Queue 초기화 추가
        Debug.Log("init targetPosition: " + targetPosition + " " + joint.name);
        lastFixedUpdateTime = Time.time;
        fixedUpdateInterval = Time.fixedDeltaTime;
        Debug.Log($"설정된 FixedUpdate 간격: {fixedUpdateInterval}초");

        // 현재 씬이 TrajectoryControl인지 확인
        isTrajectoryControlScene = UnityEngine.SceneManagement.SceneManager.GetActiveScene().name == "TrajectoryControl";
    }

    void FixedUpdate(){
        // FixedUpdate 실행 간격 계산
        float currentTime = Time.time;
        float actualInterval = currentTime - lastFixedUpdateTime;
        lastFixedUpdateTime = currentTime;

        // 매 초마다 평균 실행 횟수 출력
        fixedUpdateCount++;
        elapsedTime += actualInterval;

        if (elapsedTime >= 1f)
        {
            // Debug.Log($"FixedUpdate 실제 평균 간격: {elapsedTime/fixedUpdateCount}초");
            // Debug.Log($"초당 실행 횟수: {fixedUpdateCount}회");
            fixedUpdateCount = 0;
            elapsedTime = 0f;
        }

        speed = controller.speed;
        torque = controller.torque;
        acceleration = controller.acceleration;


        if (joint.jointType != ArticulationJointType.FixedJoint)
        {
            if (controltype == Unity.Robotics.UrdfImporter.Control.ControlType.PositionControl)
            {
                ArticulationDrive currentDrive = joint.xDrive;
                float currentPosition = currentDrive.target;

                // trajectory 제어인 경우
                if (isTrajectoryMode && startTrajectory && trajectoryPoints.Count > 0)
                {
                    float nextTarget = trajectoryPoints.Dequeue();
                    currentDrive.target = nextTarget;

                    if (joint.twistLock == ArticulationDofLock.LimitedMotion)
                    {
                        if (currentDrive.target > currentDrive.upperLimit)
                        {
                            currentDrive.target = currentDrive.upperLimit;
                        }
                        else if (currentDrive.target < currentDrive.lowerLimit)
                        {
                            currentDrive.target = currentDrive.lowerLimit;
                        }
                    }

                    joint.xDrive = currentDrive;

                    Debug.Log("TrajectoryMoving");
                    if (trajectoryPoints.Count == 0)
                    {
                        targetPosition = currentDrive.target;
                        // isTrajectoryMode = false;
                        Debug.Log("init TrajectoryMode");
                        // 마지막 포인트를 처리한 후 startTrajectory 리셋
                        if (!AnyJointStillMoving())
                        {
                            startTrajectory = false;
                            Debug.Log("init startTrajectory");
                        }
                    }
                }

                // trajectory 제어가 아닌 경우
                else{
                    if (Mathf.Abs(targetPosition - currentPosition) > 0.6f) // 허용 오차 설정
                    {
                        if (targetPosition > currentPosition)
                        {
                            direction = Unity.Robotics.UrdfImporter.Control.RotationDirection.Positive;
                        }
                        else if (targetPosition < currentPosition)
                        {
                            direction = Unity.Robotics.UrdfImporter.Control.RotationDirection.Negative;
                        }
                    }
                    else if (isTrajectoryControlScene || Mathf.Abs(targetPosition - currentPosition) <= 0.6f)
                    {
                        direction = Unity.Robotics.UrdfImporter.Control.RotationDirection.None;
                        currentDrive.target = targetPosition; // 정확한 목표 위치로 설정
                        joint.xDrive = currentDrive;
                        return;
                    }

                    // Debug.Log("fixedDeltaTime: " + Time.fixedDeltaTime);
                    float newTargetDelta = (int)direction * Time.fixedDeltaTime * speed;

                    if (joint.jointType == ArticulationJointType.RevoluteJoint)
                    {
                        if (joint.twistLock == ArticulationDofLock.LimitedMotion)
                        {
                            if (newTargetDelta + currentDrive.target > currentDrive.upperLimit)
                            {
                                currentDrive.target = currentDrive.upperLimit;
                            }
                            else if (newTargetDelta + currentDrive.target < currentDrive.lowerLimit)
                            {
                                currentDrive.target = currentDrive.lowerLimit;
                            }
                            else
                            {
                                currentDrive.target += newTargetDelta;
                            }
                        }
                        else
                        {
                            currentDrive.target += newTargetDelta;

                        }
                    }

                    else if (joint.jointType == ArticulationJointType.PrismaticJoint)
                    {
                        if (joint.linearLockX == ArticulationDofLock.LimitedMotion)
                        {
                            if (newTargetDelta + currentDrive.target > currentDrive.upperLimit)
                            {
                                currentDrive.target = currentDrive.upperLimit;
                            }
                            else if (newTargetDelta + currentDrive.target < currentDrive.lowerLimit)
                            {
                                currentDrive.target = currentDrive.lowerLimit;
                            }
                            else
                            {
                                currentDrive.target += newTargetDelta;
                            }
                        }
                        else
                        {
                            currentDrive.target += newTargetDelta;

                        }
                    }

                    joint.xDrive = currentDrive;
                }
            }
        }
    }


    public float GetCurrentTarget()
    {
        return joint.xDrive.target;
    }

    public void SetJoint(float targetDegree)
    {
        if (joint == null)
        {
            Debug.LogError("ArticulationBody is null in SetJoint for " + gameObject.name);
            return;
        }

        if (joint.jointType != ArticulationJointType.FixedJoint)
        {
            if (controltype == Unity.Robotics.UrdfImporter.Control.ControlType.PositionControl)
            {
                ArticulationDrive drive = joint.xDrive;

                if (targetDegree <= drive.upperLimit && targetDegree >= drive.lowerLimit)
                {
                    // 직접적인 위치 설정 대신 목표 위치만 설정
                    // drive.target = targetDegree;
                    // joint.xDrive = drive;
                    targetPosition = targetDegree;

                    isTrajectoryMode = false;
                    trajectoryPoints.Clear();
                }
            }
        }
    }

    public void SetTrajectoryPoints(float[] points)
    {
        // trajectoryPoints가 null인 경우 초기화
        if (trajectoryPoints == null)
        {
            trajectoryPoints = new Queue<float>();
        }

        trajectoryPoints.Clear();
        foreach (float point in points)
        {
            trajectoryPoints.Enqueue(point);
        }
        isTrajectoryMode = true;
        Debug.Log("SetTrajectoryPoints");
    }

    public static void StartTrajectory()
    {
        Debug.Log("startTrajectory");
        startTrajectory = true;
    }

    public bool AnyJointStillMoving()
    {
        var allJointControls = FindObjectsOfType<JointControl>();
        foreach (var jointControl in allJointControls)
        {
            if (jointControl.trajectoryPoints.Count > 0)
            {
                return true;
            }
        }
        return false;
    }

    // 조인트가 움직이고 있는지 확인하는 메서드 추가
    public bool IsMoving()
    {
        if (isTrajectoryMode)
        {
            return trajectoryPoints.Count > 0;
        }
        else
        {
            return Mathf.Abs(joint.xDrive.target - targetPosition) > 0.1f ||
                   Mathf.Abs(joint.jointPosition[0] - targetPosition) > 0.1f;
        }
    }

    public void StopMovement()
    {
        // 현재 진행 중인 모든 동작 중지
        isTrajectoryMode = false;
        trajectoryPoints.Clear();  // Queue 초기화

        // 현재 위치를 목표 위치로 설정하여 움직임 중지
        ArticulationDrive currentDrive = joint.xDrive;
        targetPosition = currentDrive.target;
        direction = Unity.Robotics.UrdfImporter.Control.RotationDirection.None;
    }
}
