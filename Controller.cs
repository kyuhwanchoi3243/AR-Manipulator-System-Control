using System;
using System.Collections;
using System.Collections.Generic;
using Unity.Robotics;
using UnityEngine;
using UnityEngine.UI; // UI 네임스페이스 추가
using RosMessageTypes.JointControl;
using RosMessageTypes.Std;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;

namespace Unity.Robotics.UrdfImporter.Control
{
    public enum RotationDirection { None = 0, Positive = 1, Negative = -1 };
    public enum ControlType { PositionControl };

    public class Controller : MonoBehaviour
    {
        private ArticulationBody[] articulationChain;
        private Color[] prevColor;
        private int previousIndex;

        [InspectorReadOnly(hideInEditMode: true)]
        public string selectedJoint;
        [HideInInspector]
        public int selectedIndex;

        public ControlType control = ControlType.PositionControl;
        public float stiffness;
        public float damping;
        public float forceLimit;
        public float speed = 5f; // Units: degree/s
        public float torque = 100f; // Units: Nm or N
        public float acceleration = 5f;// Units: m/s^2 / degree/s^2

        [Tooltip("Color to highlight the currently selected join")]
        public Color highLightColor = new Color(1.0f, 0, 0, 1.0f);

        public Text infoText; // Text 컴포넌트 참조 변수 추가

        private ROSConnection ros;
        private const string topicName = "joint_control";
        private const int numRobotJoints = 6;

        private Joint_listMsg currentMessage; // 현재 실행 중인 메시지
        private bool isRepeating = false;
        private Coroutine repeatCoroutine;

        private float[] lastJointPositions; // 마지막 조인트 포지션 저장 변수 추가

        private Joint_listMsg savedJointMsg;

        private bool isTrajectoryControlScene = false;
        private int currentTrajectoryIndex = 0;

        protected virtual void Start()
        {
            previousIndex = selectedIndex = 1;
            this.gameObject.AddComponent<FKRobot>();
            articulationChain = this.GetComponentsInChildren<ArticulationBody>();
            int defDyanmicVal = 4;

            // JointControl 컴포넌트들을 저장할 리스트
            List<JointControl> jointControls = new List<JointControl>();

            foreach (ArticulationBody joint in articulationChain)
            {
                JointControl jointControl = joint.gameObject.AddComponent<JointControl>();
                jointControls.Add(jointControl);
                joint.jointFriction = 0;
                joint.angularDamping = 3;
                ArticulationDrive currentDrive = joint.xDrive;
                currentDrive.forceLimit = forceLimit;
                currentDrive.driveType = ArticulationDriveType.Target;

                joint.xDrive = currentDrive;
            }

            DisplaySelectedJoint(selectedIndex);
            StoreJointColors(selectedIndex);

            if (infoText == null)
            {
                infoText = GameObject.Find("InfoText").GetComponent<Text>();
            }

            // 현재 씬이 TrajectoryControl인지 확인
            isTrajectoryControlScene = UnityEngine.SceneManagement.SceneManager.GetActiveScene().name == "TrajectoryControl";
            Debug.Log("isTrajectoryControlScene: " + isTrajectoryControlScene);

            // 모든 JointControl이 초기화될 때까지 기다림
            StartCoroutine(WaitForJointControlsInitialization(jointControls));
        }

        private IEnumerator WaitForJointControlsInitialization(List<JointControl> jointControls)
        {
            // 모든 JointControl의 Start 메서드가 실행될 때까지 기다림
            yield return new WaitForSeconds(1f);

            // JointControl들이 초기화된 후 실행
            if (!isTrajectoryControlScene)
            {
                ros = ROSConnection.GetOrCreateInstance();
                ros.Subscribe<Joint_listMsg>(topicName, ReceiveJointControl);
                Debug.Log("Subscribed to " + topicName);
            }
            else
            {
                // 저장된 조인트 메시지가 있다면 불러오기
                if (JointDataManager.IsJointMessageSaved())
                {
                    savedJointMsg = JointDataManager.GetSavedJointMessage();
                    ReceiveJointControl(savedJointMsg);
                    Debug.Log("Saved message loaded");
                }
            }
        }

        void ReceiveJointControl(Joint_listMsg msg)
        {
            if (msg.data.Length == 0 || msg.data[0].data.Length != numRobotJoints)
            {
                Debug.LogError($"Received wrong number of joint positions. Expected {numRobotJoints}");
                return;
            }

            currentMessage = msg;

            if (!isTrajectoryControlScene)
            {
                Debug.Log("Not TrajectoryControlScene");
                // 현재 메시지 저장
                JointDataManager.SetJointMessage(msg);
                // 현재 실행 중인 코루틴 정지
                if (repeatCoroutine != null)
                {
                    StopCoroutine(repeatCoroutine);
                    StopAllJointMovements();
                }
                repeatCoroutine = StartCoroutine(RepeatTrajectory());
            }
            else
            {
                Debug.Log("TrajectoryControlScene");
                StartCoroutine(InitializeTrajectoryControl());
            }
        }

        private IEnumerator InitializeTrajectoryControl()
        {
            if (currentMessage.data.Length > 0)
            {
                PlaySinglePoint(lastJointPositions);
                yield return StartCoroutine(WaitForAllJointsToComplete());
                Debug.Log("PlaySinglePoint");
                PlaySinglePoint(currentMessage.data[0].data);
                Debug.Log("PlaySinglePoint completed");
                currentTrajectoryIndex = 0;
            }
        }

        private void StopAllJointMovements()
        {
            for (int i = 2; i < numRobotJoints + 2; i++)
            {
                if (i >= articulationChain.Length) break;
                JointControl jointControl = articulationChain[i].GetComponent<JointControl>();
                jointControl?.StopMovement();
            }
        }

        private IEnumerator RepeatTrajectory()
        {
            while (true)
            {
                // 순방향 재생
                Debug.Log("Forward playback starting");
                if (currentMessage.data.Length == 1)
                {
                    // 단일 포인트 제어
                    PlaySinglePoint(currentMessage.data[0].data);
                }
                else
                {
                    // 트라젝토리 제어
                    PlayTrajectory(currentMessage.data, false); // false = 순방향
                }

                yield return StartCoroutine(WaitForAllJointsToComplete());
                Debug.Log("Forward playback completed");

                // 1초 대기
                yield return new WaitForSeconds(1f);

                // 역방향 재생 시 이전 포지션으로 돌아가기
                Debug.Log("Reverse playback starting");
                if (currentMessage.data.Length == 1)
                {
                    // 이전 메시지가 없을 경우에만 0으로 초기화
                    float[] targetPositions = lastJointPositions ?? new float[numRobotJoints];
                    PlaySinglePoint(targetPositions);
                }
                else
                {
                    // 트라젝토리 역방향 재생
                    PlayTrajectory(currentMessage.data, true); // true = 역방향
                }

                yield return StartCoroutine(WaitForAllJointsToComplete());
                Debug.Log("Reverse playback completed");

                // 2초 대기
                yield return new WaitForSeconds(2f);
            }
            // 마지막 조인트 포지션 저장
            lastJointPositions = currentMessage.data[currentMessage.data.Length - 1].data;
        }

        private void PlaySinglePoint(float[] positions)
        {
            for (int i = 2; i < numRobotJoints + 2; i++)
            {
                if (i >= articulationChain.Length) break;

                JointControl jointControl = articulationChain[i].GetComponent<JointControl>();
                if (jointControl != null)
                {
                    jointControl.SetJoint(positions[i-1]);
                }
            }
        }

        private void PlayTrajectory(Float32MultiArrayMsg[] trajectoryData, bool reverse)
        {
            for (int i = 2; i < numRobotJoints + 2; i++)
            {
                if (i >= articulationChain.Length) break;

                JointControl jointControl = articulationChain[i].GetComponent<JointControl>();
                if (jointControl != null)
                {
                    float[] trajectoryPoints = new float[trajectoryData.Length];
                    for (int j = 0; j < trajectoryData.Length; j++)
                    {
                        // reverse가 true면 역순으로 포인트 설정
                        int index = reverse ? (trajectoryData.Length - 1 - j) : j;
                        trajectoryPoints[j] = trajectoryData[index].data[i-1];
                    }
                    jointControl.SetTrajectoryPoints(trajectoryPoints);
                }
            }
            JointControl.StartTrajectory();
        }

        private IEnumerator WaitForAllJointsToComplete()
        {
            bool allCompleted = false;
            while (!allCompleted)
            {
                allCompleted = true;
                // 모든 관절(1번부터 6번까지) 확인
                for (int i = 2; i < numRobotJoints + 2; i++)
                {
                    if (i >= articulationChain.Length) break;

                    JointControl jointControl = articulationChain[i].GetComponent<JointControl>();
                    // Debug.Log($"jointControl.IsMoving(): {jointControl.IsMoving()}");
                    if (jointControl != null && jointControl.IsMoving())
                    {
                        allCompleted = false;
                        break;
                    }
                }
                yield return new WaitForSeconds(0.1f); // 성능 최적화를 위한 대기 시간 추가
            }
        }

        void SetSelectedJointIndex(int index)
        {
            if (articulationChain.Length > 0)
            {
                selectedIndex = (index + articulationChain.Length) % articulationChain.Length;
            }
        }

        void Update()
        {
            Transform baseLink = transform.Find("base_link");
            if (baseLink != null)
            {
                Debug.Log($"Base Link Position: {baseLink.position}");
                Debug.Log($"Base Link Rotation: {baseLink.rotation}");
                Debug.Log($"Base Link Scale: {baseLink.localScale}");
            }
            else
            {
                Debug.LogWarning("base_link를 찾을 수 없습니다.");
            }
        }

        public void Up()
        {
            if (selectedIndex < 0 || selectedIndex >= articulationChain.Length)
                return;

            JointControl current = articulationChain[selectedIndex].GetComponent<JointControl>();
            if (current == null)
                return;

            float currentTarget = current.GetCurrentTarget();
            current.SetJoint(currentTarget + 10f);
        }

        public void Down()
        {
            if (selectedIndex < 0 || selectedIndex >= articulationChain.Length)
                return;

            JointControl current = articulationChain[selectedIndex].GetComponent<JointControl>();
            if (current == null)
                return;

            float currentTarget = current.GetCurrentTarget();
            current.SetJoint(currentTarget - 10f);
        }

        public void Right()
        {
            SetSelectedJointIndex(selectedIndex);
            UpdateDirection(selectedIndex);
            SetSelectedJointIndex(selectedIndex + 1);
            Highlight(selectedIndex);
            UpdateDirection(selectedIndex);
        }

        public void Left()
        {
            SetSelectedJointIndex(selectedIndex);
            UpdateDirection(selectedIndex);
            SetSelectedJointIndex(selectedIndex - 1);
            Highlight(selectedIndex);
            UpdateDirection(selectedIndex);
        }

        private void Highlight(int selectedIndex)
        {
            if (selectedIndex == previousIndex || selectedIndex < 0 || selectedIndex >= articulationChain.Length)
            {
                return;
            }

            ResetJointColors(previousIndex);
            StoreJointColors(selectedIndex);
            DisplaySelectedJoint(selectedIndex);
            Renderer[] rendererList = articulationChain[selectedIndex].transform.GetChild(0).GetComponentsInChildren<Renderer>();

            foreach (var mesh in rendererList)
            {
                MaterialExtensions.SetMaterialColor(mesh.material, highLightColor);
            }

            previousIndex = selectedIndex;
        }

        void DisplaySelectedJoint(int selectedIndex)
        {
            if (selectedIndex < 0 || selectedIndex >= articulationChain.Length)
            {
                return;
            }
            selectedJoint = articulationChain[selectedIndex].name + " (" + selectedIndex + ")";

            if (infoText != null)
            {
                infoText.text = "Press left/right arrow keys to select a robot joint.\n" +
                                "Press up/down arrow keys to move " + selectedJoint + ".";
            }
        }

        private void UpdateDirection(int jointIndex)
        {
            if (jointIndex < 0 || jointIndex >= articulationChain.Length)
            {
                return;
            }

            JointControl current = articulationChain[jointIndex].GetComponent<JointControl>();
            if (current.controltype != control)
            {
                UpdateControlType(current);
            }
        }

        private void StoreJointColors(int index)
        {
            Renderer[] materialLists = articulationChain[index].transform.GetChild(0).GetComponentsInChildren<Renderer>();
            prevColor = new Color[materialLists.Length];
            for (int counter = 0; counter < materialLists.Length; counter++)
            {
                prevColor[counter] = MaterialExtensions.GetMaterialColor(materialLists[counter]);
            }
        }

        private void ResetJointColors(int index)
        {
            Renderer[] previousRendererList = articulationChain[index].transform.GetChild(0).GetComponentsInChildren<Renderer>();
            for (int counter = 0; counter < previousRendererList.Length; counter++)
            {
                MaterialExtensions.SetMaterialColor(previousRendererList[counter].material, prevColor[counter]);
            }
        }

        public void UpdateControlType(JointControl joint)
        {
            joint.controltype = control;
            if (control == ControlType.PositionControl)
            {
                ArticulationDrive drive = joint.joint.xDrive;
                drive.stiffness = stiffness;
                drive.damping = damping;
                joint.joint.xDrive = drive;
            }
        }

        public void OnGUI()
        {
            // GUI 관련 코드
        }

        protected virtual void OnDestroy()
        {
            if (repeatCoroutine != null)
            {
                StopCoroutine(repeatCoroutine);
            }

            if (ros != null)
            {
                ros.Unsubscribe(topicName);
                try
                {
                    ros.Disconnect();
                    Debug.Log("ROS connection closed successfully");
                }
                catch (Exception e)
                {
                    Debug.LogError($"Error disconnecting from ROS: {e.Message}");
                }
            }
        }
        // Update_slide 메서드 수정
    public void Update_slide(float value)
    {
        if (!isTrajectoryControlScene || currentMessage == null) return;

        if (currentMessage.data.Length == 1)
        {

        }
        else
        {
            int targetIndex = Mathf.RoundToInt(value * (currentMessage.data.Length - 1));
            Debug.Log("targetIndex: " + targetIndex);

        // 현재 실행 중인 코루틴 정지
        if (repeatCoroutine != null)
        {
            StopCoroutine(repeatCoroutine);
        }

        // 새��운 코루틴 시작
        repeatCoroutine = StartCoroutine(PlayTrajectoryToTarget(targetIndex));
        }
    }

    // 새로운 코루틴 메서드
    private IEnumerator PlayTrajectoryToTarget(int targetIndex)
    {
        // 현재 인덱스부터 목표 인덱스까지 순차적으로 실행
        int direction = targetIndex > currentTrajectoryIndex ? 1 : -1;

        while (currentTrajectoryIndex != targetIndex)
        {
            // 현재 위치의 조인트 각도 설정
            PlaySinglePoint(savedJointMsg.data[currentTrajectoryIndex].data);

            // 모든 조인트가 목표 위치에 도달할 때까지 대기
            // yield return StartCoroutine(WaitForAllJointsToComplete());

            // 다음 인덱스로 이동
            currentTrajectoryIndex += direction;

            // 범위 체크
            if (currentTrajectoryIndex < 0) currentTrajectoryIndex = 0;
            if (currentTrajectoryIndex >= savedJointMsg.data.Length)
                currentTrajectoryIndex = savedJointMsg.data.Length - 1;
        }

        // 최종 목표 위치 설정
        PlaySinglePoint(savedJointMsg.data[targetIndex].data);
        yield return StartCoroutine(WaitForAllJointsToComplete());
    }

    }
}
