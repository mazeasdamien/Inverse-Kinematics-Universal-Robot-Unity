using System;
using System.Collections;
using System.Collections.Generic;
using Unity.VisualScripting;
using UnityEngine;
using static UnityEngine.GraphicsBuffer;

// Cobot의 End-effector를 내가 원하는 위치와 회전으로 정해준다.
public class AutomationController : MonoBehaviour
{
    public static AutomationController instance;

    public IK_toolkit ikToolkit;
    public bool isRobotRunning;
    public Transform targetToPick;       // 타겟
    public Transform targetToPlaceInCNC; // CNC머신에 위치시키기 위한 게임오브젝트
    public Transform targetHome;         // 초기 위치

    private void Awake()
    {
        if(instance == null)
            instance = this;
    }

    private void Update()
    {
        if(Input.GetKeyDown(KeyCode.Space))
        {
            StartCoroutine("MachineTendingProcess");
        }
    }

    public void StartSequence(List<UIController.TeachData> datas)
    {
        StartCoroutine(Sequence(datas));
    }

    public void CycleSequence(List<UIController.TeachData> datas)
    {

    }


    IEnumerator Sequence(List<UIController.TeachData> datas)
    {
        isRobotRunning = true;

        foreach (var data in datas)
        {
            yield return MoveRobotTo(data.pos, data.rot, data.duration);
        }

        isRobotRunning = false;
    }

    // 시퀀스 프로그램 작성
    IEnumerator MachineTendingProcess()
    {
        while(true)
        {
            yield return MoveRobotTo(targetToPick, 2);

            // 그리퍼 작동

            yield return MoveRobotTo(targetToPlaceInCNC, 3);

            // 가공 대기
            yield return new WaitForSeconds(5);

            // 그리퍼 작동

            yield return MoveRobotTo(targetHome, 2);
        }
    }

    float elapsedTime;
    private IEnumerator MoveRobotTo(Transform target, int time)
    {
        Vector3 startPos = ikToolkit.ik.position;
        Quaternion startRot = ikToolkit.ik.rotation;

        elapsedTime = 0;

        while(elapsedTime < time)
        {
            elapsedTime += Time.deltaTime;

            // end-effector를 target위치로 Lerp를 사용해 time동안 이동
            ikToolkit.ik.position = Vector3.Lerp(startPos, target.position, elapsedTime / time);

            // end-effector를 target위치로 Slerp를 사용해 time동안 회전
            ikToolkit.ik.rotation = Quaternion.Slerp(startRot, target.rotation, elapsedTime / time);

            yield return new WaitForEndOfFrame();
        }
    }

    // 함수의 오버로드
    private IEnumerator MoveRobotTo(Vector3 pos, Vector3 rot, float time)
    {
        Vector3 startPos = ikToolkit.ik.position;
        Quaternion startRot = ikToolkit.ik.rotation;
        Quaternion endRot = Quaternion.Euler(rot.x, rot.y, rot.z);

        elapsedTime = 0;

        while (elapsedTime < time)
        {
            elapsedTime += Time.deltaTime;

            // end-effector를 target위치로 Lerp를 사용해 time동안 이동
            ikToolkit.ik.position = Vector3.Lerp(startPos, pos, elapsedTime / time);

            // end-effector를 target위치로 Slerp를 사용해 time동안 회전
            ikToolkit.ik.rotation = Quaternion.Slerp(startRot, endRot, elapsedTime / time);

            yield return new WaitForEndOfFrame();
        }
    }

}
