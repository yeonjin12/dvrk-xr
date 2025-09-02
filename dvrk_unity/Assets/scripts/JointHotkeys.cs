// URDFJointKeyControl.cs
// 빈 GameObject에 붙이고 robotRoot에 PSM1(또는 PSM2/ECM 루트) 드래그.
// jointNameContains에 "outer_pitch_joint" 등 이름 일부를 넣으세요.

using System.Linq;
using UnityEngine;
using DVRK;

public class JointHotkeys : MonoBehaviour
{
    public Transform robotRoot;
    public string jointNameContains = "outer_pitch_joint"; // 대상 조인트 이름 일부
    public KeyCode incKey = KeyCode.E;
    public KeyCode decKey = KeyCode.Q;
    public float stepDeg = 2f, holdSpeedDeg = 30f;   // 회전(deg)
    public float stepM = 0.002f, holdSpeedM = 0.02f; // 프리즘틱(m)
    public float turbo = 3f;                          // Shift 가속

    URDFJoint j;

    void Start()
    {
        if (!robotRoot) robotRoot = transform;
        j = robotRoot.GetComponentsInChildren<URDFJoint>(true)
                     .FirstOrDefault(x => x.name.ToLower().Contains(jointNameContains.ToLower()));
        if (!j)
        {
            Debug.LogError($"[URDFJointKeyControl] Joint not found by '{jointNameContains}'");
            foreach (var n in robotRoot.GetComponentsInChildren<URDFJoint>(true)) Debug.Log(" - " + n.name);
            enabled = false; return;
        }
        Debug.Log($"[URDFJointKeyControl] Controlling {j.name} ({j.jointType})");
    }

    void LateUpdate() // OnGUI가 값을 덮어써도 여기서 마지막에 적용
    {
        if (!j) return;

        float mult  = (Input.GetKey(KeyCode.LeftShift) || Input.GetKey(KeyCode.RightShift)) ? turbo : 1f;
        float delta = 0f;

        // 톡톡 눌렀을 때
        if (Input.GetKeyDown(incKey)) delta += (j.jointType == URDFJoint.JointType.Prismatic) ? stepM : stepDeg;
        if (Input.GetKeyDown(decKey)) delta -= (j.jointType == URDFJoint.JointType.Prismatic) ? stepM : stepDeg;
        // 누르고 있을 때
        if (Input.GetKey(incKey)) delta += ((j.jointType == URDFJoint.JointType.Prismatic) ? holdSpeedM : holdSpeedDeg) * mult * Time.deltaTime;
        if (Input.GetKey(decKey)) delta -= ((j.jointType == URDFJoint.JointType.Prismatic) ? holdSpeedM : holdSpeedDeg) * mult * Time.deltaTime;

        if (Mathf.Approximately(delta, 0f)) return;

        float target = j.currentJointValue + delta; // Revolute/Continuous: deg, Prismatic: m
        // 리밋은 URDFJoint.SetJointValue 내부에서 이미 Clamp됨
        j.SetJointValue(target);
    }
}
