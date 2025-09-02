using UnityEngine;
using UnityEngine.XR.Interaction.Toolkit;
using DVRK;
using System.Collections.Generic;

// URDF joint 핸들(멀티): 하나의 핸들로 여러 조인트를 동시에 제어
// 이 컴포넌트를 "핸들 콜라이더" 오브젝트에 붙이고,
// joints 배열에 제어할 독립 조인트들을 넣으세요.
// 핸들은 보통 첫 번째 joint.jointObject의 자식으로 두는걸 권장합니다.
[DefaultExecutionOrder(2000)]
[RequireComponent(typeof(Collider))]
public class URDFJointHandle : XRSimpleInteractable
{
    [Header("Targets")]
    public URDFJoint[] joints;      // 여러 개 할당!

    [Header("Sensitivity")]
    public float angularScale = 1f; // 회전 감도(도)
    public float linearScale  = 1f; // 프리즘틱 감도(미터)
    public bool invert = false;

    public enum Mode { Delta, Absolute }
    public Mode controlMode = Mode.Delta;

    // 내부 캐시(조인트별)
    class Target
    {
        public URDFJoint joint;
        public float q0;
        public Vector3 pivot;
        public Vector3 axisW;
        public int sign;         // Revolute/Cont:-1, Prismatic:+1
        public Vector3 vRef;     // 그랩 시 기준 핸들 방향(회전용)
    }

    Transform interactorTf;
    Vector3 grabP0;                 // 그랩 시 손 위치
    readonly List<Target> targets = new();

    protected override void OnEnable()
    {
        base.OnEnable();
        // 비어 있으면 부모에서 하나 자동 할당(편의)
        if (joints == null || joints.Length == 0)
        {
            var j = GetComponentInParent<URDFJoint>();
            if (j) joints = new[] { j };
        }
    }

    protected override void OnSelectEntered(SelectEnterEventArgs args)
    {
        base.OnSelectEntered(args);

        interactorTf = args.interactorObject.GetAttachTransform(this);
        grabP0 = interactorTf.position;

        targets.Clear();
        if (joints == null) return;

        foreach (var j in joints)
        {
            if (!j || !j.independent) continue;

            var t = new Target();
            t.joint = j;
            t.q0    = j.currentJointValue;
            t.pivot = j.jointObject.transform.position;

            (t.axisW, t.sign) = GetAxisWorldAndSign(j);

            // 회전 기준벡터: "핸들"의 현재 위치를 기준으로 축 평면에 사영
            t.vRef = Vector3.ProjectOnPlane(transform.position - t.pivot, t.axisW).normalized;
            if (t.vRef.sqrMagnitude < 1e-6f)
                t.vRef = j.jointObject.transform.right;

            targets.Add(t);
        }
    }

    protected override void OnSelectExited(SelectExitEventArgs args)
    {
        base.OnSelectExited(args);
        interactorTf = null;
        targets.Clear();
    }

    void LateUpdate()
    {
        if (interactorTf == null || targets.Count == 0) return;

        foreach (var t in targets)
        {
            var j = t.joint;
            if (!j) continue;

            float val = t.q0;

            if (j.jointType == URDFJoint.JointType.Prismatic)
            {
                // 손 이동을 축에 투영 → 슬라이드 길이(미터)
                float d = Vector3.Dot(interactorTf.position - grabP0, t.axisW) * linearScale;
                val = t.q0 + (t.sign * (invert ? -d : d));
            }
            else // Revolute/Continuous
            {
                // 손-피벗 벡터를 축 평면에 사영
                Vector3 vHand = Vector3.ProjectOnPlane(interactorTf.position - t.pivot, t.axisW).normalized;
                if (vHand.sqrMagnitude < 1e-6f) continue;

                float angDeg;

                if (controlMode == Mode.Delta)
                {
                    // 그랩 시점 대비 상대 각도
                    angDeg = Mathf.Atan2(
                        Vector3.Dot(t.axisW, Vector3.Cross(t.vRef, vHand)),
                        Mathf.Clamp(Vector3.Dot(t.vRef, vHand), -1f, 1f)
                    ) * Mathf.Rad2Deg;

                    val = t.q0 + (t.sign * (invert ? -angDeg : angDeg) * angularScale);
                }
                else // Absolute: 핸들의 현재 방향과 손 방향을 일치
                {
                    // 현재 핸들 기준 벡터(조인트가 회전하면 함께 변함)
                    Vector3 vNow = Vector3.ProjectOnPlane(transform.position - t.pivot, t.axisW).normalized;
                    if (vNow.sqrMagnitude < 1e-6f) vNow = t.vRef;

                    angDeg = Mathf.Atan2(
                        Vector3.Dot(t.axisW, Vector3.Cross(vNow, vHand)),
                        Mathf.Clamp(Vector3.Dot(vNow, vHand), -1f, 1f)
                    ) * Mathf.Rad2Deg;

                    val = j.currentJointValue + (t.sign * (invert ? -angDeg : angDeg) * angularScale);
                }
            }

            j.SetJointValue(val); // 내부 리밋/미믹 처리
        }
    }

    (Vector3, int) GetAxisWorldAndSign(URDFJoint j)
    {
        var tr = j.jointObject.transform;
        Vector3 axis = j.jointAxis switch
        {
            URDFJoint.JointAxis.X => tr.right,    // +X
            URDFJoint.JointAxis.Y => tr.forward,  // +Z (원 코드 매핑)
            _                      => tr.up       // +Y
        };
        int s = (j.jointType == URDFJoint.JointType.Prismatic) ? +1 : -1;
        return (axis.normalized, s);
    }
}
