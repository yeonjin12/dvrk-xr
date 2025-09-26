using UnityEngine;
using UnityEngine.XR.Interaction.Toolkit;
using DVRK;
using System.Collections.Generic;

// 하나의 핸들로 여러 URDF 관절 제어 (Direct / Ray 모두 OK)
// 로봇 파츠엔 XR Grab Interactable 쓰지 말고, 이 스크립트(XRSimpleInteractable)만 사용하세요.
[DefaultExecutionOrder(2000)]
[RequireComponent(typeof(Collider))]
public class URDFJointHandle : XRSimpleInteractable
{
    [Header("Targets")]
    public URDFJoint[] joints;

    [Header("Sensitivity")]
    public float angularScale = 1f;   // 회전 감도(도)
    public float linearScale  = 1f;   // 프리즘틱 감도(미터)
    public bool  invert       = false;

    public enum Mode { Delta, Absolute }
    public Mode controlMode = Mode.Delta;

    [Header("Smoothing & Limits")]
    [Tooltip("조인트 출력 보간 시간(작을수록 즉각)")]
    public float smoothTime = 0.06f;
    public float maxSmoothingSpeed = 1000f;

    [Tooltip("미세 떨림 무시 임계값")]
    public float deadzoneAngleDeg = 0.8f;     // deg
    public float deadzoneLinearM  = 0.0015f;  // m

    [Tooltip("조인트 ‘속도 제한’ (초당)")]
    public float maxDegPerSec = 180f;         // deg/s
    public float maxMPerSec   = 0.35f;        // m/s

    [Tooltip("URDF Joint Limit 존중")]
    public bool clampToJointLimits = true;

    // ────────────────────────────────────────────────────────────────

    class Target
    {
        public URDFJoint joint;
        public float q0;       // 그랩 시 기준값
        public float outVal;   // 현재 출력값(누적)
        public float vel;      // SmoothDamp용
        public Vector3 pivot;  // 관절 피벗(월드)
        public Vector3 axisW;  // 축(월드)
        public int sign;       // Revolute/Cont:-1, Prismatic:+1
        public Vector3 vRef;   // 그랩 시 기준 벡터(회전)
    }

    Transform interactorTf;       // 손/레이 attach transform
    readonly List<Target> targets = new();

    // 앵커(잡은 지점)
    Vector3 anchorW;
    Vector3 anchorUp;

    protected override void OnEnable()
    {
        base.OnEnable();
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

        // 레이면 히트포인트, Direct면 손 위치를 앵커로
        Vector3 hitPos = interactorTf.position;
        Vector3 hitNormal = interactorTf.up;
        bool valid = false;

        var ray = (args.interactorObject as Component)?.GetComponent<XRRayInteractor>();
        if (ray != null)
        {
            try
            {
                if (ray.TryGetHitInfo(out var p, out var n, out _, out var v) && v)
                { hitPos = p; hitNormal = n; valid = true; }
            }
            catch { /* 구버전 대응 */ }

            if (!valid && ray.TryGetCurrent3DRaycastHit(out RaycastHit hit))
            { hitPos = hit.point; hitNormal = hit.normal; valid = true; }
        }

        anchorW = hitPos;
        anchorUp = interactorTf.up;

        // 캐시
        targets.Clear();
        if (joints == null) return;

        foreach (var j in joints)
        {
            if (!j || !j.independent) continue;

            var t = new Target();
            t.joint  = j;
            t.q0     = j.currentJointValue;
            t.outVal = j.currentJointValue;
            t.vel    = 0f;

            t.pivot  = j.jointObject.transform.position;
            (t.axisW, t.sign) = GetAxisWorldAndSign(j);

            // 앵커 기준 회전 레퍼런스
            t.vRef = Vector3.ProjectOnPlane(anchorW - t.pivot, t.axisW).normalized;
            if (t.vRef.sqrMagnitude < 1e-6f) t.vRef = j.jointObject.transform.right;

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

        float dt = Mathf.Max(Time.deltaTime, 1e-4f);

        foreach (var t in targets)
        {
            var j = t.joint;
            if (!j) continue;

            float desired; // 목표 조인트값

            if (j.jointType == URDFJoint.JointType.Prismatic)
            {
                // 앵커에서 손 위치까지의 축 성분(절대 변위)
                float d = Vector3.Dot(interactorTf.position - anchorW, t.axisW);

                // 데드존
                if (Mathf.Abs(d) < deadzoneLinearM) d = 0f;

                desired = t.q0 + t.sign * (invert ? -d : d) * linearScale;

                if (clampToJointLimits)
                    desired = Mathf.Clamp(desired, j.jointLimit.x, j.jointLimit.y);

                // ‘속도 제한(초당)’으로 outVal을 desired 쪽으로 이동(누적)
                float maxStep = maxMPerSec * dt;
                float step    = Mathf.Clamp(desired - t.outVal, -maxStep, +maxStep);
                float next    = t.outVal + step;

                t.outVal = Mathf.SmoothDamp(t.outVal, next, ref t.vel, smoothTime, maxSmoothingSpeed);
                j.SetJointValue(t.outVal);
            }
            else // Revolute / Continuous
            {
                Vector3 vHand = Vector3.ProjectOnPlane(interactorTf.position - t.pivot, t.axisW).normalized;
                if (vHand.sqrMagnitude < 1e-6f) continue;

                float angDeg;
                if (controlMode == Mode.Delta)
                {
                    // 앵커 기준(그랩 시점) 상대각
                    angDeg = SignedAngleOnAxis(t.vRef, vHand, t.axisW);
                    desired = t.q0 + t.sign * (invert ? -angDeg : angDeg) * angularScale;
                }
                else
                {
                    Vector3 vNow = Vector3.ProjectOnPlane(transform.position - t.pivot, t.axisW).normalized;
                    if (vNow.sqrMagnitude < 1e-6f) vNow = t.vRef;
                    angDeg = SignedAngleOnAxis(vNow, vHand, t.axisW);
                    desired = j.currentJointValue + t.sign * (invert ? -angDeg : angDeg) * angularScale;
                }

                // 데드존
                if (Mathf.Abs(Mathf.DeltaAngle(t.outVal, desired)) < deadzoneAngleDeg)
                    desired = t.outVal;

                if (clampToJointLimits && j.jointType == URDFJoint.JointType.Revolute)
                    desired = Mathf.Clamp(desired, j.jointLimit.x, j.jointLimit.y);

                // ‘각속도 제한(초당)’으로 outVal을 desired 쪽으로 이동(누적)
                float maxStep = maxDegPerSec * dt;
                float step    = Mathf.Clamp(Mathf.DeltaAngle(t.outVal, desired), -maxStep, +maxStep);
                float next    = t.outVal + step;

                t.outVal = Mathf.SmoothDampAngle(t.outVal, next, ref t.vel, smoothTime, maxSmoothingSpeed);
                j.SetJointValue(t.outVal);
            }
        }
    }

    // ───────── helpers ─────────
    static float SignedAngleOnAxis(Vector3 from, Vector3 to, Vector3 axis)
    {
        float x = Vector3.Dot(axis, Vector3.Cross(from, to));
        float y = Mathf.Clamp(Vector3.Dot(from, to), -1f, 1f);
        return Mathf.Atan2(x, y) * Mathf.Rad2Deg;
    }

    static float SmoothDampAngle(float current, float target, ref float currentVelocity, float smoothTime, float maxSpeed)
    {
        return Mathf.SmoothDampAngle(current, target, ref currentVelocity, Mathf.Max(0.0001f, smoothTime), maxSpeed);
    }

    static float SmoothDamp(float current, float target, ref float currentVelocity, float smoothTime, float maxSpeed)
    {
        return Mathf.SmoothDamp(current, target, ref currentVelocity, Mathf.Max(0.0001f, smoothTime), maxSpeed);
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
