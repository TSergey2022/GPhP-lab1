using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class RobotJoint : MonoBehaviour
{
    public float MinAngle = float.NegativeInfinity;
    public float MaxAngle = float.PositiveInfinity;
    public float Angle;
    public Vector3 Axis;
    public Vector3 StartOffset;
    void FixedUpdate() {
        Angle = Mathf.Clamp(Angle, MinAngle, MaxAngle);
        transform.localEulerAngles = Axis * Angle;
    }
}
