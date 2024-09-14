using System.Collections;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;

public class Robot : MonoBehaviour
{
    public GameObject Target;
    RobotJoint[] Joints;

    private void Awake()
    {
        Joints = GetComponentsInChildren<RobotJoint>();
    }

    private void FixedUpdate()
    {
        float[] angles = Joints
                .Select(j => Vector3.Dot(j.transform.localRotation.eulerAngles, j.Axis))
                .ToArray<float>();

        Vector3 NewTargetPos = ForwardKinematics(angles);

        Target.transform.position = NewTargetPos;
    }

    private Vector3 ForwardKinematics(float[] angles)
    {
        Vector3 prevPoint = Joints[0].transform.position;
        Quaternion rotation = Quaternion.identity;
        for (int i = 1; i < Joints.Length; i++)
        {
            // Выполняет поворот вокруг новой оси
            rotation *= Quaternion.AngleAxis(angles[i - 1], Joints[i - 1].Axis);
            Vector3 nextPoint = prevPoint + rotation * Joints[i].StartOffset;

            // Debug.DrawLine(prevPoint, nextPoint, Color.green, 0.1f);
            prevPoint = nextPoint;
        }
        return prevPoint;
    }
}
