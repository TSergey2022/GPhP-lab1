using System.Collections;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;

public class Robot : MonoBehaviour
{
    public GameObject Target;
    public float SamplingDistance;
    public float LearningRate;
    public float DistanceThreshold;
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

        #region Forward Kinematics
        // Vector3 NewTargetPos = ForwardKinematics(angles);
        // Target.transform.position = NewTargetPos;
        #endregion

        #region Inverse Kinematics
        InverseKinematics(Target.transform.position, angles);
        for (int i = 0; i < Joints.Length; i++)
        {
            Joints[i].transform.localRotation = Quaternion.AngleAxis(
                angles[i], Joints[i].Axis
            );
        }
        #endregion
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

    private void InverseKinematics(Vector3 target, float[] angles)
    {
        for (int i = 0; i < Joints.Length; i++)
        {
            // Gradient descent
            float gradient = PartialGradient(target, angles, i);

            // Update: Solution -= LearningRate * Gradient;
            angles[i] -= LearningRate * gradient;

            // Stop at epsilon-neighborhood
            if (DistanceFormTarget(target, angles) < DistanceThreshold)
                return;
        }
    }

    private float DistanceFormTarget(Vector3 target, float[] angles)
    {
        Vector3 point = ForwardKinematics(angles);
        return Vector3.Distance(point, target);
    }

    private float PartialGradient(Vector3 target, float[] angles, int i)
    {
        // Saves the angle (will be restored later)
        float angle = angles[i];

        // Gradient: [F(x + SamplingDistance) - F(x)] / h
        float f_x = DistanceFormTarget(target, angles);

        angles[i] += SamplingDistance;
        float f_x_plus_d = DistanceFormTarget(target, angles);

        float gradient = (f_x_plus_d - f_x) / SamplingDistance;

        // Restore angle
        angles[i] = angle;

        return gradient;
    }
}
