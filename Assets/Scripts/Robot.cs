using System.Collections;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;

public class Robot : MonoBehaviour
{
    public enum KinematicsModeEnum {
        ForwardKinematics,
        InverseKinematics
    }

    public bool DrawDebugLines = true;

    public KinematicsModeEnum KinematicsMode = KinematicsModeEnum.ForwardKinematics;
    public GameObject Target;
    public float SamplingDistance;
    public float LearningRate;
    public float DistanceThreshold;
    public uint MaxIterations = 100;
    private uint currentIteration = 0;
    private Vector3 lastTargetPosition;
    private float minDistance = float.PositiveInfinity;
    RobotJoint[] Joints;

    private void Awake()
    {
        Joints = GetComponentsInChildren<RobotJoint>();
    }

    private void FixedUpdate()
    {
        if (Target == null) return;

        float[] angles = Joints
                .Select(j => j.Angle)
                .ToArray<float>();

        if (KinematicsMode == KinematicsModeEnum.ForwardKinematics) {
            #region Forward Kinematics
            Vector3 NewTargetPos = ForwardKinematics(angles);
            Target.transform.position = NewTargetPos;
            #endregion
        } else if (KinematicsMode == KinematicsModeEnum.InverseKinematics) {
            #region Inverse Kinematics
            try {
                if (Target.transform.position != lastTargetPosition) {
                    currentIteration = 0;
                    lastTargetPosition = Target.transform.position;
                    minDistance = DistanceFormTarget(lastTargetPosition, angles);
                } else {
                    if (currentIteration >= MaxIterations) return;
                    var currentDistance = DistanceFormTarget(lastTargetPosition, angles);
                    if (currentDistance < minDistance) {
                        minDistance = currentDistance;
                        currentIteration = 0;
                    } else {
                        currentIteration++;
                    }
                }
                InverseKinematics(lastTargetPosition, angles);
                for (int i = 0; i < Joints.Length; i++)
                {
                    Joints[i].Angle = angles[i];
                }
            } catch {}
            #endregion
        }
        
    }

    private Vector3 ForwardKinematics(float[] angles)
    {
        Vector3 prevPoint = Joints[0].transform.position;
        Quaternion rotation = Quaternion.identity;
        for (int i = 1; i < Joints.Length; i++)
        {
            // Выполняет поворот вокруг новой оси
            rotation *= Quaternion.AngleAxis(angles[i - 1], Joints[i - 1].Axis);
            Vector3 nextPoint = prevPoint + rotation * Vector3.Scale(Joints[i].StartOffset, Joints[i - 1].transform.lossyScale);
            
            if (DrawDebugLines) {
                Debug.DrawLine(prevPoint, nextPoint, Color.green, 0.1f);
            }
            
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
