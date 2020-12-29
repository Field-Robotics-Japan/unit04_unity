using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.Linq;

namespace RosSharp.RosBridgeClient
{
    [RequireComponent(typeof(RosSharp.RosBridgeClient.RosConnector))]

    public class OdomPublisher : UnityPublisher<MessageTypes.Nav.Odometry>
    {
        public string frameId = "odom";
        public string rightWheelName = "right_wheel_link";
        public string leftWheelName = "left_wheel_link";
        private float rightWheelRadius;
        private float leftWheelRadius;
        private float wheelSeparation;
        private GameObject rightWheel;
        private GameObject leftWheel;
        private HingeJoint rightHinge;
        private HingeJoint leftHinge;
        private MessageTypes.Nav.Odometry message;
        private MessageTypes.Std.Time lastStamp;
        private float x;
        private float y;
        private float yaw;

        protected override void Start()
        {
            base.Start();
            rightWheel = GameObject.Find(rightWheelName);
            leftWheel = GameObject.Find(leftWheelName);
            Debug.Log("rightWheel: " + rightWheel.name);
            Debug.Log("leftWheel: " + leftWheel.name);
            TwistTest rightWheelScript = rightWheel.GetComponent<TwistTest>();
            rightWheelRadius = rightWheelScript.WheelRadius;
            TwistTest leftWheelScript = leftWheel.GetComponent<TwistTest>();
            leftWheelRadius = leftWheelScript.WheelRadius;
            Debug.Log("rightWheelRadius: " + rightWheelRadius);
            Debug.Log("leftWheelRadius: " + leftWheelRadius);
            wheelSeparation = rightWheelScript.WheelSeparation;
            Debug.Log("wheelSeparation: " + wheelSeparation);
            rightHinge = rightWheel.GetComponent<HingeJoint>();
            leftHinge = leftWheel.GetComponent<HingeJoint>();
            message = new MessageTypes.Nav.Odometry();
            lastStamp = new MessageTypes.Std.Time();
            InitializeMessage();
        }

        private void FixedUpdate()
        {
            UpdateMessage();
        }

        private void UpdateMessage()
        {
            lastStamp = message.header.stamp;
            message.header.Update();
            // float dt = (float)(message.header.stamp.secs - lastStamp.secs + (message.header.stamp.nsecs - lastStamp.nsecs) * 1e-9);
            float dt = Time.deltaTime;
            float omegaL = leftHinge.velocity * Mathf.Deg2Rad;// [rad/s]
            float omegaR = rightHinge.velocity * Mathf.Deg2Rad;// [rad/s]
            float robotVelocity = rightWheelRadius / 2.0f * omegaR + leftWheelRadius / 2.0f * omegaL;
            float robotYawrate = rightWheelRadius / wheelSeparation * omegaR - leftWheelRadius / wheelSeparation * omegaL;
            x += robotVelocity * dt * Mathf.Cos(yaw);
            y += robotVelocity * dt * Mathf.Sin(yaw);
            yaw += robotYawrate * dt;
            yaw = Mathf.Atan2(Mathf.Sin(yaw), Mathf.Cos(yaw));
            message.pose.pose.position = GetPointMsgFromXYZ(x, y, 0);
            message.pose.pose.orientation = GetQuaternionMsgFromRPY(0, -yaw, 0);
            message.twist.twist.linear.x = robotVelocity;
            message.twist.twist.angular.z = robotYawrate;
            Publish(message);
        }

        private void InitializeMessage()
        {
            message = new MessageTypes.Nav.Odometry();
            message.header.Update();
            message.header.frame_id = frameId;
            message.child_frame_id = name;
            message.pose.pose.position = GetPointMsgFromXYZ(0, 0, 0);
            message.pose.pose.orientation = GetQuaternionMsgFromRPY(0, 0, 0);
        }

        private MessageTypes.Geometry.Point GetPointMsgFromXYZ(float x, float y, float z)
        {
            MessageTypes.Geometry.Point p = new MessageTypes.Geometry.Point();
            p.x = x;
            p.y = y;
            p.z = z;
            return p;
        }

        private MessageTypes.Geometry.Quaternion GetQuaternionMsgFromRPY(float roll, float pitch, float yaw)
        {
            Quaternion rotation = Quaternion.Euler(roll * Mathf.Rad2Deg, pitch * Mathf.Rad2Deg, yaw * Mathf.Rad2Deg);
            MessageTypes.Geometry.Quaternion q = new MessageTypes.Geometry.Quaternion();
            q.x = rotation.Unity2Ros().x;
            q.y = rotation.Unity2Ros().y;
            q.z = rotation.Unity2Ros().z;
            q.w = rotation.Unity2Ros().w;
            return q;
        }
    }
}
