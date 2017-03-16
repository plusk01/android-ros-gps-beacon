package com.magicc.beacon.gps;

import org.ros.message.Time;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.topic.Publisher;

import std_msgs.String;

public class AndroidPhoneSensorNode extends AbstractNodeMain {

    private Publisher<geometry_msgs.Vector3Stamped> FusedOrientation;
    private Publisher<geometry_msgs.TwistStamped> linearAccelertionAndAngVel;

    @Override
    public GraphName getDefaultNodeName() {
        // TODO Auto-generated method stub
        return GraphName.of("android_sensors");
    }

    @Override
    public void onStart(final ConnectedNode connectedNode) {

        FusedOrientation = connectedNode
                .newPublisher("fusedOrientation",geometry_msgs.Vector3Stamped._TYPE);
        linearAccelertionAndAngVel = connectedNode.newPublisher("Twist", geometry_msgs.TwistStamped._TYPE);

    }

    public void publishSensorValues(float[] accel2, float[] gyro2,
                                    float[] fusedOrientation2, Long timestamp) {
        // TODO Auto-generated method stub

        geometry_msgs.TwistStamped msg1 = linearAccelertionAndAngVel.newMessage();
        msg1.getHeader().setStamp(new Time(timestamp.doubleValue()/1000.0));
        msg1.getTwist().getLinear().setX(accel2[0]);
        msg1.getTwist().getLinear().setY(accel2[1]);
        msg1.getTwist().getLinear().setZ(accel2[2]);
        msg1.getTwist().getAngular().setX(gyro2[0]);
        msg1.getTwist().getAngular().setY(gyro2[1]);
        msg1.getTwist().getAngular().setZ(gyro2[2]);
        geometry_msgs.Vector3Stamped msg2 = FusedOrientation.newMessage();
        msg2.getHeader().setStamp(new Time(timestamp.doubleValue()/1000.0));
        msg2.getVector().setX(fusedOrientation2[0]);
        msg2.getVector().setY(fusedOrientation2[1]);
        msg2.getVector().setZ(fusedOrientation2[2]);
        linearAccelertionAndAngVel.publish(msg1);
        FusedOrientation.publish(msg2);

    }






}