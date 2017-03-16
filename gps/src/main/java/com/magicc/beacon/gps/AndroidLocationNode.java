package com.magicc.beacon.gps;

import org.ros.message.Time;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.topic.Publisher;

import android.location.Location;

/**
 * A ROS node that publishes Android location updates as
 * sensor_msgs/NavSatFix messages.
 *
 * @author Jesse Rosalia
 *
 */
public class AndroidLocationNode extends AbstractNodeMain {

    private Publisher<sensor_msgs.NavSatFix> fixPublisher;
    private String nodeName;

    public AndroidLocationNode(String nodeName) {
        if (nodeName != null && !nodeName.isEmpty()) {
            this.nodeName = nodeName;
        } else {
            this.nodeName = "android";
        }
    }

    /** Set the name of the node */
    @Override
    public GraphName getDefaultNodeName() {
        return GraphName.of(this.nodeName);
    }

    @Override
    public void onStart(final ConnectedNode connectedNode) {
        fixPublisher = connectedNode
                .newPublisher("~fix", sensor_msgs.NavSatFix._TYPE);
    }

    public void updateLocation(Location location) {
        // To prevent crashing when orientation changes
        if (fixPublisher == null) return;

        //TODO: fill out rest of nav information, and also publish velocity and time_reference
//		sensor_msgs.NavSatStatus.
        sensor_msgs.NavSatFix msg = fixPublisher.newMessage();
        //time since jan 1 1970...convert msec to seconds (double)
        msg.getHeader().setStamp(new Time(location.getTime()/1000.0));
        //NOTE: this may not be right...android reports alt above sealevel, ROS expects altitude above ellipsoid
        msg.setAltitude(location.getAltitude());
        msg.setLatitude(location.getLatitude());
        msg.setLongitude(location.getLongitude());
//		sensor_msgs.NavSatStatus status;
//		status.
        //TODO: accuracy, velocity (Twist)
        // TODO Auto-generated method stub
        fixPublisher.publish(msg);
    }
}