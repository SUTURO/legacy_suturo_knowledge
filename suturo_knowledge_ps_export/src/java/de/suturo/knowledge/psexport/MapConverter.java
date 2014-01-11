package de.suturo.knowledge.psexport;

import ros.NodeHandle;
import ros.Publisher;
import ros.Ros;
import ros.RosException;
import ros.communication.Time;
import ros.pkg.geometry_msgs.msg.Pose;
import ros.pkg.geometry_msgs.msg.Quaternion;
import ros.pkg.moveit_msgs.msg.CollisionObject;
import ros.pkg.shape_msgs.msg.SolidPrimitive;

/**
 * Takes input from a knowrob semantic map, transforms it into Planning Scene
 * compatible messages and publishes them.
 * 
 * @author Moritz
 * 
 */
public class MapConverter {

    private static final String NODE_NAME = "suturo_knowledge_psexport";

    private static Ros ros;
    private static NodeHandle handle;

    public static void main(String... args) {
	checkInitialized();
	MapConverter conv = new MapConverter();
	conv.publishScene();
    }

    public void publishScene() {
	Publisher<CollisionObject> pub = null;
	try {
	    pub = handle.advertise("/collision_object", new CollisionObject(),
		    100);
	    CollisionObject obj = generateDummyCollisionObject();
	    pub.publish(obj);

	    Thread.sleep(1000);

	} catch (RosException e) {
	    e.printStackTrace();

	} catch (InterruptedException e) {
	    e.printStackTrace();

	} finally {
	    if (pub != null)
		pub.shutdown();
	}
    }

    private CollisionObject generateDummyCollisionObject() {
	CollisionObject obj = new CollisionObject();
	obj.id = "table"; // Eindeutiger Name?
	obj.header.stamp = Time.now();
	obj.header.frame_id = "/base_footprint"; // odom_combined?
	obj.operation = CollisionObject.ADD;
	SolidPrimitive sp = new SolidPrimitive();
	sp.type = SolidPrimitive.BOX;
	sp.dimensions = new double[3];
	sp.dimensions[0] = 0.15; // x
	sp.dimensions[1] = 0.07; // y
	sp.dimensions[2] = 0.2; // z
	obj.primitives.add(sp);
	Pose pose = new Pose(); // Auf welchen Punkt bezieht sich das?
	pose.position.x = 0.65;
	pose.position.y = 0.3;
	pose.position.z = 0.621;
	pose.orientation = createMsgForQuaternion(0, 0, -Math.PI / 4);
	obj.primitive_poses.add(pose);
	return obj;
    }

    private static void checkInitialized() {
	ros = Ros.getInstance();
	if (!ros.isInitialized()) {
	    ros.init(NODE_NAME);
	}
	handle = ros.createNodeHandle();
	if (!handle.checkMaster()) {
	    ros.logError("PrologBridge: Ros master not available");
	    throw new IllegalStateException("Ros master not available");
	}

    }

    private static Quaternion createMsgForQuaternion(double roll, double pitch,
	    double yaw) {
	double QUATERNION_TOLERANCE = 0.1f;
	double halfYaw = yaw * 0.5;
	double halfPitch = pitch * 0.5;
	double halfRoll = roll * 0.5;
	double cosYaw = Math.cos(halfYaw);
	double sinYaw = Math.sin(halfYaw);
	double cosPitch = Math.cos(halfPitch);
	double sinPitch = Math.sin(halfPitch);
	double cosRoll = Math.cos(halfRoll);
	double sinRoll = Math.sin(halfRoll);
	Quaternion q = new Quaternion();
	q.x = sinRoll * cosPitch * cosYaw - cosRoll * sinPitch * sinYaw;
	q.y = cosRoll * sinPitch * cosYaw + sinRoll * cosPitch * sinYaw;
	q.z = cosRoll * cosPitch * sinYaw - sinRoll * sinPitch * cosYaw;
	q.w = cosRoll * cosPitch * cosYaw + sinRoll * sinPitch * sinYaw;
	double length2 = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
	if (Math.abs(length2 - 1) > QUATERNION_TOLERANCE) {
	    throw new UnsupportedOperationException(
		    "Non normalized quaternion found. Normalize not yet implemented");
	}
	return q;
    }
}
