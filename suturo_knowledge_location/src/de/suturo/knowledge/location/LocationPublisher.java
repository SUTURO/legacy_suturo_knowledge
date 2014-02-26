package de.suturo.knowledge.location;

import javax.vecmath.Matrix4d;
import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;

import ros.NodeHandle;
import ros.Publisher;
import ros.Ros;
import ros.RosException;
import ros.communication.Time;
import ros.pkg.geometry_msgs.msg.PoseStamped;
import tfjava.Stamped;
import tfjava.TFListener;

/**
 * Listens to TF robot location and publishes them to a topic
 * 
 * @author moritz
 * 
 */
public class LocationPublisher {

	private static final String TOPIC = "/suturo/robot_location";
	private static final String NODE_NAME = "suturo_knowledge_locationpublisher";
	private static final String TARGET_DEFAULT = "/map";
	private static final double PUBLISH_RATE = 10;
	private static final int DEFAULT_TIMEOUT_MS = 10000;

	private static Ros ros;
	private NodeHandle handle;
	private String target;
	private Publisher<PoseStamped> publisher;
	private double rate;
	private static TFListener tf;

	/**
	 * Main
	 * 
	 * @param args
	 */
	public static void main(String... args) {
		LocationPublisher pub = new LocationPublisher();
		pub.setArgs(args);
		pub.publish();
	}

	private void setArgs(String[] args) {
		target = TARGET_DEFAULT;
		rate = PUBLISH_RATE;
	}

	private void publish() {
		Matrix4d matrix = new Matrix4d();
		Stamped<Matrix4d> in;
		Stamped<Matrix4d> out = new Stamped<Matrix4d>();
		out.setData(new Matrix4d());
		long seq = 0;
		while (ros.ok()) {
			in = new Stamped<Matrix4d>(matrix, "/odom_combined", Time.now());
			if (tf.lookupTransform(target, in.frameID, in.timeStamp) == null) {
				ros.logWarn("Frame with ID " + target + " not found!");
			} else {
				tf.transformPose(target, in, out);
				PoseStamped pose = new PoseStamped();
				pose.header.frame_id = target;
				pose.header.stamp = out.timeStamp;
				pose.header.seq = ++seq;
				Quat4d quat = new Quat4d();
				out.getData().get(quat);
				pose.pose.orientation.w = quat.w;
				pose.pose.orientation.x = quat.x;
				pose.pose.orientation.y = quat.y;
				pose.pose.orientation.z = quat.z;
				Vector3d vec = new Vector3d();
				out.getData().get(vec);
				pose.pose.position.x = vec.x;
				pose.pose.position.y = vec.y;
				pose.pose.position.z = vec.z;
				publisher.publish(pose);
			}
			try {
				Thread.sleep((long) (1000d / rate));
			} catch (InterruptedException e) {
				ros.logWarn("LocationPublisher: Received interrupt");
			}
		}
	}

	/**
	 * Initializes node
	 */
	public LocationPublisher() {
		checkInitialized();
		try {
			publisher = handle.advertise(TOPIC, new PoseStamped(), 100);
		} catch (RosException e) {
			handle.logError("Publisher advertisement failed: " + e.getMessage());
		}
		tf = TFListener.getInstance();
		handle.logInfo("LocationPublisher initialized");
	}

	private void checkInitialized() {
		ros = Ros.getInstance();
		if (!ros.isInitialized()) {
			ros.init(NODE_NAME);
		}
		handle = ros.createNodeHandle();
		handle.setMasterRetryTimeout(DEFAULT_TIMEOUT_MS);
		if (!handle.checkMaster()) {
			ros.logError("LocationPublisher: Ros master not available");
			throw new IllegalStateException("Ros master not available");
		}
	}
}
