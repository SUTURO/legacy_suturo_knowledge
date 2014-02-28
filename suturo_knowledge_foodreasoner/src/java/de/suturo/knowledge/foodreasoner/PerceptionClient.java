package de.suturo.knowledge.foodreasoner;

import java.util.HashMap;
import java.util.Map;

import javax.vecmath.Matrix4d;
import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;

import ros.NodeHandle;
import ros.Ros;
import ros.RosException;
import ros.communication.Time;
import ros.pkg.geometry_msgs.msg.Point;
import ros.pkg.geometry_msgs.msg.Pose;
import ros.pkg.suturo_perception_msgs.msg.PerceivedObject;
import tfjava.Stamped;
import tfjava.TFListener;

/**
 * Bridge from Perception to Prolog
 * 
 * @author Moritz Horstmann
 * 
 */
public class PerceptionClient {

	private static final String NODE_NAME = "suturo_knowledge_perceptionclient";
	private static final int DEFAULT_TIMEOUT_MS = 10000;

	private static Ros ros;
	private static NodeHandle handle;
	private static GetClustersService cluster;
	private static TFListener tf;

	private final Map<Long, Stamped<Point3d>> mapCoords = new HashMap<Long, Stamped<Point3d>>();
	private final Map<Long, Stamped<Point3d>> odomCoords = new HashMap<Long, Stamped<Point3d>>();
	private final Map<Long, Stamped<Pose>> mapCuboid = new HashMap<Long, Stamped<Pose>>();

	/**
	 * Initializes node
	 */
	public PerceptionClient() {
		checkInitialized();
		tf = TFListenerSafe.getInstance();
		handle.logInfo("PerceptionClient initialized");
	}

	private static void checkInitialized() {
		ros = Ros.getInstance();
		if (!ros.isInitialized()) {
			ros.init(NODE_NAME);
		}
		handle = ros.createNodeHandle();
		handle.setMasterRetryTimeout(DEFAULT_TIMEOUT_MS);
		if (!handle.checkMaster()) {
			ros.logError("PerceptionClient: Ros master not available");
			throw new IllegalStateException("Ros master not available");
		}
	}

	/**
	 * Calls perception service and retrieves list of perceived objects
	 * 
	 * @return true if any objects were found
	 * @throws RosException
	 */
	public PerceivedObject[] updatePerception() throws RosException {
		if (cluster == null) {
			cluster = new GetClustersService(handle);
		}
		mapCoords.clear();
		mapCuboid.clear();
		odomCoords.clear();
		PerceivedObject[] pos = cluster.getClusters().toArray(
				new PerceivedObject[0]);
		for (PerceivedObject po : pos) {
			Stamped<Point3d> poPoint = getStamped3DPoint(po);
			Stamped<Matrix4d> poPose = new Stamped<Matrix4d>(
					poseToMatrix4d(po.matched_cuboid.pose), po.frame_id,
					Time.now());
			addTransformPoint("/map", poPoint, mapCoords, po.c_id);
			addTransformPoint("/odom_combined", poPoint, odomCoords, po.c_id);
			addTransformPose("/map", poPose, mapCuboid, po.c_id);
		}
		return pos;
	}

	private static void addTransformPose(String target,
			Stamped<Matrix4d> poPose, Map<Long, Stamped<Pose>> map, long cID) {
		try {
			if (tf.lookupTransform(target, poPose.frameID, poPose.timeStamp) == null) {
				ros.logWarn("Frame with ID " + target + " not found!");
				return;
			}
			Stamped<Matrix4d> out = new Stamped<Matrix4d>();
			out.setData(new Matrix4d());
			tf.transformPose(target, poPose, out);
			Stamped<Pose> pose = new Stamped<Pose>(
					matrix4dToPose(out.getData()), poPose.frameID,
					poPose.timeStamp);
			map.put(Long.valueOf(cID), pose);
		} catch (RuntimeException e) {
			throw e;
		} catch (Exception e) {
			ros.logError("TF failed: " + e);
		}
	}

	private static Matrix4d poseToMatrix4d(Pose pose) {
		Quat4d quat = new Quat4d();
		quat.w = pose.orientation.w;
		quat.x = pose.orientation.x;
		quat.y = pose.orientation.y;
		quat.z = pose.orientation.z;
		Vector3d vec = new Vector3d(pose.position.x, pose.position.y,
				pose.position.z);
		return new Matrix4d(quat, vec, 1);
	}

	private static Pose matrix4dToPose(Matrix4d matrix) {
		Quat4d quat = new Quat4d();
		matrix.get(quat);
		Vector3d vector = new Vector3d();
		matrix.get(quat);
		Pose pose = new Pose();
		pose.orientation.w = quat.w;
		pose.orientation.x = quat.x;
		pose.orientation.y = quat.y;
		pose.orientation.z = quat.z;
		pose.position.x = vector.x;
		pose.position.y = vector.y;
		pose.position.z = vector.z;
		return pose;
	}

	private static Stamped<Point3d> getStamped3DPoint(PerceivedObject po) {
		Point cent = po.c_centroid;
		Point3d point3d = new Point3d(cent.x, cent.y, cent.z);
		return new Stamped<Point3d>(point3d, po.frame_id, Time.now());
	}

	private static void addTransformPoint(String target, Stamped<Point3d> in,
			Map<Long, Stamped<Point3d>> map, long cID) {
		try {
			if (tf.lookupTransform(target, in.frameID, in.timeStamp) == null) {
				ros.logWarn("Frame with ID " + target + " not found!");
				return;
			}
			Stamped<Point3d> out = new Stamped<Point3d>();
			out.setData(new Point3d());
			tf.transformPoint(target, in, out);
			map.put(Long.valueOf(cID), out);
		} catch (RuntimeException e) {
			throw e;
		} catch (Exception e) {
			ros.logError("TF failed: " + e);
		}
	}

	/**
	 * Retrieve transformed coordinates in map frame context.
	 * 
	 * @param c_id
	 *            ID of corresponding PerceivedObject
	 * @return Point3d object
	 */
	public Stamped<Point3d> getMapCoords(long c_id) {
		return this.mapCoords.get(Long.valueOf(c_id));
	}

	/**
	 * Retrieve transformed coordinates in odom_combined frame context.
	 * 
	 * @param c_id
	 *            ID of corresponding PerceivedObject
	 * @return Point3d object
	 */
	public Stamped<Point3d> getOdomCoords(long c_id) {
		return this.odomCoords.get(Long.valueOf(c_id));
	}

	/**
	 * Retrieve transformed pose in map frame context.
	 * 
	 * @param c_id
	 *            ID of corresponding PerceivedObject
	 * @return Pose object
	 */
	public Stamped<Pose> getCuboidPose(long c_id) {
		return this.mapCuboid.get(Long.valueOf(c_id));
	}
}