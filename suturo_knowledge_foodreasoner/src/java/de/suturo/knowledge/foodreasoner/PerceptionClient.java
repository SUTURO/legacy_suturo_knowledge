package de.suturo.knowledge.foodreasoner;

import java.util.HashMap;
import java.util.Map;
import java.util.Map.Entry;

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
import de.suturo.knowledge.psexport.MapConverter;

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

	private final Map<String, Long> identifierToID = new HashMap<String, Long>();
	private final Map<Long, Stamped<Point3d>> mapCoords = new HashMap<Long, Stamped<Point3d>>();
	private final Map<Long, Stamped<Pose>> mapCuboid = new HashMap<Long, Stamped<Pose>>();

	private final ObjectClassifier classifier;
	private final MapConverter mc;

	/**
	 * Prolog benötigt:
	 * 
	 * #identifier<br />
	 * #koordinaten in map<br />
	 * #pose in map <br />
	 */

	/**
	 * Initializes node
	 */
	public PerceptionClient() {
		checkInitialized();
		tf = TFListenerSafe.getInstance();
		// classifier = new WekaClassifier();
		classifier = new ProbabilityClassifier();
		mc = new MapConverter();
		handle.logInfo("PerceptionClient initialized");
	}

	private static void checkInitialized() {
		ros = Ros.getInstance();
		if (!ros.isInitialized()) {
			ros.init(NODE_NAME);
		}
		handle = ros.createNodeHandle();
		handle.logInfo("Initialize PerceptionClient");
		handle.setMasterRetryTimeout(DEFAULT_TIMEOUT_MS);
		if (!handle.checkMaster()) {
			ros.logError("PerceptionClient: Ros master not available");
			throw new IllegalStateException("Ros master not available");
		}
	}

	/**
	 * Calls the perception service and runs classifiers on the results.
	 * 
	 * @return String array of all recognized identifiers
	 * 
	 * @throws RosException
	 */
	public String[] percieve() throws RosException {
		PerceivedObject[] pos = updatePerception();
		classifyObjects(pos);
		publishPlanningScenes();
		return identifierToID.keySet().toArray(new String[0]);
	}

	/**
	 * Calls perception service and retrieves list of perceived objects
	 * 
	 * @throws RosException
	 */
	private PerceivedObject[] updatePerception() throws RosException {
		if (cluster == null) {
			cluster = new GetClustersService(handle);
		}
		mapCoords.clear();
		mapCuboid.clear();
		identifierToID.clear();
		PerceivedObject[] pos = cluster.getClusters().toArray(
				new PerceivedObject[0]);
		for (PerceivedObject po : pos) {
			Stamped<Point3d> poPoint = getStamped3DPoint(po);
			Stamped<Matrix4d> poPose = new Stamped<Matrix4d>(
					poseToMatrix4d(po.matched_cuboid.pose), po.frame_id,
					Time.now());
			addTransformPoint("/map", poPoint, mapCoords, po.c_id);
			addTransformPose("/map", poPose, mapCuboid, po.c_id);
		}
		return pos;
	}

	/**
	 * Pass perceived objects to classifier and assign c_id with identifier
	 * 
	 * @param pos
	 *            PercievedObject list
	 */
	private void classifyObjects(PerceivedObject[] pos) {
		for (PerceivedObject po : pos) {
			identifierToID.put(classifier.classifyPerceivedObject(
					(int) po.c_color_average_h, po.c_volume), Long
					.valueOf(po.c_id));
		}
	}

	/**
	 * Publishes recognized objects to the planning scene
	 */
	private void publishPlanningScenes() {
		for (Entry<String, Long> entry : identifierToID.entrySet()) {
			Point pos = mapCuboid.get(entry.getValue()).getData().position;
			mc.addBox(entry.getKey(), 0, 0, 0, pos.x, pos.y, pos.z, "/map");
		}
		mc.publishScene();
	}

	/**
	 * Transform stamped pose to a pose in TF frame denoted by target parameter
	 * 
	 * @param target
	 *            Target TF frame ID
	 * @param poPose
	 *            Input Pose
	 * @param map
	 *            Map to write pose in
	 * @param cID
	 *            ID assigned by perception
	 */
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

	/**
	 * Utility method to convert a Pose to Matrix4d object
	 * 
	 * @param pose
	 *            Pose object
	 * @return Matrix4d object
	 */
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

	/**
	 * Utility method to convert a Matrix4d to Pose object
	 * 
	 * @param matrix
	 *            Matrix4d object
	 * @return Pose object
	 */
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

	/**
	 * Utility method to convert a PerceivedObject centroid to
	 * Stamped&lt;Point3d&gt; object
	 * 
	 * @param po
	 *            PerceivedObject object
	 * @return Stamped&lt;Point3d&gt; object
	 */
	private static Stamped<Point3d> getStamped3DPoint(PerceivedObject po) {
		Point cent = po.c_centroid;
		Point3d point3d = new Point3d(cent.x, cent.y, cent.z);
		return new Stamped<Point3d>(point3d, po.frame_id, Time.now());
	}

	/**
	 * Transform stamped point to a point in TF frame denoted by target
	 * parameter
	 * 
	 * @param target
	 *            Target TF frame ID
	 * @param poPose
	 *            Input Point
	 * @param map
	 *            Map to write pose in
	 * @param cID
	 *            ID assigned by perception
	 */
	private static void addTransformPoint(String target,
			Stamped<Point3d> poPoint, Map<Long, Stamped<Point3d>> map, long cID) {
		try {
			if (tf.lookupTransform(target, poPoint.frameID, poPoint.timeStamp) == null) {
				ros.logWarn("Frame with ID " + target + " not found!");
				return;
			}
			Stamped<Point3d> out = new Stamped<Point3d>();
			out.setData(new Point3d());
			tf.transformPoint(target, poPoint, out);
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
	 * @param identifier
	 *            Object identifier
	 * @return Point3d object
	 */
	public Stamped<Point3d> getMapCoords(String identifier) {
		return this.mapCoords.get(identifierToID.get(identifier));
	}

	/**
	 * Retrieve transformed pose in map frame context.
	 * 
	 * @param identifier
	 *            Object identifier
	 * @return Pose object
	 */
	public Stamped<Pose> getCuboidPose(String identifier) {
		return this.mapCuboid.get(identifierToID.get(identifier));
	}
}