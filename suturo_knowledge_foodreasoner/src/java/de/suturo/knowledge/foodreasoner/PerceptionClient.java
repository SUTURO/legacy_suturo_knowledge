package de.suturo.knowledge.foodreasoner;

import java.util.HashMap;
import java.util.Map;
import java.util.Map.Entry;
import java.util.Set;

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
import ros.pkg.geometry_msgs.msg.Quaternion;
import ros.pkg.suturo_perception_msgs.msg.PerceivedObject;
import tfjava.Stamped;
import tfjava.TFListener;
import de.suturo.java.serviceclient.GetClustersService;
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
	private final Map<Long, Vector3d> mapDim = new HashMap<Long, Vector3d>();

	private ObjectClassifier classifier;
	private final MapConverter mc;

	/**
	 * Initializes node
	 */
	public PerceptionClient() {
		checkInitialized();
		tf = TFListenerSafe.getInstance();
		try {
			classifier = new WekaClassifier();
		} catch (Exception e) {
			handle.logError("PerceptionClient: Could not initialize Weka classifier. Reason: "
					+ e);
			classifier = new ProbabilityClassifier();
		}
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
	 * @param pose
	 *            Pose to use as height correction to correctly place objects on
	 *            a surface in planning scene
	 * @param height
	 *            Height of surface to place objects on
	 * 
	 * @return String array of all recognized identifiers
	 * 
	 * @throws RosException
	 */
	public String[] perceive(Pose pose, double height) throws RosException {
		clearPerceived(identifierToID.keySet());
		PerceivedObject[] pos = updatePerception();
		classifyObjects(pos);
		publishPlanningScenes(pose, height);
		return identifierToID.keySet().toArray(new String[0]);
	}

	/**
	 * Remove all keys from keySet from PlanningScene
	 * 
	 * @param keySet
	 */
	private void clearPerceived(Set<String> keySet) {
		for (String key : keySet) {
			mc.removeObject(key);
		}
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
		clearPerception();
		PerceivedObject[] pos = cluster.getClusters().toArray(
				new PerceivedObject[0]);
		for (PerceivedObject po : pos) {
			Stamped<Point3d> poPoint = getStamped3DPoint(po);
			Stamped<Matrix4d> poPose = new Stamped<Matrix4d>(
					poseToMatrix4d(po.matched_cuboid.pose), po.frame_id,
					Time.now());
			// Cuboid length1 = Width of object
			// Cuboid length2 = Height of object
			// Cuboid length3 = Depth of object
			mapDim.put(Long.valueOf(po.c_id), new Vector3d(
					po.matched_cuboid.length1, po.matched_cuboid.length2,
					po.matched_cuboid.length3));
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
			String object = classifier.classifyPerceivedObject(po);
			if (object != null) {
				identifierToID.put(object, Long.valueOf(po.c_id));
			}
		}
	}

	/**
	 * Publishes recognized objects to the planning scene
	 * 
	 * @param pose
	 *            Height correction pose
	 */
	private void publishPlanningScenes(Pose pose, double height) {
		for (Entry<String, Long> entry : identifierToID.entrySet()) {
			Stamped<Pose> stampedPoint = mapCuboid.get(entry.getValue());
			if (stampedPoint == null) {
				handle.logWarn("PerceptionClient: No transformed cuboid available for object "
						+ entry.getKey());
				continue;
			}
			Point pos = stampedPoint.getData().position;
			Quaternion or = stampedPoint.getData().orientation;
			Vector3d dim = mapDim.get(entry.getValue());
			if (pose != null) {
				pos.z = pose.position.z + (height / 2) + (dim.y / 2) + 0.01;
			}
			mc.addBox(entry.getKey(), dim.x, dim.y, dim.z, pos.x, pos.y, pos.z,
					or, "/map");
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
					matrix4dToPose(out.getData()), target, poPose.timeStamp);
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
		matrix.get(vector);
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
	 * Utility method to convert prolog list representing a rotation matrix to
	 * Pose object
	 * 
	 * @param m00
	 * @param m01
	 * @param m02
	 * @param m03
	 * @param m10
	 * @param m11
	 * @param m12
	 * @param m13
	 * @param m20
	 * @param m21
	 * @param m22
	 * @param m23
	 * @param m30
	 * @param m31
	 * @param m32
	 * @param m33
	 * @return Pose object
	 */
	public static Pose prologMatrix4dToPose(double m00, double m01, double m02,
			double m03, double m10, double m11, double m12, double m13,
			double m20, double m21, double m22, double m23, double m30,
			double m31, double m32, double m33) {
		Matrix4d mat = new Matrix4d();
		mat.m00 = m00;
		mat.m01 = m01;
		mat.m02 = m02;
		mat.m03 = m03;
		mat.m10 = m10;
		mat.m11 = m11;
		mat.m12 = m12;
		mat.m13 = m13;
		mat.m20 = m20;
		mat.m21 = m21;
		mat.m22 = m22;
		mat.m23 = m23;
		mat.m30 = m30;
		mat.m31 = m31;
		mat.m32 = m32;
		mat.m33 = m33;
		return matrix4dToPose(mat);
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
	public Stamped<Matrix4d> getCuboidMatrix(String identifier) {
		Stamped<Pose> pose = this.mapCuboid.get(identifierToID.get(identifier));
		return new Stamped<Matrix4d>(poseToMatrix4d(pose.getData()),
				pose.frameID, pose.timeStamp);
	}

	/**
	 * Add semantic perception to PlanningScene. Currently only cuboids are
	 * supported. These perception won't be cleared by
	 * {@link #clearPlanningScene()}, so be careful what you add!
	 * 
	 * @param objectName
	 *            Object instance name
	 * @param frameID
	 *            FrameID of pose
	 * @param dimX
	 * @param dimY
	 * @param dimZ
	 * @param pose
	 *            Pose object as returned by prologMatrix4dToPose
	 */
	public void addSemanticPerception(String objectName, String frameID,
			double dimX, double dimY, double dimZ, Pose pose) {
		mc.addStaticBox(objectName, dimX, dimY, dimZ, pose.position.x,
				pose.position.y, pose.position.z, pose.orientation, frameID);
	}

	/**
	 * Clear perceived objects manually.
	 */
	public void clearPlanningScene() {
		mc.removeAttachedObjects();
		clearPerceived(identifierToID.keySet());
		mc.publishScene();
		clearPerception();
	}

	private void clearPerception() {
		mapCoords.clear();
		mapCuboid.clear();
		mapDim.clear();
		identifierToID.clear();
	}

	/**
	 * Remove object with specified id from planning scene
	 * 
	 * @param id
	 *            Object ID
	 */
	public void removeObjectFromPS(String id) {
		mc.removeObject(id);
		mc.publishScene();
	}

}