package de.suturo.knowledge.foodreasoner;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Map.Entry;

import javax.vecmath.Matrix4d;
import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import ros.NodeHandle;
import ros.Ros;
import ros.RosException;
import ros.pkg.geometry_msgs.msg.Point;
import ros.pkg.geometry_msgs.msg.Pose;
import ros.pkg.geometry_msgs.msg.Quaternion;
import ros.pkg.suturo_perception_msgs.msg.PerceivedObject;
import tfjava.Stamped;
import de.suturo.java.serviceclient.GetClustersService;
import de.suturo.knowledge.foodreasoner.classifier.ObjectClassifier;
import de.suturo.knowledge.foodreasoner.classifier.ProbabilityClassifier;
import de.suturo.knowledge.foodreasoner.classifier.WekaClassifier;
import de.suturo.knowledge.foodreasoner.exception.TFException;
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
    private static final String BASE_FRAME = "/map";

    private static Ros ros;
    private static NodeHandle handle;
    private static GetClustersService cluster;

    private ObjectClassifier classifier;
    private final MapConverter mc;

    private final Map<String, RecognizedObject> knownObjects;
    private final Map<String, UnknownObject> unknownObjects;
    private final Map<String, AbstractObject> allObjects;

    static {
	ExceptionHandler.setExcaptionHandler();
    }

    /**
     * Initializes node
     */
    public PerceptionClient() {
	checkInitialized();
	TFListenerSafe.getInstance(); // Init to collect TF data
	try {
	    classifier = new WekaClassifier();
	} catch (Exception e) {
	    handle.logError("PerceptionClient: Could not initialize Weka classifier. Reason: " + e);
	    classifier = new ProbabilityClassifier();
	}
	mc = new MapConverter();
	knownObjects = new HashMap<String, RecognizedObject>();
	unknownObjects = new HashMap<String, UnknownObject>();
	allObjects = new HashMap<String, AbstractObject>();
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
     *            Pose to use as height correction to correctly place objects on a surface in planning scene
     * @param height
     *            Height of surface to place objects on
     * 
     * @return String array of all recognized identifiers
     * 
     * @throws RosException
     */
    public String[] perceive(Pose pose, double height) throws RosException {
	clearPerceived(); // TODO dont remove everything
	PerceivedObject[] pos = updatePerception();
	List<AbstractObject> objs = classifyObjects(pos);
	transformObjects(objs);
	postprocessObjects(objs);
	persistObjects(objs);
	publishPlanningScenes(pose, height);
	return knownObjects.keySet().toArray(new String[0]);
    }

    private void persistObjects(List<AbstractObject> objs) {
	unknownObjects.clear();
	knownObjects.clear();
	List<UnknownObject> uObjs = filter(UnknownObject.class, objs);
	List<RecognizedObject> rObjs = filter(RecognizedObject.class, objs);
	for (UnknownObject u : uObjs) {
	    unknownObjects.put(u.getIdentifier(), u);
	}
	for (RecognizedObject r : rObjs) {
	    knownObjects.put(r.getIdentifier(), r);
	}
	allObjects.clear();
	allObjects.putAll(mapObjects(knownObjects, unknownObjects));
    }

    private void postprocessObjects(List<AbstractObject> objs) {
	for (AbstractObject o : objs) {
	    for (AbstractObject o2 : objs) {
		if (o.equals(o2)) {
		    continue;
		}
		if (o.isSameObject(BASE_FRAME, o2)) {
		    handle.logError("Object " + o + " is too close to object " + o2);
		    // TODO error handling
		    return;
		}
	    }
	    if (o instanceof UnknownObject) {
		for (AbstractObject uk : unknownObjects.values()) {
		    if (uk.isSameObject(BASE_FRAME, o)) {
			o.setIdentifier(uk.getIdentifier());
			// TODO reuse instances, replace original and TF data
		    }
		}
	    }
	    if (o instanceof RecognizedObject) {
		for (AbstractObject ao : allObjects.values()) {
		    if (ao.isSameObject(BASE_FRAME, o) && !ao.getIdentifier().equals(o.getIdentifier())) {
			mc.removeObject(ao.getIdentifier());
			// TODO add quality metric here for better checks
		    }
		}
	    }
	}
    }

    private static void transformObjects(List<AbstractObject> objs) {
	for (AbstractObject object : objs) {
	    try {
		object.transformToFrame(BASE_FRAME);
	    } catch (TFException e) {
		handle.logError("PerceptionClient: " + e.getMessage());
	    }
	}
    }

    /**
     * Remove all keys from keySet from PlanningScene
     * 
     * @param keySet
     */
    private void clearPerceived() {
	for (AbstractObject object : allObjects.values()) {
	    mc.removeObject(object.getIdentifier());
	}
    }

    /**
     * Calls perception service and retrieves list of perceived objects
     * 
     * @throws RosException
     */
    private static PerceivedObject[] updatePerception() throws RosException {
	if (cluster == null) {
	    cluster = new GetClustersService(handle);
	}
	return cluster.getClusters().toArray(new PerceivedObject[0]);
    }

    /**
     * Pass perceived objects to classifier and assign c_id with identifier
     * 
     * @param pos
     *            PercievedObject list
     * @return List of objects
     */
    private List<AbstractObject> classifyObjects(PerceivedObject[] pos) {
	ArrayList<AbstractObject> newRecogObj = new ArrayList<AbstractObject>();
	for (PerceivedObject po : pos) {
	    String object = classifier.classifyPerceivedObject(po);
	    if (object != null) {
		newRecogObj.add(new RecognizedObject(po, object));
	    } else {
		newRecogObj.add(new UnknownObject(po));
	    }
	}
	return newRecogObj;
    }

    private static Map<String, AbstractObject> mapObjects(Map<String, ? extends AbstractObject> a,
	    Map<String, ? extends AbstractObject> b) {
	Map<String, AbstractObject> out = new HashMap<String, AbstractObject>(a.size() + b.size());
	out.putAll(a);
	out.putAll(b);
	return out;
    }

    /**
     * Publishes recognized objects to the planning scene
     * 
     * @param pose
     *            Height correction pose
     */
    private void publishPlanningScenes(Pose pose, double height) {
	for (Entry<String, AbstractObject> entry : allObjects.entrySet()) {
	    String frame = entry.getValue().getOriginalFrameID();
	    Stamped<Pose> objectPose = entry.getValue().getTransformedPose(frame);
	    if (objectPose == null) {
		handle.logWarn("PerceptionClient: No transformed cuboid available for object " + entry.getKey());
		continue;
	    }
	    Point pos = objectPose.getData().position;
	    Quaternion or = objectPose.getData().orientation;
	    Vector3d dim = entry.getValue().getCuboidDim();
	    // TODO outsource object correction
	    if (pose != null) {
		pos.z = pose.position.z + (height / 2) + (dim.y / 2) + 0.01;
	    }
	    mc.addBox(entry.getKey(), dim.x, dim.y, dim.z, pos.x, pos.y, pos.z, or, frame);
	}
	mc.publishScene();
    }

    /**
     * Retrieve transformed coordinates in map frame context.
     * 
     * @param identifier
     *            Object identifier
     * @return Point3d object
     */
    public Stamped<Point3d> getMapCoords(String identifier) {
	return allObjects.get(identifier).getTransformedCentroid(BASE_FRAME);
    }

    /**
     * Retrieve transformed pose in map frame context.
     * 
     * @param identifier
     *            Object identifier
     * @return Pose object
     */
    public Stamped<Matrix4d> getCuboidMatrix(String identifier) {
	return allObjects.get(identifier).getTransformedMatrix(BASE_FRAME);
    }

    @SuppressWarnings("unchecked")
    private static <T> List<T> filter(Class<T> clazz, List<?> in) {
	List<T> list = new ArrayList<T>();
	for (Object o : in) {
	    if (clazz.isInstance(o)) {
		list.add((T) o);
	    }
	}
	return list;
    }

    /**
     * Add semantic perception to PlanningScene. Currently only cuboids are supported. These perception won't be cleared
     * by {@link #clearPlanningScene()}, so be careful what you add!
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
    public void addSemanticPerception(String objectName, String frameID, double dimX, double dimY, double dimZ,
	    Pose pose) {
	mc.addStaticBox(objectName, dimX, dimY, dimZ, pose.position.x, pose.position.y, pose.position.z,
		pose.orientation, frameID);
    }

    /**
     * Clear perceived objects manually.
     */
    public void clearPlanningScene() {
	mc.removeAttachedObjects();
	clearPerceived();
	mc.publishScene();
	clearPerception();
    }

    private void clearPerception() {
	allObjects.clear();
	knownObjects.clear();
	unknownObjects.clear();
    }

    /**
     * Remove object with specified id from planning scene
     * 
     * @param id
     *            Object ID
     */
    public void removeObjectFromPS(String id) {
	// TODO remove from Maps, too?
	mc.removeObject(id);
	mc.publishScene();
    }

    /**
     * Utility method to convert prolog list representing a rotation matrix to Pose object. Parameters represent
     * m[0..3][0..3] values from knowrob OWL class
     * 
     * @return Pose object
     */
    public static Pose prologMatrix4dToPose(double m00, double m01, double m02, double m03, double m10, double m11,
	    double m12, double m13, double m20, double m21, double m22, double m23, double m30, double m31, double m32,
	    double m33) {
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
	return TransformUtils.matrix4dToPose(mat);
    }

}