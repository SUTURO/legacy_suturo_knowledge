package de.suturo.knowledge.psexport;

import java.util.Collection;
import java.util.Iterator;
import java.util.LinkedHashMap;
import java.util.Map;

import ros.NodeHandle;
import ros.Publisher;
import ros.Ros;
import ros.RosException;
import ros.communication.Time;
import ros.pkg.geometry_msgs.msg.Quaternion;
import ros.pkg.moveit_msgs.msg.CollisionObject;
import de.suturo.knowledge.psexport.CollisionObjectWrapper.Operation;

/**
 * Takes input from a knowrob semantic map, transforms it into Planning Scene compatible messages and publishes them.
 * 
 * @author Moritz
 * 
 */
public class MapConverter {

    private static final String NODE_NAME = "suturo_knowledge_psexport";
    private static final int DEFAULT_TIMEOUT_MS = 10000;

    private static Ros ros;
    private NodeHandle handle;
    private Publisher<CollisionObject> publisher;
    private static final Time ZEROTIME = new Time(0, 0);

    private Map<String, CollisionObjectWrapper> presentObjects;
    private Map<String, CollisionObjectWrapper> pendingObjects;
    private AttachedObjectClearer aoc;

    /**
     * Initializes node
     */
    public MapConverter() {
	checkInitialized();
	try {
	    publisher = handle.advertise("/collision_object", new CollisionObject(), 100);
	    pendingObjects = new LinkedHashMap<String, CollisionObjectWrapper>();
	    handle.logInfo("MapConverter initialized");
	} catch (RosException e) {
	    handle.logError("Publisher advertisement failed: " + e.getMessage());
	}
    }

    /**
     * Publishes the new scene, removing all previously published objects beforehand.
     */
    public void publishScene() {
	if (!(publisher.isValid() && handle.ok() && ros.ok())) {
	    handle.logError("Can't publish scene: Publisher/Node/ROS is not ok!");
	}
	if (presentObjects != null && presentObjects.size() > 0) {
	    presentObjects.keySet().retainAll(pendingObjects.keySet());
	    for (String key : presentObjects.keySet()) {
		handle.logDebug("Remove object from PS with key: " + key);
		publisher.publish(CollisionObjectWrapper.removeObject(key).toCollisionObject());
	    }
	}
	Collection<CollisionObjectWrapper> objects = pendingObjects.values();
	Iterator<CollisionObjectWrapper> i = objects.iterator();
	while (i.hasNext()) {
	    CollisionObjectWrapper co = i.next();
	    if (co.getOperation() == Operation.REMOVE) {
		i.remove();
		continue; // Removal was done in loop before
	    }
	    handle.logDebug("Publish object with key: " + co.getId());
	    publisher.publish(co.toCollisionObject());
	}
	presentObjects = pendingObjects;
	pendingObjects = new LinkedHashMap<String, CollisionObjectWrapper>();
	handle.logInfo("Published new PlanningScene");

    }

    /**
     * Adds a collision object to the pending scene
     * 
     * @param co
     *            Collision object
     */
    void addCollisionObject(CollisionObjectWrapper co) {
	pendingObjects.put(co.getId(), co);
    }

    /**
     * Adds a collision object to the pending scene
     * 
     * @param id
     * @param dimX
     * @param dimY
     * @param dimZ
     * @param x
     * @param y
     * @param z
     * @param frameID
     */
    public void addBox(String id, double dimX, double dimY, double dimZ, double x, double y, double z, String frameID) {
	CollisionObjectWrapper co = new CollisionObjectWrapper(id, Operation.ADD);
	co.setFrame(frameID, ZEROTIME);
	co.addPrimitiveBox(dimX, dimY, dimZ);
	co.addPose(x, y, z, 0, 0, 0);
	addCollisionObject(co);
	handle.logDebug("Add new object with key " + id);
    }

    /**
     * Adds a collision object to the pending scene
     * 
     * @param id
     * @param dimX
     * @param dimY
     * @param dimZ
     * @param x
     * @param y
     * @param z
     * @param quat
     * @param frameID
     */

    public void addBox(String id, double dimX, double dimY, double dimZ, double x, double y, double z, Quaternion quat,
	    String frameID) {
	CollisionObjectWrapper co = new CollisionObjectWrapper(id, Operation.ADD);
	co.setFrame(frameID, ZEROTIME);
	co.addPrimitiveBox(dimX, dimY, dimZ);
	co.addPose(x, y, z, quat);
	addCollisionObject(co);
	handle.logDebug("Add new object with key " + id);
    }

    /**
     * Adds a static collision object to the pending scene. These are unaffected by any automatic cleanup procedures.
     * 
     * @param id
     * @param dimX
     * @param dimY
     * @param dimZ
     * @param x
     * @param y
     * @param z
     * @param quat
     * @param frameID
     */

    public void addStaticBox(String id, double dimX, double dimY, double dimZ, double x, double y, double z,
	    Quaternion quat, String frameID) {
	CollisionObjectWrapper co = new CollisionObjectWrapper(id, Operation.ADD);
	co.setFrame(frameID, ZEROTIME);
	co.addPrimitiveBox(dimX, dimY, dimZ);
	co.addPose(x, y, z, quat);
	publisher.publish(co.toCollisionObject());
	handle.logDebug("Published static object with key " + id);
    }

    /**
     * Removes a collision object from the pending scene
     * 
     * @param id
     */
    public void removeObject(String id) {
	addCollisionObject(CollisionObjectWrapper.removeObject(id));
    }

    /**
     * Removes a collision object from the pending scene
     * 
     * @param id
     */
    public void removeAttachedObjects() {
	if (aoc == null) {
	    aoc = new AttachedObjectClearer(handle);
	}
	try {
	    aoc.clearAllObjects();
	} catch (RosException e) {
	    handle.logError("Could not remove attached objects: " + e.getMessage());
	}
    }

    /**
     * Clean up all ros handles
     */
    public void shutdown() {
	if (publisher != null)
	    publisher.shutdown();

	if (handle != null) {
	    handle.shutdown();
	}
    }

    private void checkInitialized() {
	ros = Ros.getInstance();
	if (!ros.isInitialized()) {
	    ros.init(NODE_NAME);
	}
	handle = ros.createNodeHandle();
	handle.setMasterRetryTimeout(DEFAULT_TIMEOUT_MS);
	if (!handle.checkMaster()) {
	    ros.logError("MapConverter: Ros master not available");
	    throw new IllegalStateException("Ros master not available");
	}
    }

}
