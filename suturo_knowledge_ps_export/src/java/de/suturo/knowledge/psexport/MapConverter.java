package de.suturo.knowledge.psexport;

import java.util.LinkedHashMap;
import java.util.Map;

import ros.NodeHandle;
import ros.Publisher;
import ros.Ros;
import ros.RosException;
import ros.communication.Time;
import ros.pkg.moveit_msgs.msg.CollisionObject;
import de.suturo.knowledge.psexport.CollisionObjectWrapper.Operation;

/**
 * Takes input from a knowrob semantic map, transforms it into Planning Scene
 * compatible messages and publishes them.
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

	private Map<String, CollisionObjectWrapper> presentObjects;

	private Map<String, CollisionObjectWrapper> pendingObjects;

	private AttachedObjectClearer aoc;

	/**
	 * Initializes node
	 */
	public MapConverter() {
		checkInitialized();
		try {
			publisher = handle.advertise("/collision_object",
					new CollisionObject(), 100);
		} catch (RosException e) {
			handle.logError("Publisher advertisement failed: " + e.getMessage());
		}
		pendingObjects = new LinkedHashMap<String, CollisionObjectWrapper>();
		handle.logInfo("MapConverter initialized");
	}

	/**
	 * Publishes the new scene, removing all previously published objects
	 * beforehand.
	 */
	public void publishScene() {
		if (!(publisher.isValid() && handle.ok() && ros.ok())) {
			handle.logError("Can't publish scene: Publisher/Node/ROS is not ok!");
		}
		if (presentObjects != null && presentObjects.size() > 0) {
			presentObjects.keySet().retainAll(pendingObjects.keySet());
			for (String key : presentObjects.keySet()) {
				handle.logDebug("Remove object from PS with key: " + key);
				publisher.publish(CollisionObjectWrapper.removeObject(key)
						.toCollisionObject());
			}
		}
		for (CollisionObjectWrapper co : pendingObjects.values()) {
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
	public void addBox(String id, double dimX, double dimY, double dimZ,
			double x, double y, double z, String frameID) {
		CollisionObjectWrapper co = new CollisionObjectWrapper(id,
				Operation.ADD);
		co.setFrame(frameID, Time.now());
		co.addPrimitiveBox(dimX, dimY, dimZ);
		co.addPose(x, y, z, 0, 0, 0);
		addCollisionObject(co);
		handle.logDebug("Add new object with key " + id);
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
	@Deprecated
	public void removeAttachedObject(String id) {
		if (aoc == null) {
			aoc = new AttachedObjectClearer(handle);
		}
		try {
			aoc.clearObject(id);
		} catch (RosException e) {
			handle.logError("Could not remove attached object: "
					+ e.getMessage());
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
