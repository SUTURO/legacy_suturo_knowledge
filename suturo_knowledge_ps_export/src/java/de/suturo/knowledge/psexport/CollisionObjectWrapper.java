package de.suturo.knowledge.psexport;

import java.util.ArrayList;

import ros.communication.Time;
import ros.pkg.geometry_msgs.msg.Pose;
import ros.pkg.geometry_msgs.msg.Quaternion;
import ros.pkg.moveit_msgs.msg.CollisionObject;
import ros.pkg.shape_msgs.msg.SolidPrimitive;

/**
 * Wraps auto generated collision object message in to something more usable
 * 
 * @author Moritz
 * 
 */
public class CollisionObjectWrapper {

	/**
	 * Enum to determine the operation
	 * 
	 * @author Moritz
	 * 
	 */
	public enum Operation {
		/** Add collision object */
		ADD((short) 0),
		/** Remove collision object */
		REMOVE((short) 1),
		/** Append collision object */
		APPEND((short) 2),
		/** Move collision object */
		MOVE((short) 3);

		private final short op;

		Operation(short op) {
			this.op = op;
		}

		/** Return operation specific short representing the operation */
		short getOp() {
			return this.op;
		}
	}

	private final CollisionObject co;
	private final String id;
	private Operation op;
	private String frameID;
	private Time time;
	private ArrayList<Pose> poses;
	private ArrayList<SolidPrimitive> primitives;

	/**
	 * Creates a new collision object
	 * 
	 * @param id
	 *            Unique (in ROS instance) ID
	 */
	public CollisionObjectWrapper(String id) {
		co = new CollisionObject();
		this.id = id;
	}

	/**
	 * Creates a new collision object with given ID and {@link Operation}.
	 * 
	 * @param id
	 *            Unique (in ROS instance) ID
	 * @param op
	 *            Operation
	 */
	public CollisionObjectWrapper(String id, Operation op) {
		this(id);
		this.op = op;
	}

	/**
	 * Simple method to create a Collision Object message which removes the
	 * object with the given id
	 * 
	 * @param id
	 *            Unique (in ROS instance) ID
	 * @return CollisionObjectWrapper
	 */
	public static CollisionObjectWrapper removeObject(String id) {
		return new CollisionObjectWrapper(id, Operation.REMOVE);
	}

	/**
	 * Add a new box to object
	 * 
	 * @param x
	 * @param y
	 * @param z
	 */
	public void addPrimitiveBox(double x, double y, double z) {
		if (primitives == null) {
			primitives = new ArrayList<SolidPrimitive>();
		}
		SolidPrimitive sp = new SolidPrimitive();
		sp.type = SolidPrimitive.BOX;
		sp.dimensions = new double[3];
		sp.dimensions[0] = x;
		sp.dimensions[1] = y;
		sp.dimensions[2] = z;
		primitives.add(sp);
	}

	/**
	 * Add a new cone to object
	 * 
	 * @param height
	 * @param radius
	 */
	public void addPrimitiveCone(double height, double radius) {
		if (primitives == null) {
			primitives = new ArrayList<SolidPrimitive>();
		}
		SolidPrimitive sp = new SolidPrimitive();
		sp.type = SolidPrimitive.CONE;
		sp.dimensions = new double[2];
		sp.dimensions[0] = height;
		sp.dimensions[1] = radius;
		primitives.add(sp);
	}

	/**
	 * Add a new cylinder to object
	 * 
	 * @param height
	 * @param radius
	 */
	public void addPrimitiveCylinder(double height, double radius) {
		if (primitives == null) {
			primitives = new ArrayList<SolidPrimitive>();
		}
		SolidPrimitive sp = new SolidPrimitive();
		sp.type = SolidPrimitive.CYLINDER;
		sp.dimensions = new double[2];
		sp.dimensions[0] = height;
		sp.dimensions[1] = radius;
		primitives.add(sp);
	}

	/**
	 * Add a new sphere to object
	 * 
	 * @param radius
	 */
	public void addPrimitiveSphere(double radius) {
		if (primitives == null) {
			primitives = new ArrayList<SolidPrimitive>();
		}
		SolidPrimitive sp = new SolidPrimitive();
		sp.type = SolidPrimitive.SPHERE;
		sp.dimensions = new double[1];
		sp.dimensions[0] = radius;
		primitives.add(sp);
	}

	/**
	 * Add a pose to list
	 * 
	 * @param posX
	 * @param posY
	 * @param posZ
	 * @param roll
	 * @param pitch
	 * @param yaw
	 * @param quat
	 */
	public void addPose(double posX, double posY, double posZ, double roll,
			double pitch, double yaw) {
		if (poses == null) {
			poses = new ArrayList<Pose>();
		}
		Pose pose = new Pose();
		pose.position.x = posX;
		pose.position.y = posY;
		pose.position.z = posZ;
		pose.orientation = Util.createMsgForQuaternion(roll, pitch, yaw);
		poses.add(pose);
	}

	/**
	 * Add a pose to list
	 * 
	 * @param posX
	 * @param posY
	 * @param posZ
	 * @param quat
	 */
	public void addPose(double posX, double posY, double posZ, Quaternion quat) {
		if (poses == null) {
			poses = new ArrayList<Pose>();
		}
		Pose pose = new Pose();
		pose.position.x = posX;
		pose.position.y = posY;
		pose.position.z = posZ;
		pose.orientation = quat;
		poses.add(pose);
	}

	/**
	 * Set frame parameters
	 * 
	 * @param frameID
	 *            Frame ID
	 * @param time
	 *            ROS time
	 */
	public void setFrame(String frameID, Time time) {
		this.frameID = frameID;
		this.time = time;
	}

	private void sanityCheck() throws IncompleteValueException {
		switch (op) {
		case ADD:
			co.header.frame_id = check(frameID, "frameID");
			co.header.stamp = check(time, "time");
			co.primitive_poses = check(poses, "Poses");
			co.primitives = check(primitives, "Primitives");
			//$FALL-THROUGH$
		default:
			co.id = check(id, "id");
			co.operation = check(op, "Operation").getOp();
		}
	}

	private static class IncompleteValueException extends Exception {
		private final String missing;

		public String getMissing() {
			return missing;
		}

		public IncompleteValueException(String missing) {
			this.missing = missing;
		}

		private static final long serialVersionUID = -1938760844310534426L;
	}

	private static <T> T check(T in, String field)
			throws IncompleteValueException {
		if (in == null) {
			throw new IncompleteValueException(field);
		}
		return in;
	}

	/**
	 * @return the CollisionObject unique id
	 */
	public String getId() {
		return id;
	}

	/**
	 * Generates a {@link CollisionObject} message while performing sanity
	 * checks, assuring completeness.
	 * 
	 * @return CollisionObject
	 */
	public CollisionObject toCollisionObject() {
		try {
			sanityCheck();
		} catch (IncompleteValueException e) {
			throw new IllegalStateException("Missing attribute: "
					+ e.getMissing());
		}
		return co;
	}

}
