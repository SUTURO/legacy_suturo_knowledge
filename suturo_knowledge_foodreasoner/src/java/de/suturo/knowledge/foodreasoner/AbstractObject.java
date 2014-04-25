package de.suturo.knowledge.foodreasoner;

import java.util.HashMap;
import java.util.Map;

import javax.vecmath.Matrix4d;
import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import ros.pkg.geometry_msgs.msg.Pose;
import ros.pkg.suturo_perception_msgs.msg.PerceivedObject;
import tfjava.Stamped;
import tfjava.TFListener;
import de.suturo.knowledge.foodreasoner.exception.TFException;

/**
 * Abstract representation of an object
 * 
 * @author Moritz
 * 
 */
public abstract class AbstractObject {
	private final Map<String, Stamped<Point3d>> transformedCentroids = new HashMap<String, Stamped<Point3d>>();
	private final Map<String, Stamped<Matrix4d>> transformedMatrix = new HashMap<String, Stamped<Matrix4d>>();
	private final Map<String, Stamped<Pose>> transformedPose = new HashMap<String, Stamped<Pose>>();
	private final Stamped<Point3d> centroid;
	private final Stamped<Matrix4d> pose;
	private final Vector3d cuboidDim;
	private final static TFListener tf = TFListenerSafe.getInstance();

	/**
	 * Construct an object from a PerceivedObject
	 * 
	 * @param po
	 *            The perceived object
	 */
	public AbstractObject(PerceivedObject po) {
		centroid = TransformUtils.getStamped3DPoint(po);
		pose = TransformUtils.getStamped4DPose(po);
		cuboidDim = new Vector3d(po.matched_cuboid.length1,
				po.matched_cuboid.length2, po.matched_cuboid.length3);
	}

	/**
	 * Copy constructor
	 * 
	 * @param po
	 *            The perceived object
	 */
	protected AbstractObject(AbstractObject object) {
		transformedCentroids.putAll(object.transformedCentroids);
		transformedMatrix.putAll(object.transformedMatrix);
		transformedPose.putAll(object.transformedPose);
		centroid = object.centroid;
		pose = object.pose;
		cuboidDim = object.cuboidDim;
	}

	/**
	 * @return the transformedCentroids
	 */
	public Stamped<Point3d> getTransformedCentroid(String frame) {
		return transformedCentroids.get(frame);
	}

	/**
	 * @return the transformedCuboid
	 */
	public Stamped<Matrix4d> getTransformedMatrix(String frame) {
		return transformedMatrix.get(frame);
	}

	/**
	 * @return the transformedPose
	 */
	public Stamped<Pose> getTransformedPose(String frame) {
		return transformedPose.get(frame);
	}

	/**
	 * @return the cuboidDim
	 */
	public Vector3d getCuboidDim() {
		return cuboidDim;
	}

	/**
	 * Execute and save TF transformation to given target frame
	 * 
	 * @param frame
	 *            Target frame
	 * @throws TFException
	 */
	public void transformToFrame(String frame) throws TFException {
		addTransformPoint(frame);
		addTransformPose(frame);
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
	private void addTransformPoint(String target) throws TFException {
		try {
			if (tf.lookupTransform(target, centroid.frameID, centroid.timeStamp) == null) {
				throw new TFException(target);
			}
			Stamped<Point3d> out = new Stamped<Point3d>();
			out.setData(new Point3d());
			tf.transformPoint(target, centroid, out);
			transformedCentroids.put(target, out);
		} catch (RuntimeException e) {
			throw e;
		} catch (Exception e) {
			throw new TFException(centroid.frameID, target, e);
		}
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
	 * @throws TFException
	 */
	private void addTransformPose(String target) throws TFException {
		try {
			if (tf.lookupTransform(target, pose.frameID, pose.timeStamp) == null) {
				throw new TFException(target);
			}
			Stamped<Matrix4d> out = new Stamped<Matrix4d>();
			out.setData(new Matrix4d());
			tf.transformPose(target, pose, out);
			transformedMatrix.put(target, out);
			Stamped<Pose> sPose = new Stamped<Pose>(
					TransformUtils.matrix4dToPose(out.getData()), out.frameID,
					out.timeStamp);
			transformedPose.put(target, sPose);
		} catch (RuntimeException e) {
			throw e;
		} catch (Exception e) {
			throw new TFException(centroid.frameID, target, e);
		}
	}

	/**
	 * Return identifier of object. This is shared with manipulation and
	 * planning.
	 */
	public abstract String getIdentifier();
}
