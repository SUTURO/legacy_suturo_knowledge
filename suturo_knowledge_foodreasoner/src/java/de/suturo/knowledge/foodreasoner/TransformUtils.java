package de.suturo.knowledge.foodreasoner;

import javax.vecmath.Matrix4d;
import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;

import ros.communication.Time;
import ros.pkg.geometry_msgs.msg.Point;
import ros.pkg.geometry_msgs.msg.Pose;
import ros.pkg.suturo_perception_msgs.msg.PerceivedObject;
import tfjava.Stamped;

/**
 * Holds transformation utils for easy conversion between ROS message types,
 * javax.vecmath objects and Prolog in/output.
 * 
 * @author Moritz
 * 
 */
public class TransformUtils {
	private TransformUtils() {
		// utility class
	}

	/**
	 * Utility method to convert a Pose to Matrix4d object
	 * 
	 * @param pose
	 *            Pose object
	 * @return Matrix4d object
	 */
	public static Matrix4d poseToMatrix4d(Pose pose) {
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
	public static Pose matrix4dToPose(Matrix4d matrix) {
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
	 * Utility method to convert a PerceivedObject centroid to
	 * Stamped&lt;Point3d&gt; object
	 * 
	 * @param po
	 *            PerceivedObject object
	 * @return Stamped&lt;Point3d&gt; object
	 */
	public static Stamped<Point3d> getStamped3DPoint(PerceivedObject po) {
		Point cent = po.c_centroid;
		Point3d point3d = new Point3d(cent.x, cent.y, cent.z);
		return new Stamped<Point3d>(point3d, po.frame_id, Time.now());
	}

	/**
	 * Utility method to convert a PerceivedObject cuboid pose to
	 * Stamped&lt;Matrix4d&gt; object
	 * 
	 * @param po
	 *            PerceivedObject object
	 * @return Stamped&lt;Point3d&gt; object
	 */
	public static Stamped<Matrix4d> getStamped4DPose(PerceivedObject po) {
		Matrix4d mat = TransformUtils.poseToMatrix4d(po.matched_cuboid.pose);
		return new Stamped<Matrix4d>(mat, po.frame_id, Time.now());
	}
}
