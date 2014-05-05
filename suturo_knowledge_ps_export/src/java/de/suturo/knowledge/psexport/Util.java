package de.suturo.knowledge.psexport;

import ros.pkg.geometry_msgs.msg.Quaternion;

/**
 * Provide util function for some abstraction helper methods to assist in using (auto generated) rosjava stuff.
 * 
 * @author Moritz
 * 
 */
public final class Util {

    private Util() {
	// Utility class
    }

    /**
     * Java implementation of C++ length2 from tf::Quaternion
     * 
     * @param q
     *            Quaternion message
     * @return length2 value
     */
    public static double quatLength2(Quaternion q) {
	return q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
    }

    /**
     * Java implementation of C++ setValue from tf::Quaternion
     * 
     * @param q
     *            Quaternion message
     * @param x
     * @param y
     * @param z
     * @param w
     */
    public static void quatSetValue(Quaternion q, double x, double y, double z, double w) {
	q.x = x;
	q.y = y;
	q.z = z;
	q.w = w;
    }

    /**
     * Create Quaternion message from roll, pitch, yaw.
     * 
     * @param roll
     * @param pitch
     * @param yaw
     * @return Quaternion message
     */
    public static Quaternion createMsgForQuaternion(double roll, double pitch, double yaw) {
	double QUATERNION_TOLERANCE = 0.1f;
	double halfYaw = yaw * 0.5;
	double halfPitch = pitch * 0.5;
	double halfRoll = roll * 0.5;
	double cosYaw = Math.cos(halfYaw);
	double sinYaw = Math.sin(halfYaw);
	double cosPitch = Math.cos(halfPitch);
	double sinPitch = Math.sin(halfPitch);
	double cosRoll = Math.cos(halfRoll);
	double sinRoll = Math.sin(halfRoll);
	Quaternion q = new Quaternion();
	q.x = sinRoll * cosPitch * cosYaw - cosRoll * sinPitch * sinYaw;
	q.y = cosRoll * sinPitch * cosYaw + sinRoll * cosPitch * sinYaw;
	q.z = cosRoll * cosPitch * sinYaw - sinRoll * sinPitch * cosYaw;
	q.w = cosRoll * cosPitch * cosYaw + sinRoll * sinPitch * sinYaw;
	if (Math.abs(quatLength2(q) - 1) > QUATERNION_TOLERANCE) {
	    throw new UnsupportedOperationException("Non normalized quaternion found. Normalize not yet implemented");
	}
	return q;
    }
}
