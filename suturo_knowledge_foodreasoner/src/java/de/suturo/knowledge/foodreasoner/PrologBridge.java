package de.suturo.knowledge.foodreasoner;

import java.util.HashMap;
import java.util.Map;

import javax.vecmath.Point3d;

import ros.NodeHandle;
import ros.Ros;
import ros.RosException;
import ros.communication.Time;
import ros.pkg.geometry_msgs.msg.Point;
import ros.pkg.suturo_perception_msgs.msg.PerceivedObject;
import tfjava.Stamped;
import tfjava.TFListener;

/**
 * Bridge from Perception to Prolog
 * 
 * @author Moritz Horstmann
 * 
 */
public class PrologBridge {

    private static final String NODE_NAME = "suturo_knowledge_javaclient";
    private static TFListener tf = TFListener.getInstance();

    private static Ros ros;
    private static NodeHandle handle;
    private static GetClustersService cluster;

    private final Map<Long, Stamped<Point3d>> mapCoords = new HashMap<Long, Stamped<Point3d>>();
    private final Map<Long, Stamped<Point3d>> odomCoords = new HashMap<Long, Stamped<Point3d>>();

    private static void checkInitialized() {
	ros = Ros.getInstance();
	if (!ros.isInitialized()) {
	    ros.init(NODE_NAME);
	}
	handle = ros.createNodeHandle();
	if (!handle.checkMaster()) {
	    ros.logError("PrologBridge: Ros master not available");
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
	checkInitialized();
	if (cluster == null) {
	    cluster = new GetClustersService(handle);
	}
	mapCoords.clear();
	odomCoords.clear();
	PerceivedObject[] pos = cluster.getClusters().toArray(
		new PerceivedObject[0]);
	for (PerceivedObject po : pos) {
	    Stamped<Point3d> poPoint = getStamped3DPoint(po);
	    addTransformPoint("/map", poPoint, mapCoords, po.c_id);
	    addTransformPoint("/odom_combined", poPoint, odomCoords, po.c_id);
	}
	return pos;
    }

    private static Stamped<Point3d> getStamped3DPoint(PerceivedObject po) {
	Point cent = po.c_centroid;
	Point3d point3d = new Point3d(cent.x, cent.y, cent.z);
	return new Stamped<Point3d>(point3d, po.frame_id, Time.now());
    }

    private static void addTransformPoint(String target, Stamped<Point3d> in,
	    Map<Long, Stamped<Point3d>> map, long cID) {
	if (!handle.getAdvertisedTopics().contains(target)) {
	    ros.logError("Topic " + target + " not advertised!");
	    return;
	}
	try {
	    Stamped<Point3d> out = new Stamped<Point3d>();
	    tf.transformPoint(target, in, out);
	    map.put(Long.valueOf(cID), out);
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
}