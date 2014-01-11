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

    private final Map<Long, Point3d> mapCoords = new HashMap<Long, Point3d>();
    private final Map<Long, Point3d> odomCoords = new HashMap<Long, Point3d>();

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

	    Stamped<Point3d> poMap = new Stamped<Point3d>();
	    Stamped<Point3d> poOdom = new Stamped<Point3d>();
	    Stamped<Point3d> poPoint = getStamped3DPoint(po);
	    tf.transformPoint("/map", poPoint, poMap);
	    tf.transformPoint("/odom_combined", poPoint, poOdom);
	    mapCoords.put(Long.valueOf(po.c_id), poMap.getData());
	    odomCoords.put(Long.valueOf(po.c_id), poOdom.getData());
	}
	return pos;
    }

    private static Stamped<Point3d> getStamped3DPoint(PerceivedObject po) {
	Point cent = po.c_centroid;
	Point3d point3d = new Point3d(cent.x, cent.y, cent.z);
	return new Stamped<Point3d>(point3d, po.frame_id, Time.now());
    }

    /**
     * Retrieve transformed coordinates in map frame context.
     * 
     * @param c_id
     *            ID of corresponding PerceivedObject
     * @return Point3d object
     */
    public Point3d getMapCoords(long c_id) {
	return this.mapCoords.get(Long.valueOf(c_id));
    }

    /**
     * Retrieve transformed coordinates in odom_combined frame context.
     * 
     * @param c_id
     *            ID of corresponding PerceivedObject
     * @return Point3d object
     */
    public Point3d getOdomCoords(long c_id) {
	return this.odomCoords.get(Long.valueOf(c_id));
    }
}