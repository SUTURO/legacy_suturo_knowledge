package de.suturo.knowledge.foodreasoner.classifier;

import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStreamReader;
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
import de.suturo.java.serviceclient.GetClustersService;
import de.suturo.knowledge.foodreasoner.TFListenerSafe;

/**
 * Bridge from Perception to Prolog
 * 
 * @author Moritz Horstmann
 * 
 */
public class ClassificationTest {

	private static final String NODE_NAME = "suturo_knowledge_perception_testclient";
	private static final int DEFAULT_TIMEOUT_MS = 10000;

	private static Ros ros;
	private static NodeHandle handle;
	private static GetClustersService cluster;
	private static TFListener tf;

	private final Map<Long, Stamped<Point3d>> mapCoords = new HashMap<Long, Stamped<Point3d>>();
	private final Map<Long, Stamped<Point3d>> odomCoords = new HashMap<Long, Stamped<Point3d>>();

	public static void main(String[] argv) throws Exception {
		ClassificationTest ct = new ClassificationTest();
		ct.doStuff();
	}

	/**
	 * Initializes node
	 */
	public ClassificationTest() {
		checkInitialized();
		tf = TFListenerSafe.getInstance();
		handle.logInfo("ClassificationTest initialized");
	}

	public void doStuff() throws Exception {
		ProbabilityClassifier pc = new ProbabilityClassifier();
		WekaClassifier wc = new WekaClassifier();
		BufferedReader br = new BufferedReader(new InputStreamReader(System.in));
		while (true) {
			System.out.print("> ");
			try {
				String in = br.readLine();
				if (in == null)
					continue;
				if ("prob".equals(in)) {
					printClassification(pc);
					System.out.println(pc.classificationInfo());
					continue;
				}
				if ("weka".equals(in)) {
					printClassification(wc);
					continue;
				}
				if ("exit".equals(in)) {
					System.exit(0);
					break;
				}
				if ("quit".equals(in)) {
					System.exit(0);
					break;
				}
				System.out.println("Commands:\n\n  prob\n  weka\n");
			} catch (IOException e) {
				e.printStackTrace();
			} catch (RosException ex) {
				ex.printStackTrace();
			}
		}
	}

	private void printClassification(ObjectClassifier cl) throws RosException {
		System.out.println("calling perception... ");
		PerceivedObject[] objs = updatePerception();
		System.out.println("done");
		for (PerceivedObject obj : objs) {
			String res = cl.classifyPerceivedObject(obj);
			System.out.println("classification result: " + res);
		}
	}

	private static void checkInitialized() {
		ros = Ros.getInstance();
		if (!ros.isInitialized()) {
			ros.init(NODE_NAME);
		}
		handle = ros.createNodeHandle();
		handle.setMasterRetryTimeout(DEFAULT_TIMEOUT_MS);
		if (!handle.checkMaster()) {
			ros.logError("ClassificationTest: Ros master not available");
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
			// FINDME TODO switch back to odom when it doesn't fuck up over time
			// addTransformPoint("/odom_combined", poPoint, odomCoords,
			// po.c_id);
			addTransformPoint("/base_link", poPoint, odomCoords, po.c_id);
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
		try {
			Stamped<Point3d> out = new Stamped<Point3d>();
			out.setData(new Point3d());
			if (tf.lookupTransform(target, in.frameID, in.timeStamp) == null) {
				ros.logWarn("Frame with ID " + target + " not found!");
				return;
			}
			tf.transformPoint(target, in, out);
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

	public double classifyObjectColorHue(int avg_hue) {
		return 0.5;
	}
}
