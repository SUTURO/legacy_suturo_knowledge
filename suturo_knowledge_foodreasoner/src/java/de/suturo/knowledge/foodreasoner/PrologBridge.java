package de.suturo.knowledge.foodreasoner;

import ros.NodeHandle;
import ros.Ros;
import ros.RosException;
import ros.pkg.suturo_perception_msgs.msg.PerceivedObject;

/**
 * Bridge from Perception to Prolog
 * 
 * @author Moritz Horstmann
 * 
 */
public class PrologBridge {

    private static final String NODE_NAME = "suturo_knowledge_javaclient";

    static Ros ros;
    static NodeHandle handle;
    static GetClustersService cluster;

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
    public static PerceivedObject[] updatePerception() throws RosException {
	checkInitialized();
	if (cluster == null) {
	    cluster = new GetClustersService(handle);
	}
	return cluster.getClusters().toArray(new PerceivedObject[0]);
    }

}