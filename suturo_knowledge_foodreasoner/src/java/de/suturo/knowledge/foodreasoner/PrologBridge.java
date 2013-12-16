package de.suturo.knowledge.foodreasoner;

import java.util.List;

import ros.NodeHandle;
import ros.Ros;
import ros.RosException;
import ros.ServiceClient;
import ros.pkg.suturo_perception_msgs.msg.PerceivedObject;
import ros.pkg.suturo_perception_msgs.srv.GetClusters;

/**
 * Bridge from Perception to Prolog
 * 
 * @author Moritz Horstmann
 * 
 */
public class PrologBridge {

    private static final String OBJECT_SERVICE = "/GetClusters";

    private static final String NODE_NAME = "suturo_knowledge_javaclient";

    static Ros ros;
    static NodeHandle handle;

    private static void checkInitialized() {
	ros = Ros.getInstance();
	if (!ros.isInitialized()) {
	    ros.init(NODE_NAME);
	}
	handle = ros.createNodeHandle();
	if (!handle.checkMaster()) {
	    throw new IllegalStateException("BLUREAGFBJORKIMAMALDIBALLOON!");
	}

    }

    /**
     * Calls perception service and retrieves list of perceived objects
     * 
     * @return true if any objects were found
     * @throws RosException
     */
    public static boolean updatePerception() throws RosException {
	checkInitialized();
	List<PerceivedObject> objects = null;
	// TODO: This needs refactoring!
	try {

	    GetClusters.Request req = new GetClusters.Request();
	    req.s = "get";

	    ServiceClient<GetClusters.Request, GetClusters.Response, GetClusters> cl = handle
		    .serviceClient(OBJECT_SERVICE, new GetClusters());
	    objects = cl.call(req).perceivedObjs;
	    cl.shutdown();

	} catch (RosException e) {
	    ros.logError("ROSClient: Call to service /GetClusters failed");
	    throw e;
	}
	return objects != null && objects.size() > 0;
    }

}