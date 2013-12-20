package de.suturo.knowledge.foodreasoner;

import java.util.List;

import ros.NodeHandle;
import ros.RosException;
import ros.ServiceClient;
import ros.pkg.suturo_perception_msgs.msg.PerceivedObject;
import ros.pkg.suturo_perception_msgs.srv.GetClusters;

/**
 * Manages access to GetClusters service of suturo_perception
 * 
 * @author Moritz
 * 
 */
public class GetClustersService {
    private NodeHandle handle;

    private final String serviceName;

    private static final String DEFAULT_SERVICE_NAME = "/GetClusters";

    enum Operation {
	GET("get");

	private String op;

	Operation(String op) {
	    this.op = op;
	}

	String getOp() {
	    return this.op;
	}
    }

    /**
     * Access to GetClusters using given handle
     * 
     * @param handle
     */
    public GetClustersService(NodeHandle handle) {
	this(handle, DEFAULT_SERVICE_NAME);
    }

    /**
     * Access to GetClusters using given handle and serviceName
     * 
     * @param instance
     * @param serviceName
     */
    public GetClustersService(NodeHandle instance, String serviceName) {
	this.serviceName = serviceName;
    }

    /**
     * Simply retrieves perceived objects from service
     * 
     * @return list of perceived objects
     * @throws RosException
     */
    public List<PerceivedObject> getClusters() throws RosException {
	return callService(Operation.GET);
    }

    /**
     * Calls service with given operation and returns a list of perceived
     * objects
     * 
     * @param op
     *            Operation
     * 
     * @return list of perceived objects
     * @throws RosException
     */
    private List<PerceivedObject> callService(Operation op) throws RosException {
	List<PerceivedObject> objects = null;
	GetClusters.Request req = new GetClusters.Request();
	req.s = op.getOp();
	ServiceClient<GetClusters.Request, GetClusters.Response, GetClusters> cl = handle
		.serviceClient(serviceName, new GetClusters());
	objects = cl.call(req).perceivedObjs;
	cl.shutdown();
	return objects;
    }
}
