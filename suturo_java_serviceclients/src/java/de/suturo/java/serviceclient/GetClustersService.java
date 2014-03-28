package de.suturo.java.serviceclient;

import java.util.List;

import ros.NodeHandle;
import ros.RosException;
import ros.ServiceClient;
import ros.pkg.suturo_perception_msgs.msg.PerceivedObject;
import ros.pkg.suturo_perception_msgs.srv.GetClusters;
import ros.pkg.suturo_perception_msgs.srv.GetClusters.Request;
import ros.pkg.suturo_perception_msgs.srv.GetClusters.Response;

/**
 * Manages access to GetClusters service of suturo_perception
 * 
 * @author Moritz
 * 
 */
public class GetClustersService {
	private final NodeHandle handle;

	private final String serviceName;

	private static final String DEFAULT_SERVICE_NAME = "/suturo/GetClusters";

	/**
	 * Enum to determine the operation
	 * 
	 * @author Moritz
	 * 
	 */
	enum Operation {
		/** Get recognized objects */
		GET("get");

		private String op;

		Operation(String op) {
			this.op = op;
		}

		/** Return operation specific short representing the operation */
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
	 * @param handle
	 * @param serviceName
	 */
	public GetClustersService(NodeHandle handle, String serviceName) {
		this.handle = handle;
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
		Request req = new Request();
		req.s = op.getOp();
		ServiceClient<Request, Response, GetClusters> cl = handle
				.serviceClient(serviceName, new GetClusters());
		List<PerceivedObject> objects = cl.call(req).perceivedObjs;
		cl.shutdown();
		return objects;
	}
}
