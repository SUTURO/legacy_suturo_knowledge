package de.suturo.java.serviceclient;

import java.util.List;

import ros.NodeHandle;
import ros.RosException;
import ros.ServiceClient;
import ros.pkg.suturo_perception_msgs.msg.Barcode;
import ros.pkg.suturo_perception_msgs.srv.GetBarcode;
import ros.pkg.suturo_perception_msgs.srv.GetBarcode.Request;
import ros.pkg.suturo_perception_msgs.srv.GetBarcode.Response;

/**
 * Manages access to GetBarcode service of suturo_perception
 * 
 * @author Moritz
 * 
 */
public class GetBarcodeService {
	private final NodeHandle handle;

	private final String serviceName;

	private static final String DEFAULT_SERVICE_NAME = "/suturo/GetBarcode";

	/**
	 * Enum to determine the operation
	 * 
	 * @author Moritz
	 * 
	 */
	enum Operation {
		/** Get recognized barcodes */
		GET();
	}

	/**
	 * Access to GetBarcode using given handle
	 * 
	 * @param handle
	 */
	public GetBarcodeService(NodeHandle handle) {
		this(handle, DEFAULT_SERVICE_NAME);
	}

	/**
	 * Access to GetBarcode using given handle and serviceName
	 * 
	 * @param handle
	 * @param serviceName
	 */
	public GetBarcodeService(NodeHandle handle, String serviceName) {
		this.handle = handle;
		this.serviceName = serviceName;
	}

	/**
	 * Simply retrieves barcodes from service
	 * 
	 * @return list of barcodes
	 * @throws RosException
	 */
	public List<Barcode> getBarcodes() throws RosException {
		return callService(Operation.GET);
	}

	/**
	 * Calls service with given operation and returns a list of barcodes
	 * 
	 * @param op
	 *            Operation
	 * 
	 * @return list of barcodes
	 * @throws RosException
	 */
	private List<Barcode> callService(Operation op) throws RosException {
		ServiceClient<Request, Response, GetBarcode> cl = handle.serviceClient(
				serviceName, new GetBarcode());
		List<Barcode> barcodes = cl.call(new Request()).barcodes;
		cl.shutdown();
		return barcodes;
	}
}
