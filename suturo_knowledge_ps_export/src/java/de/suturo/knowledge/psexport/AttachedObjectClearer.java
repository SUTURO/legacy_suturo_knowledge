package de.suturo.knowledge.psexport;

import java.util.ArrayList;

import ros.NodeHandle;
import ros.Publisher;
import ros.RosException;
import ros.ServiceClient;
import ros.pkg.moveit_msgs.msg.AttachedCollisionObject;
import ros.pkg.moveit_msgs.msg.CollisionObject;
import ros.pkg.moveit_msgs.msg.PlanningScene;
import ros.pkg.moveit_msgs.srv.GetPlanningScene;

/**
 * This can clear attached collision states from the Planning Scene
 * 
 * @author Moritz
 * 
 */
public class AttachedObjectClearer {

	private final NodeHandle handle;
	private Publisher<AttachedCollisionObject> publisher;

	/**
	 * Initialize publisher
	 * 
	 * @param handle
	 *            ROS node handle of MapConverter node
	 */
	public AttachedObjectClearer(NodeHandle handle) {
		this.handle = handle;
		try {
			publisher = handle.advertise("/attached_collision_object",
					new AttachedCollisionObject(), 100);
		} catch (RosException e) {
			handle.logError("Publisher advertisement failed: " + e.getMessage());
		}
	}

	/**
	 * Dirty method to remove attached states from Planning Scene
	 * 
	 * @param object
	 *            Object name
	 * @throws RosException
	 */
	public void clearObject(String object) throws RosException {
		GetPlanningScene.Request req = new GetPlanningScene.Request();
		ServiceClient<GetPlanningScene.Request, GetPlanningScene.Response, GetPlanningScene> cl = handle
				.serviceClient("/get_planning_scene", new GetPlanningScene());
		PlanningScene ps = cl.call(req).scene;
		cl.shutdown();
		ArrayList<AttachedCollisionObject> acos = ps.robot_state.attached_collision_objects;
		AttachedCollisionObject o = null;
		for (AttachedCollisionObject aco : acos) {
			if (object.equals(aco.object.id)) {
				o = aco;
				break;
			}
		}
		if (o == null) {
			return;
		}
		AttachedCollisionObject rem = new AttachedCollisionObject();
		rem.link_name = o.link_name;
		rem.object.id = object;
		rem.object.operation = CollisionObject.REMOVE;
		publisher.publish(rem);
	}
}
