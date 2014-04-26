package de.suturo.knowledge.foodreasoner;

import ros.pkg.suturo_perception_msgs.msg.PerceivedObject;

/**
 * Representation of a recognized object
 * 
 * @author Moritz
 * 
 */
public class RecognizedObject extends AbstractObject {

	private String identifier;

	/**
	 * Create a new instance from a perception
	 * 
	 * @param po
	 *            PerceivedObject
	 * @param identifier
	 *            Identifier
	 */
	public RecognizedObject(PerceivedObject po, String identifier) {
		super(po);
		this.identifier = identifier;
	}

	/**
	 * Create a new instance from a previously perceived object
	 * 
	 * @param object
	 *            AbstractObject instance
	 * @param identifier
	 *            Identifier
	 */
	public RecognizedObject(AbstractObject object, String identifier) {
		super(object);
		this.identifier = identifier;
	}

	@Override
	public String getIdentifier() {
		return identifier;
	}

	@Override
	public void setIdentifier(String identifier) {
		this.identifier = identifier;
	}

}
