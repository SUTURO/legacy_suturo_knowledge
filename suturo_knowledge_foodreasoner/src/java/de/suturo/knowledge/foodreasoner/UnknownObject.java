package de.suturo.knowledge.foodreasoner;

import java.util.UUID;

import ros.pkg.suturo_perception_msgs.msg.PerceivedObject;

/**
 * Representation of unknown object
 * 
 * @author moritz
 * 
 */
public class UnknownObject extends AbstractObject {

    private String temporaryIdentifier = "http://www.suturo.de/ontology/unclassified#" + UUID.randomUUID().toString();

    /**
     * Default constructor
     * 
     * @param po
     *            Perceived object
     */
    public UnknownObject(PerceivedObject po) {
	super(po);
    }

    @Override
    public String getIdentifier() {
	return temporaryIdentifier;
    }

    @Override
    public void setIdentifier(String identifier) {
	temporaryIdentifier = identifier;
    }

}
