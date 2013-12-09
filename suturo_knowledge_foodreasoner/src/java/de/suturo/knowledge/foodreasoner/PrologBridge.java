package de.suturo.knowledge.foodreasoner;

/**
 * Bridge from Perception to Prolog
 * 
 * @author Moritz Horstmann
 * 
 */
public class PrologBridge {
    private final String name;

    /**
     * @param name
     */
    public PrologBridge(String name) {
	this.name = name;
    }

    /**
     * @return the name
     */
    public String getName() {
	return this.name;
    }
}