package de.suturo.knowledge.foodreasoner.exception;

/**
 * This exception is thrown if a TF conversion fails.
 * 
 * @author Moritz
 * 
 */
public class TFException extends Exception {

    private static final long serialVersionUID = -866117225046165234L;

    /**
     * Default constructor
     */
    public TFException() {
	super("An error occured while executing a TF transformation");
    }

    /**
     * Default constructor with cause
     * 
     * @param cause
     */
    public TFException(Throwable cause) {
	super("An error occured while executing a TF transformation", cause);
    }

    /**
     * For TF lookup failures
     * 
     * @param lookupFrame
     *            Frame failed to get looked up
     */
    public TFException(String lookupFrame) {
	super("Could not find " + lookupFrame + " in the TF cache!");
    }

    /**
     * For TF lookup failures with cause
     * 
     * @param lookupFrame
     *            Frame failed to get looked up
     * @param cause
     */
    public TFException(String lookupFrame, Throwable cause) {
	super("Could not find " + lookupFrame + " in the TF cache!", cause);
    }

    /**
     * For TF transform failures
     * 
     * @param sourceFrame
     *            Source frame
     * @param targetFrame
     *            Target frame
     */
    public TFException(String sourceFrame, String targetFrame) {
	super("Could not transform from " + sourceFrame + " to " + targetFrame);
    }

    /**
     * For TF transform failures with cause
     * 
     * @param sourceFrame
     *            Source frame
     * @param targetFrame
     *            Target frame
     * @param cause
     */
    public TFException(String sourceFrame, String targetFrame, Throwable cause) {
	super("Could not transform from " + sourceFrame + " to " + targetFrame, cause);
    }
}
