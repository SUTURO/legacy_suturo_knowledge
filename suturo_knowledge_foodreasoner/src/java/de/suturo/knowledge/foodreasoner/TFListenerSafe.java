package de.suturo.knowledge.foodreasoner;

import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStreamReader;

import tfjava.TFListener;

/**
 * This provides a TFListener which will work in rosjava environments used in
 * rosprolog environments that are launched with json_prolog
 * 
 * @author Moritz
 * 
 */
public class TFListenerSafe extends TFListener {

	/**
	 * Returns the TFListener instance.
	 */
	public synchronized static TFListener getInstance() {
		if (instance == null) {
			instance = new TFListenerSafe();
		}
		if (!(instance instanceof TFListenerSafe)) {
			ros.logError("Non ROS Spinner safe TFListener instance detected!");
		}
		return instance;
	}

	@Override
	protected void spinInSeperateThread() {
		try {
			Process rospack = Runtime.getRuntime().exec("rosservice list ");
			BufferedReader br = new BufferedReader(new InputStreamReader(
					rospack.getInputStream()));
			String line;
			while ((line = br.readLine()) != null) {
				if (line.startsWith("/json_prolog")) {
					return;
				}
			}
		} catch (IOException e) {
			rosNode.logError("Could not call rosservice. Check your environment!");
		}
		super.spinInSeperateThread();
	}
}
