package de.suturo.knowledge.foodreasoner;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;
import java.lang.Thread.UncaughtExceptionHandler;

/**
 * Simple exception logger
 * 
 * @author Moritz
 * 
 */
public class ExceptionHandler {

	/**
	 * Set exception logger for current JVM instance
	 */
	public static void setExcaptionHandler() {
		Thread.setDefaultUncaughtExceptionHandler(new UncaughtExceptionHandler() {

			@Override
			public void uncaughtException(Thread t, Throwable e) {
				File logFold = new File("/tmp/javacrashlog");
				if (!logFold.exists()) {
					logFold.mkdir();
				}
				File logFile = new File(logFold, "errorlog"
						+ System.currentTimeMillis() + ".log");
				FileWriter fw = null;
				try {
					fw = new FileWriter(logFile);
					fw.write("Thread:" + t.getName());
					fw.write("\nClass:" + e.getClass().getSimpleName());
					fw.write("\nMessage:" + e.getMessage());
					fw.write("\nStacktrace:\n");
					e.printStackTrace(new PrintWriter(fw));
				} catch (IOException e1) {
					e1.printStackTrace();
				} finally {
					try {
						if (fw != null) {
							fw.close();
						}
					} catch (IOException e1) {
						// close quietly
					}
				}
			}
		});
	}
}
