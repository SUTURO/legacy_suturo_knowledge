package de.suturo.knowledge.psexport;

import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStreamReader;

import ros.communication.Time;
import de.suturo.knowledge.psexport.CollisionObjectWrapper.Operation;

/**
 * Spawns a simple console which takes object attributes and spawns them in the PlanningScene (currently only boxes). If
 * an object name was used before, it will be removed first.
 * 
 * @author Moritz
 * 
 */
public class PSConsole {

    /**
     * Main entrance
     * 
     * @param args
     * @throws IOException
     */
    public static void main(String... args) throws IOException {
	MapConverter conv = new MapConverter();
	BufferedReader br = new BufferedReader(new InputStreamReader(System.in));
	while (true) {
	    System.out.println("New box coords (Name, FrameID, dimX, dimY, dimZ, posX, posY, posZ) ");
	    String in = br.readLine();
	    String[] split = in.split(",");
	    if (split[0] != null && split[0].trim().equals("exit")) {
		break;
	    }
	    if (split[0] != null && split[0].trim().equals("publish")) {
		conv.publishScene();
		break;
	    }
	    conv.addCollisionObject(generateFromInput(split));
	}
	conv.shutdown();
    }

    private static CollisionObjectWrapper generateFromInput(String[] in) {
	CollisionObjectWrapper obj = new CollisionObjectWrapper(getArr(in, 0, "table"), Operation.ADD);
	obj.setFrame(getArr(in, 1, "/base_footprint"), Time.now());
	double dimX = Double.parseDouble(getArr(in, 2, "0"));
	double dimY = Double.parseDouble(getArr(in, 3, "0"));
	double dimZ = Double.parseDouble(getArr(in, 4, "0"));
	obj.addPrimitiveBox(dimX, dimY, dimZ);
	double posX = Double.parseDouble(getArr(in, 5, "0"));
	double posY = Double.parseDouble(getArr(in, 6, "0"));
	double posZ = Double.parseDouble(getArr(in, 7, "0"));
	obj.addPose(posX, posY, posZ, 0, 0, 0);
	return obj;
    }

    private static String getArr(String[] in, int pos, String def) {
	if (in.length >= pos + 1 && in[pos] != null && !in[pos].isEmpty()) {
	    return in[pos];
	}
	return def;
    }

}
