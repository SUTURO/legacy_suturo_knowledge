package de.suturo.knowledge.foodreasoner.classifier;

import java.io.BufferedReader;
import java.io.FileNotFoundException;
import java.io.IOException;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.util.ArrayList;
import java.util.Enumeration;
import java.util.List;

import ros.pkg.suturo_perception_msgs.msg.PerceivedObject;
import weka.classifiers.Classifier;
import weka.core.Attribute;
import weka.core.Instance;
import weka.core.Instances;
import weka.core.SerializationHelper;

/**
 * Interface to Weka classifier
 * 
 * @author Tom - Initial classifier
 * @author Moritz - Completely rewritten
 * 
 */
public class WekaClassifier implements ObjectClassifier {
	private static final String ARFF_FILE = "/arff/milestone4.arff";
	private static final String CLASSIFIER_PATH = "/classifier.model";
	private static final String OWL_NS = "http://www.suturo.de/ontology/suturo#";
	private final Classifier classifier;
	private final List<Attribute> attributes;
	private final Instances dataSet;

	/**
	 * Initialize classifier
	 * 
	 * @throws Exception
	 */
	@SuppressWarnings("unchecked")
	public WekaClassifier() throws Exception {
		@SuppressWarnings("resource")
		InputStream arff = WekaClassifier.class.getResourceAsStream(ARFF_FILE);
		if (arff == null) {
			throw new FileNotFoundException("Could not get arff file "
					+ ARFF_FILE + " from classpath!");
		}
		Instances data = getInstances(arff);
		attributes = new ArrayList<Attribute>(data.numAttributes());
		Enumeration<Attribute> atts = data.enumerateAttributes();
		while (atts.hasMoreElements()) {
			attributes.add(atts.nextElement());
		}
		data.setClassIndex(data.numAttributes() - 1);
		this.dataSet = data;
		classifier = (Classifier) SerializationHelper.read(WekaClassifier.class
				.getResourceAsStream(CLASSIFIER_PATH));

	}

	private static Instances getInstances(InputStream is) throws IOException {
		BufferedReader reader = null;
		try {
			InputStreamReader isr = new InputStreamReader(is);
			reader = new BufferedReader(isr);
			return new Instances(reader);
		} finally {
			if (reader != null) {
				try {
					reader.close();
				} catch (IOException e) {
					// Close quietly
				}
			}
		}
	}

	@Override
	public String classifyPerceivedObject(PerceivedObject po) {
		Instance inst = new Instance(attributes.size());
		inst.setDataset(dataSet);
		for (Attribute att : attributes) {
			setPOValue(po, inst, att);
		}
		inst.setClassMissing();
		try {
			// double index = classifier.classifyInstance(inst);
			double[] dist = classifier.distributionForInstance(inst);
			for (int i = 0; i < dist.length; i++) {
				System.out.println("Class " + inst.classAttribute().value(i)
						+ " has probability " + dist[i]);
			}
			int index = getMaxDistIndex(dist);
			return OWL_NS + inst.classAttribute().value(index);
		} catch (Exception e) {
			e.printStackTrace();
		}
		return null;
	}

	private static int getMaxDistIndex(double[] probs) {
		int index = 0;
		for (int i = 0; i < probs.length; i++) {
			if (probs[i] > probs[index]) {
				index = i;
			}
		}
		return index;
	}

	private static void setPOValue(PerceivedObject po, Instance inst,
			Attribute att) {
		double val = po.c_color_average_h * Math.PI / 180;
		double sin = Math.sin(val);
		double cos = Math.cos(val);
		double maxLen = Math.max(po.matched_cuboid.length1,
				Math.max(po.matched_cuboid.length2, po.matched_cuboid.length3));
		double middleLen = Math.min(
				Math.max(po.matched_cuboid.length1, po.matched_cuboid.length2),
				Math.max(po.matched_cuboid.length2, po.matched_cuboid.length3));
		double minLen = Math.min(po.matched_cuboid.length1,
				Math.min(po.matched_cuboid.length2, po.matched_cuboid.length3));
		String name = att.name();
		if ("red".equals(name))
			inst.setValue(att, po.c_color_average_r);
		else if ("green".equals(name))
			inst.setValue(att, po.c_color_average_g);
		else if ("blue".equals(name))
			inst.setValue(att, po.c_color_average_b);
		else if ("hue_sin".equals(name))
			inst.setValue(att, sin);
		else if ("hue_cos".equals(name))
			inst.setValue(att, cos);
		else if ("saturation".equals(name))
			inst.setValue(att, po.c_color_average_s);
		else if ("value".equals(name))
			inst.setValue(att, po.c_color_average_v);
		else if ("vol".equals(name))
			inst.setValue(att, po.matched_cuboid.volume);
		else if ("length_1".equals(name))
			inst.setValue(att, maxLen);
		else if ("length_2".equals(name))
			inst.setValue(att, middleLen);
		else if ("length_3".equals(name))
			inst.setValue(att, minLen);
		else if ("cuboid_length_relation_1".equals(name))
			inst.setValue(att, maxLen / middleLen);
		else if ("cuboid_length_relation_2".equals(name))
			inst.setValue(att, maxLen / minLen);
		else if ("label_2d".equals(name)) {
			if (po.recognition_label_2d.isEmpty())
				inst.setMissing(att);
			else
				inst.setValue(att, po.recognition_label_2d);
		} else if ("shape".equals(name))
			inst.setValue(att, po.c_shape);
	}
}
