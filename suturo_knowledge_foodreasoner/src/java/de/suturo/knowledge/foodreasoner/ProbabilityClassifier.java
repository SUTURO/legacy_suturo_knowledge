package de.suturo.knowledge.foodreasoner;

import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import ros.pkg.geometry_msgs.msg.Point;
import ros.pkg.suturo_perception_msgs.msg.PerceivedObject;

class ProbabilityClassifier implements ObjectClassifier {

  private String resultPrefix = "http://www.suturo.de/ontology/hierarchy#";
  private HashMap<String, Double> probData;
  private ArrayList<ProbabilityObject> probRes;
  private ArrayList<String> instanceNames;

  public ProbabilityClassifier() {
    ProbabilityData probDataCls = new ProbabilityData();
    probData = probDataCls.getData();
    probRes = new ArrayList<ProbabilityObject>();
    instanceNames = new ArrayList<String>();
    instanceNames.add("baguette");
    instanceNames.add("corny");
    instanceNames.add("dlink");
  }
  
  /**
   * Classifies an object given some values that were calculated by the 
   * perception
   * @param int average hue value of the object
   * @param int volume of the object
   * @return instance string that's used by prolog
   */
  public String classifyPerceivedObject(PerceivedObject percObject) {
    probRes = new ArrayList<ProbabilityObject>();

    for (String inst : instanceNames) {
      probRes.add(new ProbabilityObject(inst, 
          gauss(Math.sin(percObject.c_color_average_h * Math.PI / 180), probData.get(inst+"_hue_sin_mean"), probData.get(inst+"_hue_sin_sd")) +
          gauss(Math.sin(percObject.c_color_average_h * Math.PI / 180), probData.get(inst+"_hue_cos_mean"), probData.get(inst+"_hue_cos_sd")) 
          ));
    }

    Collections.sort(probRes);
    
    return resultPrefix + probRes.get(probRes.size()-1).instance;
  }
  
  /**
   * Calculates the probability density of a value given mean and standard
   * deviation
   * @param double value to calculate probability of
   * @param double mean
   * @param double standard deviation
   * @return the probability density
   */
  private static double probabilityDensity(double x, double mean, double sd) {
    return (1 / (sd * Math.sqrt(2 * Math.PI))) * 
            Math.exp( - ((x-mean)*(x-mean) / (2*sd*sd)) );
  }
  
  private static double gauss(double x, double mean, double sd) {
    return Math.exp(- ( (x - mean) * (x - mean) ) / ( 2 * sd * sd ) ) / (sd * Math.sqrt( 2 * Math.PI ));
  }
  
  /**
   * Returns a String filled with detailed information about the last
   * classification
   * @return String with details about the last classification
   */
  public String classificationInfo() {
    String ret = "Classification results:\n";
    for (ProbabilityObject po : probRes) {
      ret += po.toString() + "\n";
    }
    return ret;
  }
}
