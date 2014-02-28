package de.suturo.knowledge.foodreasoner;

import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;

class ProbabilityClassifier implements ObjectClassifier {

  private HashMap<String, Double> probData;
  private ArrayList<ProbabilityObject> probRes;
  private ArrayList<String> instanceNames;

  public ProbabilityClassifier() {
    System.out.println("Hallo Welt!");
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
  public String classifyPerceivedObject(int avgHue, double volume) {
    probRes = new ArrayList<ProbabilityObject>();

    for (String inst : instanceNames) {
      probRes.add(new ProbabilityObject(inst, 
          normalDistribution(Math.sin(avgHue * Math.PI / 180), probData.get(inst+"_hue_sin_mean"), probData.get(inst+"_hue_sin_sd")) +
          normalDistribution(Math.sin(avgHue * Math.PI / 180), probData.get(inst+"_hue_cos_mean"), probData.get(inst+"_hue_cos_sd")) 
          ));
    }

    Collections.sort(probRes);
    
    return probRes.get(probRes.size()-1).instance;
  }
  
  /**
   * Calculates the probability density of a value given mean and standard
   * deviation
   * @param double value to calculate probability of
   * @param double mean
   * @param double standard deviation
   * @return the probability density
   */
  private static double normalDistribution(double x, double mean, double sd) {
    return (1 / (sd * Math.sqrt(2 * Math.PI))) * 
            Math.exp( - ((x-mean)*(x-mean) / (2*sd*sd)) );
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
