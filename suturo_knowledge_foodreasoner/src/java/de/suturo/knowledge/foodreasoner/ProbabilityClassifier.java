package de.suturo.knowledge.foodreasoner;

import java.util.ArrayList;

class ProbabilityClassifier {

  private static ArrayList<ArrayList<String>> objects = new ArrayList<ArrayList<String>>();
  
  public ProbabilityClassifier() {
    
  }
  
  public String classifyPerceivedObject(int avgHue, double volume) {
    String ret = "";
    for (ArrayList<String> entry : objects) {
      for (String x : entry) {
        ret += x + ", ";
      }
      ret += "; ";
    }
    return ret;
  }
  
  public int appendToObjectsList(String instance, String hueMean, String hueSD, String volMean, String volSD) {
    ArrayList<String> entry = new ArrayList<String>();
    entry.add(instance);
    entry.add(hueMean);
    entry.add(hueSD);
    entry.add(volMean);
    entry.add(volSD);
    objects.add(entry);
    return objects.size();
  }
}