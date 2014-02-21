package de.suturo.knowledge.foodreasoner;

import java.util.ArrayList;
import java.util.Collections;

class ProbabilityClassifier {

  private static ArrayList<PerceptionProbabilities> objects = new ArrayList<PerceptionProbabilities>();
  
  public ProbabilityClassifier() {
    
  }
  
  /**
   * Classifies an object given some values that were calculated by the 
   * perception
   * @param int average hue value of the object
   * @param int volume of the object
   * @return instance string that's used by prolog
   */
  public String classifyPerceivedObject(int avgHue, double volume) {
    if (objects == null)
      return "";
    if (objects.size() < 1)
      return "";
    
    for (int i = 0; i < objects.size(); i++) {
      objects.get(i).probability = 
          normalDistribution(avgHue, objects.get(i).hueMean, objects.get(i).hueSD) +
          normalDistribution(volume, objects.get(i).volMean, objects.get(i).volSD) ;
    }
    
    Collections.sort(objects);
    
    return objects.get(objects.size()-1).toString();
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
   * This will be called by prolog on startup to fill the list of objects that
   * we need to classify
   * @param String instance name in prolog
   * @param String hue mean value
   * @param String hue standard deviation
   * @param String volume mean
   * @param String volume standard deviation
   */
  public String appendToObjectsList(String instance, String hueMean, String hueSD, 
                                 String volMean, String volSD) {
    double hueMeanD;
    double hueSDD;
    double volMeanD;
    double volSDD;
    try {
      hueMeanD = Double.parseDouble(hueMean);
      hueSDD = Double.parseDouble(hueSD);
      volMeanD = Double.parseDouble(volMean);
      volSDD = Double.parseDouble(volSD);
    } catch (NumberFormatException ex) {
      return ex.toString();
    }
    PerceptionProbabilities entry = new PerceptionProbabilities(instance, 
                                                                hueMeanD, 
                                                                hueSDD, 
                                                                volMeanD, 
                                                                volSDD);
    objects.add(entry);
    return "number of objects for classification: " + objects.size();
  }
}