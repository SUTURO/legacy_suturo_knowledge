package de.suturo.knowledge.foodreasoner;

/**
 * Container class for the probabilities of some attributes of perceived objects
 */
class PerceptionProbabilities implements Comparable<PerceptionProbabilities> {
  public final String instance; /// this is the string that prolog uses to uniquely identify an object
  public final double hueMean;
  public final double hueSD;
  public final double volMean;
  public final double volSD;
  public Double probability; // the resulting probability
  
  public PerceptionProbabilities(String instance, double hueMean, double hueSD,
                                 double volMean, double volSD) {
    this.instance = instance;
    this.hueMean = hueMean;
    this.hueSD = hueSD;
    this.volMean = volMean;
    this.volSD = volSD;
  }
  
  @Override
  public int compareTo(PerceptionProbabilities  o) {
    return probability.compareTo(o.probability);
  }
  
  @Override
  public String toString() {
    return instance + ": " + probability;
  }
}