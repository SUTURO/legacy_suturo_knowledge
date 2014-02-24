package de.suturo.knowledge.foodreasoner;


class ProbabilityObject implements Comparable<ProbabilityObject> {
  public String instance;
  public Double probability;

  public ProbabilityObject(String inst, Double prob) {
    instance = inst;
    probability = prob;
  }

  @Override
  public int compareTo(ProbabilityObject o) {
    return probability.compareTo(o.probability);
  } 

  @Override
  public String toString() {
    return instance + ": " + probability;
  }
}

