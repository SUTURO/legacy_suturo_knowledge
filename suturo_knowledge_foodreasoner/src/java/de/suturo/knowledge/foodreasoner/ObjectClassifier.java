package de.suturo.knowledge.foodreasoner;

import ros.pkg.geometry_msgs.msg.Pose;
import ros.pkg.suturo_perception_msgs.msg.PerceivedObject;

interface ObjectClassifier {
  public String classifyPerceivedObject(PerceivedObject po);
}

