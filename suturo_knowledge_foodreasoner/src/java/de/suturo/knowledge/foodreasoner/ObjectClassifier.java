package de.suturo.knowledge.foodreasoner;

import ros.pkg.geometry_msgs.msg.Point;
import ros.pkg.suturo_perception_msgs.msg.PerceivedObject;

interface ObjectClassifier {
  public String classifyPerceivedObject(PerceivedObject obj);
}