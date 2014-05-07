package de.suturo.knowledge.foodreasoner.classifier;

import ros.pkg.suturo_perception_msgs.msg.PerceivedObject;

public interface ObjectClassifier {
    public String classifyPerceivedObject(PerceivedObject po);
}
