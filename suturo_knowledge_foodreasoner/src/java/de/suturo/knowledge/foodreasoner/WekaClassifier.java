package de.suturo.knowledge.foodreasoner;

import ros.pkg.geometry_msgs.msg.Point;
import ros.pkg.geometry_msgs.msg.Pose;
import ros.pkg.suturo_perception_msgs.msg.PerceivedObject;

import weka.core.Instances;
import weka.classifiers.bayes.NaiveBayes;
import java.io.BufferedReader;
import java.io.FileReader;
import java.io.InputStreamReader;
import java.io.IOException;

class WekaClassifier implements ObjectClassifier {
    private NaiveBayes classifier = new NaiveBayes();
  public WekaClassifier() {
    String foodreasoner_path = executeCommand("rospack find suturo_knowledge_foodreasoner");
    String arff_path = foodreasoner_path + "/raw_data/milestone4.arff";
    
    System.out.println("arff path: "+arff_path);

    try {
      BufferedReader reader = new BufferedReader(new FileReader(arff_path));
      Instances data = new Instances(reader);
      reader.close();
      data.setClassIndex(data.numAttributes() - 1);
    } catch (IOException ex) {
      ex.printStackTrace();
    }
    classifier.buildClassifier(data);

  }

  public String classifyPerceivedObject(PerceivedObject po) {
    classifier.classfyInstance(Instance);
    return "";
  }

  /**
   * executes a command. taken from http://www.mkyong.com/java/how-to-execute-shell-command-from-java/
   */
  private String executeCommand(String command) {
    StringBuffer output = new StringBuffer();
    Process p;
    try {
      p = Runtime.getRuntime().exec(command);
      p.waitFor();
      BufferedReader reader = new BufferedReader(new InputStreamReader(p.getInputStream()));
      String line = "";     
      while ((line = reader.readLine())!= null) {
        output.append(line + "\n");
      }
    } catch (Exception e) {
      e.printStackTrace();
    }
    return output.toString();
  }
}
