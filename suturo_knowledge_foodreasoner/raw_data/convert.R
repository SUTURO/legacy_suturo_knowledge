writeJavaData <- function(arffData, name) {
  bg_red <- arffData[,c('red')]
  bg_green <- arffData[,c('green')]
  bg_blue <- arffData[,c('blue')]
  bg_hue_sin <- arffData[,c('hue_sin')]
  bg_hue_cos <- arffData[,c('hue_cos')]
  bg_saturation <- arffData[,c('saturation')]
  bg_value <- arffData[,c('value')]
  bg_vol <- arffData[,c('vol')]
  bg_length_1 <- arffData[,c('length_1')]
  bg_length_2 <- arffData[,c('length_2')]
  bg_length_3 <- arffData[,c('length_3')]
  bg_cuboid_length_relation_1 <- arffData[,c('cuboid_length_relation_1')]
  bg_cuboid_length_relation_2 <- arffData[,c('cuboid_length_relation_2')]
  write(paste("    data.put(\"", name,"_red_sd\", ", sd(bg_red), ");", sep=""), dest_file, 1, 1)
  write(paste("    data.put(\"", name,"_red_mean\", ", mean(bg_red), ");", sep=""), dest_file, 1, 1)
  write(paste("    data.put(\"", name,"_green_sd\", ", sd(bg_green), ");", sep=""), dest_file, 1, 1)
  write(paste("    data.put(\"", name,"_green_mean\", ", mean(bg_green), ");", sep=""), dest_file, 1, 1)
  write(paste("    data.put(\"", name,"_blue_sd\", ", sd(bg_blue), ");", sep=""), dest_file, 1, 1)
  write(paste("    data.put(\"", name,"_blue_mean\", ", mean(bg_blue), ");", sep=""), dest_file, 1, 1)
  write(paste("    data.put(\"", name,"_hue_sin_sd\", ", sd(bg_hue_sin), ");", sep=""), dest_file, 1, 1)
  write(paste("    data.put(\"", name,"_hue_sin_mean\", ", mean(bg_hue_sin), ");", sep=""), dest_file, 1, 1)
  write(paste("    data.put(\"", name,"_hue_cos_sd\", ", sd(bg_hue_cos), ");", sep=""), dest_file, 1, 1)
  write(paste("    data.put(\"", name,"_hue_cos_mean\", ", mean(bg_hue_cos), ");", sep=""), dest_file, 1, 1)
  write(paste("    data.put(\"", name,"_saturation_sd\", ", sd(bg_saturation), ");", sep=""), dest_file, 1, 1)
  write(paste("    data.put(\"", name,"_saturation_mean\", ", mean(bg_saturation), ");", sep=""), dest_file, 1, 1)
  write(paste("    data.put(\"", name,"_value_sd\", ", sd(bg_value), ");", sep=""), dest_file, 1, 1)
  write(paste("    data.put(\"", name,"_value_mean\", ", mean(bg_value), ");", sep=""), dest_file, 1, 1)
  write(paste("    data.put(\"", name,"_vol_sd\", ", sd(bg_vol), ");", sep=""), dest_file, 1, 1)
  write(paste("    data.put(\"", name,"_vol_mean\", ", mean(bg_vol), ");", sep=""), dest_file, 1, 1)
  write(paste("    data.put(\"", name,"_length_1_sd\", ", sd(bg_length_1), ");", sep=""), dest_file, 1, 1)
  write(paste("    data.put(\"", name,"_length_1_mean\", ", mean(bg_length_1), ");", sep=""), dest_file, 1, 1)
  write(paste("    data.put(\"", name,"_length_2_sd\", ", sd(bg_length_2), ");", sep=""), dest_file, 1, 1)
  write(paste("    data.put(\"", name,"_length_2_mean\", ", mean(bg_length_2), ");", sep=""), dest_file, 1, 1)
  write(paste("    data.put(\"", name,"_length_3_sd\", ", sd(bg_length_3), ");", sep=""), dest_file, 1, 1)
  write(paste("    data.put(\"", name,"_length_3_mean\", ", mean(bg_length_3), ");", sep=""), dest_file, 1, 1)
  write(paste("    data.put(\"", name,"_cuboid_length_relation_1_sd\", ", sd(bg_cuboid_length_relation_1), ");", sep=""), dest_file, 1, 1)
  write(paste("    data.put(\"", name,"_cuboid_length_relation_1_mean\", ", mean(bg_cuboid_length_relation_1), ");", sep=""), dest_file, 1, 1)
  write(paste("    data.put(\"", name,"_cuboid_length_relation_2_sd\", ", sd(bg_cuboid_length_relation_2), ");", sep=""), dest_file, 1, 1)
  write(paste("    data.put(\"", name,"_cuboid_length_relation_2_mean\", ", mean(bg_cuboid_length_relation_2), ");", sep=""), dest_file, 1, 1)
}

library(foreign)
baguette = read.arff("raw_data/baguette.arff")
corny = read.arff("raw_data/corny.arff")
dlink = read.arff("raw_data/dlink.arff")
dest_file = "ProbabilityData.java"
write("package de.suturo.knowledge.foodreasoner;", dest_file, 1, 0)
write("import java.util.HashMap;", dest_file, 1, 1)
write("class ProbabilityData {", dest_file, 1, 1)
write("  private final HashMap<String, Double> data = new HashMap<String, Double>();", dest_file, 1, 1)
write("  public ProbabilityData() {", dest_file, 1, 1)
writeJavaData(baguette, "baguette")
write("  ", dest_file, 1, 1)
writeJavaData(corny, "corny")
write("  ", dest_file, 1, 1)
writeJavaData(dlink, "dlink")
write("  }", dest_file, 1, 1)
write("  public HashMap<String, Double> getData() { return data; }", dest_file, 1, 1)
write("}", dest_file, 1, 1)

