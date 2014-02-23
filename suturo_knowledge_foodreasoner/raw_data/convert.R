library(foreign)
data = read.arff("baguette.arff")
hue_sin <- data[,c('hue_sin')]
sd(hue_sin)
mean(hue_sin)
