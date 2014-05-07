package de.suturo.knowledge.foodreasoner.classifier;

import java.util.HashMap;

class ProbabilityData {
    private final HashMap<String, Double> data = new HashMap<String, Double>();

    public ProbabilityData() {
	data.put("baguette_red_sd", 11.4240190518517);
	data.put("baguette_red_mean", 151.6375);
	data.put("baguette_green_sd", 8.96529707281946);
	data.put("baguette_green_mean", 87.6291666666667);
	data.put("baguette_blue_sd", 10.210954819524);
	data.put("baguette_blue_mean", 64.75);
	data.put("baguette_hue_sin_sd", 0.0302804573280007);
	data.put("baguette_hue_sin_mean", 0.957279679166667);
	data.put("baguette_hue_cos_sd", 0.124473446671674);
	data.put("baguette_hue_cos_mean", 0.259371788772436);
	data.put("baguette_saturation_sd", 0.0431876479103363);
	data.put("baguette_saturation_mean", 0.609451420833333);
	data.put("baguette_value_sd", 0.0447386359041553);
	data.put("baguette_value_mean", 0.597306766666667);
	data.put("baguette_vol_sd", 0.000157885412616177);
	data.put("baguette_vol_mean", 0.00133527541666667);
	data.put("baguette_length_1_sd", 0.00737963276730741);
	data.put("baguette_length_1_mean", 0.209933879166667);
	data.put("baguette_length_2_sd", 0.0397577738999275);
	data.put("baguette_length_2_mean", 0.189733808333333);
	data.put("baguette_length_3_sd", 0.00734979182193423);
	data.put("baguette_length_3_mean", 0.0520248516666667);
	data.put("baguette_cuboid_length_relation_1_sd", 0.290378945812223);
	data.put("baguette_cuboid_length_relation_1_mean", 1.16628216666667);
	data.put("baguette_cuboid_length_relation_2_sd", 0.669788459668063);
	data.put("baguette_cuboid_length_relation_2_mean", 4.12736195833333);

	data.put("corny_red_sd", 1.99580714613778);
	data.put("corny_red_mean", 119.295833333333);
	data.put("corny_green_sd", 4.10408511660656);
	data.put("corny_green_mean", 84.9);
	data.put("corny_blue_sd", 4.96340162071694);
	data.put("corny_blue_mean", 68.225);
	data.put("corny_hue_sin_sd", 0.0348486005905728);
	data.put("corny_hue_sin_mean", 0.952818741666667);
	data.put("corny_hue_cos_sd", 0.125863534912188);
	data.put("corny_hue_cos_mean", 0.27413809625);
	data.put("corny_saturation_sd", 0.0419938718589591);
	data.put("corny_saturation_mean", 0.489280854166667);
	data.put("corny_value_sd", 0.00641720061531263);
	data.put("corny_value_mean", 0.475098391666667);
	data.put("corny_vol_sd", 9.50353567713586e-05);
	data.put("corny_vol_mean", 0.0009352629);
	data.put("corny_length_1_sd", 0.00688437818709832);
	data.put("corny_length_1_mean", 0.146330420833333);
	data.put("corny_length_2_sd", 0.00771526465648428);
	data.put("corny_length_2_mean", 0.145693341666667);
	data.put("corny_length_3_sd", 0.00676733837919862);
	data.put("corny_length_3_mean", 0.0470241183333333);
	data.put("corny_cuboid_length_relation_1_sd", 0.012501806844029);
	data.put("corny_cuboid_length_relation_1_mean", 1.00475383333333);
	data.put("corny_cuboid_length_relation_2_sd", 0.555090201140966);
	data.put("corny_cuboid_length_relation_2_mean", 3.18962954166667);

	data.put("dlink_red_sd", 4.94081957244138);
	data.put("dlink_red_mean", 123.104166666667);
	data.put("dlink_green_sd", 3.43205620042868);
	data.put("dlink_green_mean", 120.858333333333);
	data.put("dlink_blue_sd", 7.1662806811175);
	data.put("dlink_blue_mean", 128.091666666667);
	data.put("dlink_hue_sin_sd", 0.0980043409090151);
	data.put("dlink_hue_sin_mean", 0.779065454166667);
	data.put("dlink_hue_cos_sd", 0.141720589204374);
	data.put("dlink_hue_cos_mean", -0.602902316666667);
	data.put("dlink_saturation_sd", 0.0148877476478032);
	data.put("dlink_saturation_mean", 0.339908854166667);
	data.put("dlink_value_sd", 0.0181150727117278);
	data.put("dlink_value_mean", 0.583085933333333);
	data.put("dlink_vol_sd", 0.000270319681269629);
	data.put("dlink_vol_mean", 0.00205615220833333);
	data.put("dlink_length_1_sd", 0.0077711440085488);
	data.put("dlink_length_1_mean", 0.244508191666667);
	data.put("dlink_length_2_sd", 0.0384714125281599);
	data.put("dlink_length_2_mean", 0.225319233333333);
	data.put("dlink_length_3_sd", 0.00755562881145472);
	data.put("dlink_length_3_mean", 0.0519459008333333);
	data.put("dlink_cuboid_length_relation_1_sd", 0.209034137933606);
	data.put("dlink_cuboid_length_relation_1_mean", 1.12022633333333);
	data.put("dlink_cuboid_length_relation_2_sd", 0.752384272934509);
	data.put("dlink_cuboid_length_relation_2_mean", 4.811517375);
    }

    public HashMap<String, Double> getData() {
	return data;
    }
}
