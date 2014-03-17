package de.suturo.knowledge.barcodeparser

import java.net.*;
import java.io.*;
import java.util.*;
import java.lang.*;

import ros.NodeHandle;
import ros.RosException;
import ros.ServiceClient;

public class suturo_barcode_reader{

	public static void main(String[] args) throws MalformedURLException, IOException{

		String bezeichnung = "";
 		String kohlenhydrate ="";
 		String fett ="";
 		String eiweiß ="";
 		String ballaststoffe ="";
 		String brennwert ="";
 		String kalorien ="";
 		String zucker ="";
 		String broteinheiten ="";

 		String result = "";

        StringBuilder sbQuell = new StringBuilder();
        try 
        {
        	Scanner scanner = new Scanner(new URL("http://www.barcoo.com/4014400400007?source=pb").openStream());
            while (scanner.hasNextLine())
            {
            	sbQuell.append(scanner.nextLine() + "\n");
            	if (scanner.findInLine("title\" content=\"") != null) 
            	{
            		bezeichnung += scanner.nextLine();
            		bezeichnung += scanner.nextLine();
            		bezeichnung = bezeichnung.replace("\" />  <meta property=\"og:type\" content=\"product\"/>","");	
            	} else if (scanner.findInLine(">Kohlenhydrate</a></td>") != null)
            	{	
            		kohlenhydrate += scanner.nextLine();
            		kohlenhydrate += scanner.nextLine();
            		kohlenhydrate = kohlenhydrate.replace("<td>","").replace("</td>","").replace("    ","");
            	} else if (scanner.findInLine("<td>Fett</td>") != null)
            	{	
            		fett += scanner.nextLine();
            		fett += scanner.nextLine();
            		fett = fett.replace("<td>","").replace("</td>","").replace("    ","");
            	} else if (scanner.findInLine("<td>Eiweiß</td>") != null)
            	{	
            		eiweiß += scanner.nextLine();
            		eiweiß += scanner.nextLine();
            		eiweiß = eiweiß.replace("<td>","").replace("</td>","").replace("    ","");
            	} else if (scanner.findInLine("<td>Ballaststoffe</td>") != null)
            	{	
            		ballaststoffe += scanner.nextLine();
            		ballaststoffe += scanner.nextLine();
            		ballaststoffe = ballaststoffe.replace("<td>","").replace("</td>","").replace("    ","");
            	} else if (scanner.findInLine("<td>Brennwert</td>") != null)
            	{	
            		brennwert += scanner.nextLine();
            		brennwert += scanner.nextLine();
            		brennwert = brennwert.replace("<td>","").replace("</td>","").replace("    ","");
            	} else if (scanner.findInLine(">Kalorien</a></td>") != null)
            	{	
            		kalorien += scanner.nextLine();
            		kalorien += scanner.nextLine();
            		kalorien = kalorien.replace("<td>","").replace("</td>","").replace("    ","");
            	} else if (scanner.findInLine("<td>Zucker</td>") != null)
            	{	
            		zucker += scanner.nextLine();
            		zucker += scanner.nextLine();
            		zucker = zucker.replace("<td>","").replace("</td>","").replace("    ","");
            	} else if (scanner.findInLine("<td>Broteinheiten</td>") != null)
            	{	
            		broteinheiten += scanner.nextLine();
            		broteinheiten += scanner.nextLine();
            		broteinheiten = broteinheiten.replace("<td>","").replace("</td>","").replace("    ","");
            	} 
            }
            scanner.close();
        }
        catch (MalformedURLException e)
        {
        	e.printStackTrace();
        }
        catch (IOException e)
        {
        	e.printStackTrace();
        }
        //System.out.println(sbQuell.toString());
        System.out.println(bezeichnung);
        System.out.println(kohlenhydrate);
        System.out.println(fett);
        System.out.println(eiweiß);
        System.out.println(ballaststoffe);
        System.out.println(brennwert);
        System.out.println(kalorien);
        System.out.println(zucker);
        System.out.println(broteinheiten);

        /*result += kohlenhydrate+fett+eiweiß+ballaststoffe+brennwert+kalorien+zucker+broteinheiten;

        return result;*/
	}
}
