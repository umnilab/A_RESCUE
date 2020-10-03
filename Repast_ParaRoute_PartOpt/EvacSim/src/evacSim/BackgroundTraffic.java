package evacSim;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;
import java.util.ArrayList;
//import java.util.LinkedList;
//import java.util.Map;
//import java.util.Queue;
import java.util.TreeMap;
//import repast.simphony.essentials.RepastEssentials;
//import repast.simphony.space.gis.Geography;
import au.com.bytecode.opencsv.CSVReader;
//import evacSim.citycontext.CityContext;
//import evacSim.citycontext.Road;
import java.util.*;

/* Author: Wenbo Zhang
 * read background traffic into treemap with roadid as key and hourly link speed as arraylist
 * */


public class BackgroundTraffic{
	public static TreeMap<Integer,ArrayList<Float>> backgroundTraffic;
	//initialize everything
	public BackgroundTraffic(){
		backgroundTraffic=new TreeMap<Integer,ArrayList<Float>>();
		readEventFile();
	}
	// read and parse CSV files
	public void readEventFile() {
		File bteventFile = new File(GlobalVariables.BT_EVENT_FILE);
		CSVReader csvreader = null;
		String[] nextLine;
		int roadID = 0;
				
		try {
			csvreader = new CSVReader(new FileReader(bteventFile));
			// This is used to avoid reading the header (Data is assumed to
			// start from the second row)
			boolean readingheader = true;
			// This while loop is used to read the CSV iterating through the row
			while ((nextLine = csvreader.readNext()) != null) {
				// Do not read the first row (header)
				if (readingheader) {
					readingheader = false;
					
				} else {
					ArrayList<Float> value = new ArrayList<Float>(Collections.nCopies(24,0.0f));
					roadID = Integer.parseInt(nextLine[0]);
					for (int i=0 ; i<24 ; i++ ){
						value.set(i, Float.parseFloat(nextLine[i+1]));
					}
					BackgroundTraffic.backgroundTraffic.put(roadID, value);
	            }
			}
		} catch (FileNotFoundException e) {
			e.printStackTrace();
		} catch (IOException e) {
			e.printStackTrace();
		}
	}
	
	//public TreeMap<Integer,ArrayList<Double>> gettreemap(){
		//return backgroundTraffic;
	//}
	
}



//public class BackgroundTraffic {
	
	//public LinkedList<BackgroundTrafficObject> backgroundTraffic;
	
	// Constructor: initialize everything
	//public BackgroundTraffic() {
		//backgroundTraffic = new LinkedList<BackgroundTrafficObject>();
		//readEventFile();
	//}
	
	
	//public void readEventFile(){
		// Wenbo: implement the file read
		
		//File bteventFile = new File(GlobalVariables.BT_EVENT_FILE);
		//CSVReader csvreader = null;
		//String[] nextLine;
		//int roadID = 0;
		//ArrayList<Double> value = new ArrayList<Double>(Collections.nCopies(24,0.0d));
		
		//try {
			//csvreader = new CSVReader(new FileReader(bteventFile));
			// This is used to avoid reading the header (Data is assumed to
			// start from the second row)
			//boolean readingheader = true;

			// This while loop is used to read the CSV iterating through the row
			//while ((nextLine = csvreader.readNext()) != null) {
				// Do not read the first row (header)
				//if (readingheader) {
					//readingheader = false;
					
				//} else {
					//roadID = Integer.parseInt(nextLine[0]);
					//for (int i=0 ; i<24 ; i++ ){
					//value.set(i, Double.parseDouble(nextLine[i+1]));
					//}
					//System.out.println("roadID = "+ roadID+"value =" + value);
					
					//BackgroundTrafficObject BTObject = new BackgroundTrafficObject(roadID,value); 
					//this.backgroundTraffic.add(BTObject);
				//}
			//}
		//} catch (FileNotFoundException e) {
			//e.printStackTrace();
		//} catch (IOException e) {
			//e.printStackTrace();
		//}
	//}
	
//}