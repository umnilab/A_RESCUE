package evacSim.demand;

import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.TreeMap;

import au.com.bytecode.opencsv.CSVReader;
import evacSim.GlobalVariables;
import evacSim.citycontext.House;

public class DatasetOfHouseholdsPerZones {
	//private HashMap<Integer, ArrayList<House>> housesbyzone;
	//private HashMap<Integer, House> housesbyID; // unused variable
	private TreeMap<Integer,HashMap<Integer, ArrayList<House>>> housesbyhour;

	// constructor to use with CSV file
	public DatasetOfHouseholdsPerZones(String filepath) {
		if (GlobalVariables.SET_DEMAND_FROM_ACTIVITY_MODELS){
			this.readActivityPlans(filepath);
		}
	}

	public void readActivityPlans(String filepath) {
		
		CSVReader csvreader = null;
		String[] nextLine;
		housesbyhour = new TreeMap<Integer,HashMap<Integer, ArrayList<House>>>();
		//housesbyID = new HashMap<Integer, House>();
		// the hashmap housesbyzone temporally organizes and relates houses read
		// from the CSV with their corresponding zones
		// housesbyzone = new HashMap<Integer, ArrayList<House>>();
		try {
			csvreader = new CSVReader(new FileReader(filepath));
			// This is used to avoid reading the header (Data is assumed to
			// start from the second row)
			boolean readingheader = true;
			boolean firsthouse = true;

			// This while loop is used to read the CSV iterating through the row
			int ID = 0;
			int zoneID = 0;
			int departureTime = 0;
			// appraoch
			int prevID = 0; // assign to the first row			
			ArrayList<Integer> locations = new ArrayList<Integer>();
			ArrayList<Integer> durations = new ArrayList<Integer>();

			while ((nextLine = csvreader.readNext()) != null) {
				// Do not read the first row (header)
				if (readingheader) {
					readingheader = false;
				} else {
					// Sets the number of columns in the CSV
					int length = nextLine.length;
					// for loop that iterates through the columns
					for (int j = 0; j < length; j++) {
						// USer ID
						if (j == 0) {
							ID = Integer.parseInt(nextLine[j]);
							if (firsthouse) {
								prevID = ID;
								firsthouse = false;
							}
						}
						if (ID != prevID) {
							// input the arrays to the house's plan
							zoneID = locations.get(0);
							departureTime =  (int) Math.floor(durations.get(0)/60);
							House h = new House(prevID, zoneID);
							h.setActivityPlan(locations, durations);
							
							// RV: count the total no. of houses created so far
							GlobalVariables.NUM_HOUSES++;
							
							if (!housesbyhour.containsKey(departureTime)) {
								HashMap<Integer, ArrayList<House>> housebyzone = new HashMap<Integer, ArrayList<House>>();
								housesbyhour.put(departureTime, housebyzone);
							}
							if (!housesbyhour.get(departureTime).containsKey(zoneID)) {
								ArrayList<House> arraylistwithfirsthouse = new ArrayList<House>();
								arraylistwithfirsthouse.add(h);
								housesbyhour.get(departureTime).put(zoneID, arraylistwithfirsthouse);
							}
							else {
								housesbyhour.get(departureTime).get(zoneID).add(h);
							}
//							if (!housesbyzone.containsKey(zoneID)){
//								ArrayList<House> arraylistwithfirsthouse = new ArrayList<House>();
//								arraylistwithfirsthouse.add(h);
//								housesbyzone.put(zoneID, arraylistwithfirsthouse);
//								//housesbyID.put(ID, h);
//							}
//							else{
//								//housesbyID.put(ID, h);
//								housesbyzone.get(zoneID).add(h);
//							}
							// clear the arrays
							locations.clear();
							durations.clear();
						}
						prevID = ID;
						// location ID
						if (j == 1) {
							locations.add(Integer.parseInt(nextLine[j]));
						}
						// duration
						if (j == 2) {
							durations.add(Integer.parseInt(nextLine[j]));
						}
					}
				}
			}
			System.out.println("Total houses generated: " + GlobalVariables.NUM_HOUSES);
			
		} catch (FileNotFoundException e) {
			e.printStackTrace();
		} catch (IOException e) {
			e.printStackTrace();
		}
		// return housesbyzone;
	}

	public TreeMap<Integer,HashMap<Integer, ArrayList<House>>> getHousesByHour() {
		return housesbyhour;
	}

}