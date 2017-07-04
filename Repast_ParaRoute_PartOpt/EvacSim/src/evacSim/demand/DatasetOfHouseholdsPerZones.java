package evacSim.demand;

import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;
import java.util.ArrayList;
import java.util.HashMap;

import au.com.bytecode.opencsv.CSVReader;

import evacSim.GlobalVariables;
import evacSim.citycontext.House;

public class DatasetOfHouseholdsPerZones {
	private HashMap<Integer, ArrayList<House>> housesbyzone;
	private HashMap<Integer, House> housesbyID;

	// constructor to use with CSV file
	public DatasetOfHouseholdsPerZones(String filepath) {
		if (GlobalVariables.SET_DEMAND_FROM_ACTIVITY_MODELS){
			this.readActivityPlans(filepath);
		}

	}

	public void readActivityPlans(String filepath) {
		//if (ContextCreator.debug)
		//	System.out.println("Reading Activity Plans Data");
		
		CSVReader csvreader = null;
		String[] nextLine;
		housesbyzone = new HashMap<Integer, ArrayList<House>>();
		housesbyID = new HashMap<Integer, House>();
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
			int loc = 0;
			float dur = 0.0f;
			int zoneID =0;
			//int prevID = 1; // Has to be the ID of first person not so good
			// appraoch			
			int prevID = 0; // assign to the first row			
			ArrayList<Integer> locations = new ArrayList<Integer>();
			ArrayList<Float> durations = new ArrayList<Float>();

			while ((nextLine = csvreader.readNext()) != null) {
				// Do not read the first row (header)
				if (readingheader) {
					readingheader = false;
				} else {
					// Sets the number of columns in the CSV
					int length = nextLine.length;
					//System.out.println(" Size of the first list: " + length);
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
							//House h = this.housesbyID.get(prevID);
							zoneID = locations.get(0);
							House h = new House (prevID, zoneID);
							h.setActivityPlan(locations, durations);
							if (!housesbyzone.containsKey(zoneID)){
								ArrayList<House> arraylistwithfirsthouse = new ArrayList<House>();
								arraylistwithfirsthouse.add(h);
								housesbyzone.put(zoneID, arraylistwithfirsthouse);
								housesbyID.put(ID, h);
							}
							else{
								housesbyID.put(ID, h);
								housesbyzone.get(zoneID).add(h);
							}
			
							locations.clear();
							durations.clear();
							// clear the location and duration arrays
						}
						prevID = ID;
						// location ID
						if (j == 1) {
							loc = Integer.parseInt(nextLine[j]);
							locations.add(loc);
						}
						// duration
						if (j == 2) {
							dur = Float.parseFloat(nextLine[j]);
							durations.add(dur);
						}
					}
					//System.out.println("ID " + ID + " location " + loc
					//		+ " duration " + dur);
				}
			}
		} catch (FileNotFoundException e) {
			e.printStackTrace();
		} catch (IOException e) {
			e.printStackTrace();
		}
		// return housesbyzone;
	}

	public HashMap<Integer, ArrayList<House>> getHousesByZone() {
		return housesbyzone;
	}

}