package evacSim;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;
import java.util.ArrayList;
import java.util.TreeMap;
import au.com.bytecode.opencsv.CSVReader;
import java.util.*;

/**
 * Author: Wenbo Zhang
 * read background traffic into treemap with roadid as key and hourly link
 * speed as arraylist
 */
public class BackgroundTraffic{
	public static TreeMap<Integer,ArrayList<Float>> backgroundTraffic;
	// no. of hours in the demand file
	public static int numHours = GlobalVariables.HOUR_OF_SPEED;
	
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
					ArrayList<Float> value = new ArrayList<Float>(
							Collections.nCopies(numHours, 0.0f));
					roadID = Integer.parseInt(nextLine[0]);
					for (int i=0 ; i<numHours ; i++ ) {
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
}
