package evacSim.citycontext;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;
import java.net.URI;
import java.util.ArrayList;
import java.util.HashMap;

import org.apache.log4j.Logger;

import evacSim.routing.SOShelterRouting;
import repast.simphony.context.DefaultContext;
import repast.simphony.context.space.gis.GeographyFactoryFinder;
import repast.simphony.engine.environment.RunEnvironment;
import repast.simphony.space.gis.Geography;
import repast.simphony.space.gis.GeographyParameters;
import repast.simphony.space.gis.ShapefileLoader;
import evacSim.ContextCreator;
import evacSim.GlobalVariables;
import evacSim.data.DataCollector;
import evacSim.demand.DatasetOfHouseholdsPerZones;
import repast.simphony.parameter.Parameters;

public class ZoneContext extends DefaultContext<Zone> {
	public DatasetOfHouseholdsPerZones dataset;
	// RV: shelter SO routing matcher object
	public SOShelterRouting soShelterMatcher;
	// RV: added for quick access in this.recordShelterStatus()
	public ArrayList<Zone> shelters;
	Logger logger = ContextCreator.logger;

	public ZoneContext() {

		super("ZoneContext");

		logger.info("ZoneContext creation");
		/* GIS projection for spatial information about Roads. This is used to
		 * then create junctions and finally the road network. */
		GeographyParameters<Zone> geoParams = new GeographyParameters<Zone>();
		Geography<Zone> zoneGeography = GeographyFactoryFinder
				.createGeographyFactory(null)
				.createGeography("ZoneGeography", this, geoParams);

		/* Read in the data and add to the context and geography */
		try {
			/* Load the demand zones (census block groups) */
			logger.info("Initializing demand zones only");
			File zoneFile = null;
			ShapefileLoader<Zone> zoneLoader = null;
			zoneFile = new File(GlobalVariables.ZONES_SHAPEFILE);
			URI uri=zoneFile.toURI();
			zoneLoader = new ShapefileLoader<Zone>(Zone.class,
					uri.toURL(), zoneGeography, this);
			int int_id =  1;
			while (zoneLoader.hasNext()) {
				Zone zone = zoneLoader.nextWithArgs(int_id);
				zone.setGeometry(zoneGeography);
				int_id += 1;
			}

			/* Load the emergency shelters (as zones) */
			File shelterFile = null;
			String shelterCsvName = null;
			ShapefileLoader<Zone> shelterLoader = null;
			ArrayList<Zone> shelters = new ArrayList<Zone>();
			logger.info("Initializing shelters");

			// read the shelters shape-file
			shelterFile = new File(GlobalVariables.SHELTERS_SHAPEFILE);
			uri = shelterFile.toURI();
			shelterLoader = new ShapefileLoader<Zone>(Zone.class,
					uri.toURL(), zoneGeography, this);

			// read the shelter attributes CSV file
			shelterCsvName = GlobalVariables.SHELTERS_CSV;
			BufferedReader br = new BufferedReader(new FileReader(shelterCsvName));
			int lineNum = 0;
			while (shelterLoader.hasNext()) {
				lineNum++;
				String line = br.readLine();
				if (lineNum == 1) continue;
				String[] result = line.split(",");
				Zone shelter = shelterLoader.nextWithArgs(-Integer.parseInt(result[0]));
				// set the fields
				shelter.setName(result[2]);
				shelter.setType(1);
				shelter.setCapacity(Integer.parseInt(result[7]));
				shelter.setOccupancy(0);
				shelter.setGeometry(zoneGeography);
				shelters.add(shelter);
			}
			br.close();

			// create the SO routing scheduler
			this.soShelterMatcher = new SOShelterRouting(shelters);
			logger.info("Created SO shelter matcher: " + this.soShelterMatcher.toString());

			// RV: also add the list of shelter objects so that
			this.shelters = shelters;

		} catch (java.net.MalformedURLException e) {
			System.err
			.println("Malformed URL exception when reading housesshapefile.");
			e.printStackTrace();
		} catch (FileNotFoundException e){
			System.out
			.println("ContextCreator: No road csv file found");
			e.printStackTrace();
		} catch (IOException e) {
			e.printStackTrace();
		}

		// RV: Shelter matching for excess shelter seekers
		if (GlobalVariables.DYNAMIC_DEST_STRATEGY == 3) {
			logger.info("Trying shelter matching!");
		}

		// SH - for implementing activity simulator
		if (GlobalVariables.SET_DEMAND_FROM_ACTIVITY_MODELS) {
			/* HG, CT: this is to be true when we plan to run 
			 * multiple instances of demand simultaneously in batches. 
			 * Before setting it true, we need to have demand files numbered
			 * 1,2,.. etc. in the folder multiple_instances_of_demand
			 * (the current demand files are copies of
			 * evacuation_only_2005_low_about200vehicles_noduplicates_sameOD_trialjacksonville.csv)
			 * */
			if (GlobalVariables.SIMULATION_MULTIPLE_DEMAND_INPUTS) {
				Parameters params = RunEnvironment.getInstance().getParameters();
				int demandNumber = params.getInteger("demandFile");
				System.out.println("Demand file number is " + demandNumber);
				String folder = GlobalVariables.MULTIPLE_DEMAND_CSV_DIR;
				String filepath = folder + "/" + demandNumber + ".csv";
				logger.info("data file: " + filepath);
				dataset = new DatasetOfHouseholdsPerZones(filepath);
			} else {
				String filepath = GlobalVariables.ACTIVITY_CSV;
				logger.info("data file: "+filepath);
				dataset = new DatasetOfHouseholdsPerZones(filepath);
			}
			HashMap<Integer, ArrayList<House>> housesbyzone = dataset.
					getHousesByHour().pollFirstEntry().getValue();
			for (Zone z : zoneGeography.getAllObjects()) {
				int keyzone = z.getIntegerId();
				if (housesbyzone.containsKey(keyzone)){
					ArrayList<House> temparraylist = housesbyzone.get(keyzone);
					z.setHouses(temparraylist);
					for (House hh : z.getHouses()) {
						hh.setZone(z);
					}
				}
			}
		}
	}

	public void loadDemandofNextHour() {
		HashMap<Integer, ArrayList<House>> housesbyzone = dataset.
				getHousesByHour().pollFirstEntry().getValue(); // turnOnAfterTest
		for (Zone z : ContextCreator.getZoneGeography().getAllObjects()) {
			int keyzone = z.getIntegerId();
			if (housesbyzone.containsKey(keyzone)){
				ArrayList<House> temparraylist = housesbyzone.get(keyzone);
				z.setHouses(temparraylist);
				for (House hh : z.getHouses()) {
					hh.setZone(z);
				}
			}
			else {
				ArrayList<House> temparraylist = new ArrayList<House>();
				z.setHouses(temparraylist);
			}
		}
	}

	/**
	 * RV: Whenever vehicles are recorded for visualization,
	 * also record the status of all the shelters. This function
	 * is scheduled in `cityContext`.
	 */
	public void recordShelterStatus() {
		for (Zone shelter : this.shelters) {
			if (shelter.getOccupancy() != shelter.getLastRecordedOccupancy()) {
				try {
					DataCollector.getInstance().recordShelterTickSnapshot(shelter);
				} catch (Exception e) {
					logger.error(e);
				}
			}
		}
	}

}
