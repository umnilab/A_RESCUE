/**
 Copyright 2008 Nick Malleson/Samiul Hasan/Xianyuan Zhan

 This file is part of RepastCity.

 RepastCity is free software: you can redistribute it and/or modify it under the terms of the GNU
 General Public License as published by the Free Software Foundation, either version 3 of the 
 License, or (at your option) any later version.

 RepastCity is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without 
 even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU 
 General Public License for more details.

 You should have received a copy of the GNU General Public License along with RepastCity. If not, 
 see <http://www.gnu.org/licenses/>.
 */
package evacSim;

/**
 * Central location for any useful variables (e.g. filenames).
 * 
 * @author Nick Malleson
 * @author Samiul Hasan (SH)
 * @author Xianyuan Zhan0string type.
 * 
 * To add a new global variable: 
 * 1. Define the variable's name and value in the Data.properties
 * 2. Adding corresponding global variable in the GlobalVariables.java
 * e.g.: To add int type new variable "test", first give value for test i	n Data.properties, as
 * 		 test = 0.5
 * 		 Then, create corresponding declaration for variable test, as
 * 		 public static final int test = Integer.valueOf(loadConfig("test"))
 * The loadConfig method is used to load property's value from the config file, and data type 
 * transform is required. Useful transform method is list below:
 * - to integer: Integer.valueOf(String s)
 * - to Float: Float.valueOf(String s)
 * - to Double: Double.valueOf(String s)
 * - to Boolean: Boolean.valueOf(String s)
 * - to single char: .atChar(0),
 * 		e.g.: public static final char whosRunning = loadConfig("whosRunning").charAt(0);
 * 
 * Note:
 * 1. The value for each property in Config File should only include numeric data or string, 
 * such content of "/", "0.05f", "0.05 " cannot be reconized
 * 2. To add comment in any content, use "#", instead of "//" or "/*...". e.g. : "#Input Files"
 */

import java.io.FileInputStream;
import java.io.IOException;
import java.util.ArrayList;
import java.util.LinkedList;
import java.util.Properties;
import java.util.Random;

public class GlobalVariables {
private static Properties config;
	/* Loading properties from ConfigFile, by Zhan. Modified by Chris later. */
	private static String loadConfig(String property) {
	    if (config == null) {
	        config = new Properties();
	        try {
			    config.load(new FileInputStream("data/Data.properties"));
//			    config.load(new FileInputStream("dynamic_dest_test/properties/case_3-1.properties"));
//			    config.load(new FileInputStream("config/Data.properties"));
		    } catch (IOException ex) {
			    ex.printStackTrace();
		    }
	    }
		return config.getProperty(property);
	}

	/* Following global variables are reading from the ConfigFile, by Zhan: */
	/* Input Files */
	public static final String ROADS_SHAPEFILE = loadConfig("ROADS_SHAPEFILE");

	public static final String LANES_SHAPEFILE = loadConfig("LANES_SHAPEFILE");

	public static final String ZONES_SHAPEFILE = loadConfig("ZONES_SHAPEFILE");
	
	public static final String SHELTERS_SHAPEFILE = loadConfig("SHELTERS_SHAPEFILE");
	
	public static final String ROADS_CSV = loadConfig("ROADS_CSV");

	public static final String LANES_CSV = loadConfig("LANES_CSV");

	public static final String SHELTERS_CSV = loadConfig("SHELTERS_CSV");
	
	public static final String ACTIVITY_CSV = loadConfig("ACTIVITY_SEQ_CSV");

	/* Path for the supply side event file */
	public static final String EVENT_FILE = loadConfig("EVENT_FILE");
	public static final int EVENT_CHECK_FREQUENCY = Integer.
			valueOf(loadConfig("EVENT_CHECK_FREQUENCY"));
	
	/* Path for background traffic event file by Wenbo */
	public static final String BT_EVENT_FILE=loadConfig("BT_EVENT_FILE");
	
	/* Simulation Setup */
	public static final Random RandomGenerator = new Random(123456777); // 123456777
																		// generate
																		// problem
	
	public static final float SIMULATION_STEP_SIZE = Float
			.valueOf(loadConfig("SIMULATION_STEP_SIZE"));
	
	public static final int SIMULATION_NETWORK_REFRESH_INTERVAL = Integer
			.valueOf(loadConfig("SIMULATION_NETWORK_REFRESH_INTERVAL"));
	
	public static final int SIMULATION_PARTITION_REFRESH_INTERVAL = Integer
			.valueOf(loadConfig("SIMULATION_PARTITION_REFRESH_INTERVAL"));
	/* Maximum network partitioning interval */
	public static final int SIMULATION_MAX_PARTITION_REFRESH_INTERVAL = Integer
			.valueOf(loadConfig("SIMULATION_MAX_PARTITION_REFRESH_INTERVAL"));
	/* Threshold amount of vehicles that requires more frequent network partitioning */
	public static final int THRESHOLD_VEHICLE_NUMBER = Integer
			.valueOf(loadConfig("THRESHOLD_VEHICLE_NUMBER"));
	/* Gehlot and Qian: This variable decides if simulator pauses for sometime to
	 * listen visualization. If zero then it waits, else it moves forward.*/
	public static int SIMULATION_SLEEPS = Integer
			.valueOf(loadConfig("SIMULATION_SLEEPS")); 
	
	/* For global variables of the adaptive network weighting */
	public static final int PART_ALPHA = Integer
			.valueOf(loadConfig("PART_ALPHA"));
	public static final int PART_BETA = Integer
			.valueOf(loadConfig("PART_BETA"));
	public static final int PART_GAMMA = Integer
			.valueOf(loadConfig("PART_GAMMA"));
	/* No. of times that the partition interval is larger than the network refresh interval */
	public static final int PART_REFRESH_MULTIPLIER = (int) 
			(SIMULATION_PARTITION_REFRESH_INTERVAL / SIMULATION_NETWORK_REFRESH_INTERVAL);
	/* Gehlot: If this is true then we input multiple demand files for batch runs,
	 * else we run single demand file that is inputed by ACTIVITY_CSV */
	public static final boolean SIMULATION_MULTIPLE_DEMAND_INPUTS = Boolean
			.valueOf(loadConfig("SIMULATION_MULTIPLE_DEMAND_INPUTS"));
	public static final int SIMULATION_INTERVAL_SIZE = Integer
			.valueOf(loadConfig("SIMULATION_INTERVAL_SIZE"));
	public static final int SIMULATION_STOP_TIME = Integer
			.valueOf(loadConfig("SIMULATION_STOP_TIME"));
	public static final double TRAVEL_PER_TURN = Double
			.valueOf(loadConfig("TRAVEL_PER_TURN"));
	// RV: if change in lat/lon of vehicle per turn is less than this, make it zero
	// this is useful to overcome the precision error in GeodeticCalculator
	public static final double MIN_DX_DY_PER_TURN = Double
			.valueOf(loadConfig("MIN_DX_DY_PER_TURN"));
	
	// RV: Stop the simulation if all vehicles have been killed
	public static final boolean ENABLE_SIMULATION_STOP_IF_NO_VEHICLE =
			Boolean.valueOf(loadConfig("ENABLE_SIMULATION_STOP_IF_NO_VEHICLE"));
	public static final int SIMULATION_STOP_IF_NO_VEHICLE_CHECK_INTERVAL =
			Integer.valueOf(loadConfig("SIMULATION_STOP_IF_NO_VEHICLE_CHECK_INTERVAL"));
	// RV: no. of houses in the demand file; used to check when to stop the simulation
	public static int NUM_HOUSES = 0;

	public static final int Global_Vehicle_ID = Integer
			.valueOf(loadConfig("Global_Vehicle_ID"));
	public static final int Global_Road_ID = Integer
			.valueOf(loadConfig("Global_Road_ID"));
	public static final boolean Debug_On_Road = Boolean
			.valueOf(loadConfig("Debug_On_Road"));

	public static final double XXXX_BUFFER = Double
			.valueOf(loadConfig("XXXX_BUFFER")); // used in CityContext.getRoadAtCoords()
	public static final boolean MULTI_THREADING = Boolean
			.valueOf(loadConfig("MULTI_THREADING"));
	
	public static final int N_THREADS = Integer
			.valueOf(loadConfig("N_THREADS"));
	
	/* Load the number of partitions from the config file */
	public static final int N_Partition = Integer.valueOf(loadConfig("N_THREADS"));

//	public static final float FREE_SPEED = Float.valueOf(loadConfig("FREE_SPEED"));

	public static final float MAX_ACCELERATION = Float
			.valueOf(loadConfig("MAX_ACCELERATION")); // meter/sec2
	public static final float MAX_DECELERATION = Float
			.valueOf(loadConfig("MAX_DECELERATION")); // meter/sec2

	public static final float DEFAULT_VEHICLE_WIDTH = Float
			.valueOf(loadConfig("DEFAULT_VEHICLE_WIDTH")); // meters
	public static final float DEFAULT_VEHICLE_LENGTH = Float
			.valueOf(loadConfig("DEFAULT_VEHICLE_LENGTH")); // meters

	public static final double SPEED_EPSILON = Double
			.valueOf(loadConfig("SPEED_EPSILON")); // meter/sec
	public static final double ACC_EPSILON = Double
			.valueOf(loadConfig("ACC_EPSILON")); // meter/sec2

	public static final float LANE_WIDTH = Float
			.valueOf(loadConfig("LANE_WIDTH"));

	public static final float H_UPPER = Float.valueOf(loadConfig("H_UPPER"));
	public static final float H_LOWER = Float.valueOf(loadConfig("H_LOWER"));

	public static final double FLT_INF = Float.MAX_VALUE;
	public static final double FLT_EPSILON = 1.0 / FLT_INF;

	public static final int STATUS_REGIME_FREEFLOWING = 0x00000000; // 0
	public static final int STATUS_REGIME_CARFOLLOWING = 0x00000080; // 128
	public static final int STATUS_REGIME_EMERGENCY = 0x00000100; // 256

	public static final float ALPHA_DEC = Float
			.valueOf(loadConfig("ALPHA_DEC"));
	public static final float BETA_DEC = Float.valueOf(loadConfig("BETA_DEC"));
	public static final float GAMMA_DEC = Float
			.valueOf(loadConfig("GAMMA_DEC"));

	public static final float ALPHA_ACC = Float
			.valueOf(loadConfig("ALPHA_ACC"));
	public static final float BETA_ACC = Float.valueOf(loadConfig("BETA_ACC"));
	public static final float GAMMA_ACC = Float
			.valueOf(loadConfig("GAMMA_ACC"));

//	public static final boolean APPROX_DYNAMIC_ROUTING = Boolean
//			.valueOf(loadConfig("APPROX_DYNAMIC_ROUTING")); // SH - created this
//															// variable to
//	// enable dynamic routing
	public static final boolean SINGLE_SHORTEST_PATH = Boolean
			.valueOf(loadConfig("SINGLE_SHORTEST_PATH")); // SH - created this
															// variable to
															// enable
	// dynamic routing
	// Both SINGLE_SHORTEST_PATH and K_SHORTEST_PATH can't be true or false if
	// APPROX_DYNAMIC_ROUTING= true
	public static final boolean K_SHORTEST_PATH = Boolean
			.valueOf(loadConfig("K_SHORTEST_PATH")); // SH - created this
														// variable to enable
	// dynamic routing

	public static final int K_VALUE = Integer.valueOf(loadConfig("K_VALUE"));
	public static final double THETA_LOGIT = Double
			.valueOf(loadConfig("THETA_LOGIT"));
	
	/* Number of future road segments to be considered in counting shadow vehicles */
	public static final int N_SHADOW = Integer.valueOf(loadConfig("N_SHADOW"));

	/*
	 * BL: Following are parameters used in lane changing model
	 */
	// public static final double minLead = 0.914; //(m/sec)
	// public static final double minLag = 1.524; //(m/sec)
	public static final double minLead = 3.0; // (m/sec)
	public static final double minLag = 5.0; // (m/sec)

	/* Parameters for MLC */
	public static final double betaLeadMLC01 = 0.05;
	public static final double betaLeadMLC02 = 0.15;
	public static final double betaLagMLC01 = 0.15;
	public static final double betaLagMLC02 = 0.40;
	public static final double gama = 2.5e-5;
	public static final double critDisFraction = 0.6;

	/* Parameters for DLC */
	public static final double betaLeadDLC01 = 0.05;
	public static final double betaLeadDLC02 = 0.15;
	public static final double betaLagDLC01 = 0.15;
	public static final double betaLagDLC02 = 0.40;
	public static final double minLeadDLC = 0.05;
	public static final double minLagDLC = 0.05;

	public static final String DB_URL = String.valueOf(loadConfig("DB_URL"));

	public static final boolean SET_DEMAND_FROM_ACTIVITY_MODELS = Boolean
			.valueOf(loadConfig("SET_DEMAND_FROM_ACTIVITY_MODELS"));
	
	/** Chris:
	 * DEBUG_DATA_BUFFER and DEBUG_NETWORK are the ones that determine whether or not the 
	 * data collection classes or the network classes print their debugging statements to the 
	 * simulation stdout.  I checked it into the repository with them both disabled.
	 * 
	 * ENABLE_CSV_WRITE and ENABLE_NETWORK set whether or not the CSV file is written to the 
	 * local disk and whether or not the simulation listens for incoming connections. I committed
	 * the file with both enabled.
	 * 
	 * DATA_CLEANUP_REFRESH is set to 30 seconds and determines how often the data buffer will 
	 * delete old items that are no longer needed.  Things are only removed from the buffer once 
	 * all the users of the buffer report that they are processing a tick # that is greater, i.e.,
	 * if the CSV file writer says that it is writing tick #1500 to disk and the network says it
	 * is sending tick #1600, then everything before 1500 can be removed as already used by all
	 * but everything between 1500-1600 will remain. The processing threads for the CSV file
	 * writing and the network sending will each read all the items in the buffer until they run
	 * to the end of the buffer. Then they will wait a short while before checking to see if new
	 * items have arrived in the buffer.
	 *   
	 * CSV_BUFFER_REFRESH and NETWORK_BUFFER_REFRESH determine how long that wait is. For the CSV
	 * writing, it is 5 seconds, and for the network sending it is 2.5 seconds. There is no real
	 * reason why they need to be these values. They just seemed like reasonable times to use.
	 * Separate from the actual data sending, the socket is also periodically reporting the status
	 * of the simulation  (not running, loading, processing tick #2345, etc.) to the remote
	 * programs that are listening.
	 *   
	 * NETWORK_STATUS_REFRESH is how often those status message send. It is set to 5 seconds.
	 * NETWORK_LISTEN_PORT is the port number to use when connecting to the simulation. As I
	 * mentioned last week, the default is 8080 but that is a very commonly used port by web
	 * servers which may already be in use or blocked by a firewall. It is set it for now as 33131,
	 * which is a reference to Miami (the zipcode of the downtown area). You can change this to any
	 * value higher than 1024 and lower than 65535.
	 * 
	 * NETWORK_MAX_MESSAGE_SIZE is the size (in bytes) of the largest message to be sent over the
	 * socket. Sockets technically have no size limitation, but some of the socket libraries have
	 * a very low default limit on what they will accept if you do not explicitly set this value.
	 * I set it to 16MB which is multiple magnitudes higher than any sizes IæŠ³e seen during
	 * testing, but we may need to increase it in the future if the number of vehicle movements in
	 * one tick gets very larger.The remaining three values pertain to the CSV file location. The
	 * CSV file writer allows you to specify exactly where the file is to be placed on disk if we
	 * add a UI element to handle that or just code in a fixed location. If you do not give it a
	 * location, though, it will do what it is doing now: write a file with a unique filename to a
	 * default location. It will be written to the location
	 * <CSV_DEFAULT_PATH>/<CSV_DEFAULT_FILENAME>_<current timestamp>.<CSV_DEFAULT_EXTENSION>.
	 * If the CSV_DEFAULT_PATH value is blank (as I have committed it), the user's home directory
	 * will be used. So, as it is committed right now, every time the simulation is run, it will
	 * create something like vacSimOutput_2017-11-29-0255.csv in your home directory.
	 * */
	
	/* Parameters for the data collection buffer */
	public static final boolean ENABLE_DATA_COLLECTION = 
			Boolean.valueOf(loadConfig("ENABLE_DATA_COLLECTION"));
	public static final boolean DEBUG_DATA_BUFFER =
	        Boolean.valueOf(loadConfig("DEBUG_DATA_BUFFER"));
	public static final int DATA_CLEANUP_REFRESH =
	        Integer.valueOf(loadConfig("DATA_CLEANUP_REFRESH"));
	
	/* Parameters for the CSV output file writer */
	public static final boolean ENABLE_CSV_WRITE =
	        Boolean.valueOf(loadConfig("ENABLE_CSV_WRITE"));
	public static final String CSV_DEFAULT_FILENAME =
	        loadConfig("CSV_DEFAULT_FILENAME");
	public static final String CSV_DEFAULT_EXTENSION =
	        loadConfig("CSV_DEFAULT_EXTENSION");
	public static final String CSV_DEFAULT_PATH = 
	        loadConfig("CSV_DEFAULT_PATH");
	public static final int CSV_BUFFER_REFRESH =
	        Integer.valueOf(loadConfig("CSV_BUFFER_REFRESH"));
	public static final int CSV_LINE_LIMIT =
	        Integer.valueOf(loadConfig("CSV_LINE_LIMIT"));
	
	/* Parameters for the JSON output file writer (similar to as the csv parameters except
	 * JSON_TICK_LIMIT_PER_FILE which represents the number of ticks are written in a json file) */
	public static final boolean ENABLE_JSON_WRITE =
	        Boolean.valueOf(loadConfig("ENABLE_JSON_WRITE"));
	public static final String JSON_DEFAULT_FILENAME =
	        loadConfig("JSON_DEFAULT_FILENAME");
	public static final String JSON_DEFAULT_EXTENSION =
	        loadConfig("JSON_DEFAULT_EXTENSION");
	public static final String JSON_DEFAULT_PATH = 
	        loadConfig("JSON_DEFAULT_PATH");
	public static final int JSON_BUFFER_REFRESH =
	        Integer.valueOf(loadConfig("JSON_BUFFER_REFRESH"));
	public static final int JSON_TICK_LIMIT_PER_FILE =
	        Integer.valueOf(loadConfig("JSON_TICK_LIMIT_PER_FILE"));
	
	/* Parameters for handling network connections to remote programs */
	public static final boolean DEBUG_NETWORK =
	        Boolean.valueOf(loadConfig("DEBUG_NETWORK"));
	public static final boolean ENABLE_NETWORK =
	        Boolean.valueOf(loadConfig("ENABLE_NETWORK"));
	public static final int NETWORK_BUFFER_RERESH =
	        Integer.valueOf(loadConfig("NETWORK_BUFFER_REFRESH"));
	public static final int NETWORK_STATUS_REFRESH =
	        Integer.valueOf(loadConfig("NETWORK_STATUS_REFRESH"));
	public static final int NETWORK_LISTEN_PORT =
	        Integer.valueOf(loadConfig("NETWORK_LISTEN_PORT"));
	public static final int NETWORK_MAX_MESSAGE_SIZE =
	        Integer.valueOf(loadConfig("NETWORK_MAX_MESSAGE_SIZE"));
	
	/** H Gehlot: Parameter to determine how frequently (in terms of ticks) we should separately 
	 * record the snapshot (for visualization interpolation purposes) of each vehicle in the 
	 * network irrespective of whether it moves or not.
	 * Note that this is independent of the snapshot that is getting collected from the movement 
	 * of vehicles. If a vehicle moves in a tick and it happens that in this tick we had also 
	 * recorded the snapshot for visualization purposes before then finally the snapshot 
	 * corresponding to the movement will have priority and it will override the snapshot written 
	 * for visualization purposes (everything will be overwritten except the coordinated for 
	 * previous epoch when same recording was done).
	 * This variable is also useful for sending the data at this frequency.
	 * */
	public static final int FREQ_RECORD_VEH_SNAPSHOT_FORVIZ = 
			Integer.valueOf(loadConfig("FREQ_RECORD_VEH_SNAPSHOT_FORVIZ"));
	public static final int FREQ_RECORD_ROAD_SNAPSHOT_FORVIZ =
			Integer.valueOf(loadConfig("FREQ_RECORD_ROAD_SNAPSHOT_FORVIZ"));
	public static final int FREQ_RECORD_SHELT_SNAPSHOT_FORVIZ =
			Integer.valueOf(loadConfig("FREQ_RECORD_SHELT_SNAPSHOT_FORVIZ"));
	
	public static double datacollection_start = 0.0;
	public static double datacollection_total = 0.0;	
	
	public static final double BLOCKAGE_SPEED_FOREVENTS = 2.0; //In miles per hour
	
	public static LinkedList<NetworkEventObject> newEventQueue = new
			LinkedList<NetworkEventObject>();//Global queue for storing events
	
	/* 
	 * RV: This variable tracks the no. of generated vehicles (loaded from demand) so far 
	 * This is useful to let the simulator know when to stop
	 * Similarly, also track the number of vehicles having entered the network so far.
	 * */
	public static int NUM_GENERATED_VEHICLES = 0;
	public static int NUM_VEHICLES_ENTERED_ROAD_NETWORK = 0;
	
	/* Gehlot: This variable will keep track of the number of vehicles arrived 
	 * at destination for visualization purposes*/
	public static int NUM_KILLED_VEHICLES = 0;
	
	/* Parameters for handling multiclass routing.
	 * Note that the proportion of original routing vehicles being generated is equal to
	 * 1 - (PROPORTION_OF_PREDEFINED_ROUTING_VEHICLES + PROPORTION_OF_LESS_FREQUENT_ROUTING_VEHICLES). */ 
	public static final boolean ENABLE_MULTICLASS_ROUTING =
	        Boolean.valueOf(loadConfig("ENABLE_MULTICLASS_ROUTING"));
	public static final double PROPORTION_OF_PREDEFINED_ROUTING_VEHICLES =
	        Double.valueOf(loadConfig("PROPORTION_OF_PREDEFINED_ROUTING_VEHICLES"));
	public static final double PROPORTION_OF_LESS_FREQUENT_ROUTING_VEHICLES =
	        Double.valueOf(loadConfig("PROPORTION_OF_LESS_FREQUENT_ROUTING_VEHICLES"));
	public static final double PROBABILITY_OF_UPDATING_ROUTING =
	        Double.valueOf(loadConfig("PROBABILITY_OF_UPDATING_ROUTING"));
	
	/* Parameters for multiclass vehicles (having different parameters).
	TODO: Code a mechanism to generate vehicles with different parameters */
	public static final boolean ENABLE_MULTICLASS_VEHICLES_DIFF_PARAMETERS =
	        Boolean.valueOf(loadConfig("ENABLE_MULTICLASS_VEHICLES_DIFF_PARAMETERS"));
	public static final double RATIO_OF_ORIGINIAL_CLASS =
	        Double.valueOf(loadConfig("RATIO_OF_ORIGINIAL_CLASS"));
	public static final float MAX_ACCELERATION_2 = Float
			.valueOf(loadConfig("MAX_ACCELERATION_2")); // meter/sec2
	public static final float MAX_DECELERATION_2 = Float
			.valueOf(loadConfig("MAX_DECELERATION_2")); // meter/sec2
	
	/* Parameters for adaptive routing */
	public static final double ETA = Double.valueOf(loadConfig("ETA"));
	public static final double TAU  = Double.valueOf(loadConfig("TAU"));
	
	/* LZ,RV: Dynamic destination strategy.
	 * See CityContext.getClosestShelter() for details */
	public static final int DYNAMIC_DEST_STRATEGY = Integer
			.valueOf(loadConfig("DYNAMIC_DEST_STRATEGY"));
//	public static final int SHELTER_CAP = Integer.
//			valueOf(loadConfig("SHELTER_CAP")); // same capacity of each shelter
	public static final String SO_SHELTER_MATCHING_ALGO = 
			String.valueOf(loadConfig("SO_SHELTER_MATCHING_ALGO"));
	public static final int SO_SHELTER_MATCHING_INTERVAL =
			Integer.valueOf(loadConfig("SO_SHELTER_MATCHING_INTERVAL"));
	public static ArrayList<ArrayList<Integer>> shelterRelocateTracker = 
			new ArrayList<ArrayList<Integer>>();	
	public static final Boolean ENABLE_NEW_VEHICLE_MOVEMENT_FUNCTION = 
			Boolean.valueOf(loadConfig("ENABLE_NEW_VEHICLE_MOVEMENT_FUNCTION"));
	/* RV: Record the runtime spent in each 1000 ticks for performance analysis */
	public static final boolean ENABLE_RUNTIME_RECORD =
			Boolean.valueOf(loadConfig("ENABLE_RUNTIME_RECORD"));
	public static final int RUNTIME_RECORD_INTERVAL =
			Integer.valueOf(loadConfig("RUNTIME_RECORDER_INTERVAL"));
	public static ArrayList<Double> RUNTIME_RECORD_LIST = new ArrayList<Double>();
	// LZ
	public static final Boolean DISABLE_GEOMETRY = Boolean.valueOf(loadConfig("DISABLE_GEOMETRY"));
}
