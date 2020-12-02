package evacSim;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileInputStream;
import java.io.FileWriter;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Paths;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.Properties;
//import java.util.concurrent.locks.ReentrantLock;

import javax.measure.unit.NonSI;
import javax.measure.unit.SI;

import org.apache.log4j.Logger;
import org.apache.log4j.PropertyConfigurator;
import org.geotools.referencing.GeodeticCalculator;

import com.vividsolutions.jts.geom.Coordinate;

//import com.vividsolutions.jts.geom.Coordinate;
import repast.simphony.context.Context;
import repast.simphony.dataLoader.ContextBuilder;
import repast.simphony.engine.environment.RunEnvironment;
import repast.simphony.engine.schedule.ISchedule;
import repast.simphony.engine.schedule.ScheduleParameters;
import repast.simphony.space.gis.Geography;
import repast.simphony.space.graph.Network;
import evacSim.GlobalVariables;
import evacSim.citycontext.*;
import evacSim.vehiclecontext.*;
import evacSim.partition.*;
import evacSim.data.*;

public class ContextCreator implements ContextBuilder<Object> {
	
	public static Logger logger; // progress logger
	
	public static BufferedWriter bw; // Vehicle logger

	private static Context<Object> mainContext; // Useful to keep a reference to
												// the main context
	private static int agentID = 0; // Used to generate unique agent ids

	public static double startTime;

	private static int duration_= (int) (3600/GlobalVariables.SIMULATION_STEP_SIZE);
	
	private static int duration02_= GlobalVariables.SIMULATION_NETWORK_REFRESH_INTERVAL;
	
	private static int duration03_= GlobalVariables.SIMULATION_PARTITION_REFRESH_INTERVAL;
	
	// Create the network partitioning object
	public static MetisPartition partitioner = new MetisPartition(GlobalVariables.N_Partition);
	
	// Create the event handler object
	public static NetworkEventHandler eventHandler = new NetworkEventHandler();
	
	// Hemant, Wenbo: Reading background traffic file into treemap
	public static BackgroundTraffic backgroundtraffic = new BackgroundTraffic();
	
	// Create a global lock to enforce concurrency
//	 public static ReentrantLock lock = new ReentrantLock();
	
	public Context<Object> build(Context<Object> context) {
		
		// read the simulation settings
		Properties propertyFile = new Properties();
		try {
			propertyFile.load(new FileInputStream("data/Data.properties"));
		} catch (Exception e) {
			e.printStackTrace();
		}
		
		// RV: get the name of the simulation run scenario to be used in
		// naming the output files and directory & set it as a global variable
		String scenarioName = setScenarioName();
		GlobalVariables.SCENARIO_NAME = scenarioName;
		
		// RV: change the output directory if outputs are to be organized by
		// demand file name
		updateOutputDirectory();

		// RV: set up the log file
		setSimulationLogger();
		
		ContextCreator.mainContext = context;

		while(GlobalVariables.SIMULATION_SLEEPS == 0){
			try {
				Thread.sleep(1000);
				logger.info("Waiting for visualization");
			} catch (InterruptedException e) {
				e.printStackTrace();
			}
		}
		
		/* Build all the subcontexts */
		CityContext cityContext = new CityContext();
		context.addSubContext(cityContext);
		cityContext.createSubContexts();
		
		cityContext.buildRoadNetwork();
		cityContext.createNearestRoadCoordCache();
		
		logger.info("Create other context");
		VehicleContext vehicleContext = new VehicleContext();
		logger.info("Create vehicleContext: "+vehicleContext.getTypeID());
		context.addSubContext(vehicleContext);
		logger.info("Adding vehicleContext");
		
		DataCollectionContext dataContext = new DataCollectionContext();
		context.addSubContext(dataContext);
		
		// debug here
		for (Context<?> con : context.getSubContexts()) {
			logger.info("SubContext typeIDs: " + con.getTypeID());
		}
		// debug ends
		
		// Check if link length and geometry are consistent, fix the inconsistency if there is.
		for(Lane lane: ContextCreator.getLaneGeography().getAllObjects()){
			Coordinate[] coords = ContextCreator.getLaneGeography().getGeometry(lane).getCoordinates();
			double distance = 0;
			for (int i = 0; i < coords.length - 1; i++) {
				distance += getDistance(coords[i], coords[i + 1]);
			}
			lane.setLength(distance);
		}
		
		// Schedule simulation to stop at a certain time and also record the
		// runtime.
		RunEnvironment.getInstance()
				.endAt(GlobalVariables.SIMULATION_STOP_TIME);

		ISchedule schedule = RunEnvironment.getInstance().getCurrentSchedule();
		ScheduleParameters endParams = ScheduleParameters
				.createAtEnd(ScheduleParameters.LAST_PRIORITY);
		schedule.schedule(endParams, this, "end");
		ScheduleParameters startParams = ScheduleParameters.createOneTime(1); //When reaching the end, end the simulation, priority 1
		schedule.schedule(startParams, this, "start");
		// RV: Stop the simulation if all vehicles have been killed 
		if (GlobalVariables.ENABLE_SIMULATION_STOP_IF_NO_VEHICLE) {
			ScheduleParameters endIfNoVehicleParams = ScheduleParameters.createRepeating(
					0, GlobalVariables.SIMULATION_STOP_IF_NO_VEHICLE_CHECK_INTERVAL,
					ScheduleParameters.LAST_PRIORITY);
			schedule.schedule(endIfNoVehicleParams, this, "endIfNoVehicle");
		}
		
		// K: schedule parameter for network reloading
		ScheduleParameters agentParamsNW = ScheduleParameters.createRepeating(
				0, duration02_, 3);
		schedule.schedule(agentParamsNW, cityContext, "modifyRoadNetwork");
		//BL: speedProfileParams is the schedule parameter to update free flow speed for each road every hour
		ScheduleParameters speedProfileParams = ScheduleParameters.createRepeating(0, duration_, 4);
		for (Road r : getRoadContext().getObjects(Road.class)) {
			schedule.schedule(speedProfileParams, r, "updateFreeFlowSpeed");
		}
		
		// LZ: schedule events for loading demand of the next hour
		ScheduleParameters demandLoadingParams = ScheduleParameters.createRepeating(0, duration_, 5);
		schedule.schedule(demandLoadingParams, cityContext, "loadDemandOfNextHour");

		// ZH: schedule the check of the supply side events
		ScheduleParameters supplySideEventParams = ScheduleParameters.createRepeating(0, GlobalVariables.EVENT_CHECK_FREQUENCY, 1);
		schedule.schedule(supplySideEventParams, eventHandler, "checkEvents");
		
		// RV: schedule SO shelter routing
		if (GlobalVariables.DYNAMIC_DEST_STRATEGY == 3) {
			ScheduleParameters soShelterRoutingParams = ScheduleParameters.createRepeating(
					1, GlobalVariables.SO_SHELTER_MATCHING_INTERVAL, 6);
			schedule.schedule(soShelterRoutingParams, cityContext, "soShelterRoutingSchedule");
		}
		
		// Schedule parameters for both serial and parallel road updates
		if (GlobalVariables.MULTI_THREADING){
			ThreadedScheduler s = new ThreadedScheduler(GlobalVariables.N_Partition);
			ScheduleParameters agentParaParams = ScheduleParameters.createRepeating(1, 1, 0);
			schedule.schedule(agentParaParams, s, "paraStep");
			// Schedule shutting down the parallel thread pool
			ScheduleParameters endParallelParams = ScheduleParameters
					.createAtEnd(1);
			schedule.schedule(endParallelParams, s, "shutdownScheduler");
			
			// Schedule the time counter
			ScheduleParameters timerParaParams = ScheduleParameters.createRepeating(1, duration03_, 0);
			schedule.schedule(timerParaParams, s, "reportTime");
		} else {
			ScheduleParameters agentParams = ScheduleParameters.createRepeating(1, 1, 0);
			ThreadedScheduler s = new ThreadedScheduler(GlobalVariables.N_Partition);
			schedule.schedule(agentParams, s, "step");
			double delay = agentParams.getDuration();
			logger.info("TIME BETWEEN TWO TICKS " + delay);
//			schedule.schedule(agentParams, vehicleContext, "refreshAllVehicles");
//			for (Road r : getRoadContext().getObjects(Road.class)) {
//				schedule.schedule(agentParams, r, "step");
//			}
		}
		
//		ScheduleParameters printParams = ScheduleParameters.createRepeating(1,1,0);
//		Road r=getRoadContext().getRandomObject();
		//schedule.schedule(agentParams, r, "printTick");
		
		/* Schedule Parameters for the graph partitioning */
		if (GlobalVariables.MULTI_THREADING){
			ScheduleParameters partitionParams = ScheduleParameters.createRepeating(duration03_, duration03_, 2);
			//Network partitioning, priority 2
			ScheduleParameters initialPartitionParams = ScheduleParameters.createOneTime(0, 2);
			schedule.schedule(initialPartitionParams, partitioner, "first_run");
			schedule.schedule(partitionParams, partitioner, "check_run");
		}
		
		// schedule the data collection framework tasks to mark the start
		// and stop of the model and the start and stop of each sim tick
		// TODO: figure out the double value for the tick duration
		double tickDuration = 1.0d;
				
	    if (GlobalVariables.ENABLE_DATA_COLLECTION) {
	    		//Data collection, priority infinity
			ScheduleParameters dataStartParams = ScheduleParameters.createOneTime(
					0.0, ScheduleParameters.FIRST_PRIORITY);
			schedule.schedule(dataStartParams, dataContext, "startCollecting");
			
			ScheduleParameters dataEndParams = ScheduleParameters.createAtEnd(
					ScheduleParameters.LAST_PRIORITY);
			schedule.schedule(dataEndParams, dataContext, "stopCollecting");
			
			ScheduleParameters tickStartParams = ScheduleParameters.createRepeating(
					0.0d, tickDuration, ScheduleParameters.FIRST_PRIORITY);
			schedule.schedule(tickStartParams, dataContext, "startTick");
			
			ScheduleParameters tickEndParams = ScheduleParameters.createRepeating(
					0.0d, tickDuration, ScheduleParameters.LAST_PRIORITY);
			schedule.schedule(tickEndParams, dataContext, "stopTick");
			
			// RV: schedule the recording of shelter snapshots for the visualization interface
			ScheduleParameters recordShelterSnapshotParams = ScheduleParameters.createRepeating(
					0, GlobalVariables.FREQ_RECORD_SHELT_SNAPSHOT_FORVIZ, 6);
			schedule.schedule(recordShelterSnapshotParams,
					ContextCreator.getZoneContext(), "recordShelterStatus");
			
			// RV: For recording the actual system time spent per few ticks
			ScheduleParameters recordRuntimeParams = ScheduleParameters.createRepeating(
					0, GlobalVariables.RUNTIME_RECORD_INTERVAL, 6);
			schedule.schedule(recordRuntimeParams, dataContext, "recordRuntime");
			
			// LZ: initialize UCBLogger
			createVehicleLogger();
	     }
		
		agentID = 0;
		
		return context;
	}
	
	public void printTick(){
		logger.info("Tick: "
				+ RunEnvironment.getInstance().getCurrentSchedule().getTickCount());
	}
	
	public static void stopSim(Exception ex, Class<?> clazz){
		ISchedule sched = RunEnvironment.getInstance().getCurrentSchedule();
		sched.setFinishing(true);
		sched.executeEndActions();
		logger.info("ContextCreator has been told to stop by  "
				+ clazz.getName()+ ex);
	}
	
	public static void end() {
		logger.info("Finished sim: "
				+ (System.currentTimeMillis() - startTime));
		try {
			bw.close();
		} catch (IOException e){
			e.printStackTrace();
		}
	}

	/**
	 * RV: Prematurely end the simulation if there is no vehicle on the road
	 * network or in the queue of loaded vehicles. This is achieved with a
	 * proxy and is not fool proof. If the no. of vehicles generated so far is
	 * the same as (or fewer in the pathological case) the no. of vehicles
	 * destroyed so far (after they reach the destination). This excludes the
	 * case in the beginning when both of these terms are zero. This may not
	 * work in the special case when there is a gap in the demand file so large
	 * that all vehicles reach their destination in that time and there is no
	 * vehicle on the network.
	 */
	public static void endIfNoVehicle() {
		int nGenerated = GlobalVariables.NUM_GENERATED_VEHICLES;
		int nDestroyed = GlobalVariables.NUM_KILLED_VEHICLES;
		int nHouses = GlobalVariables.NUM_HOUSES;
		if ((nGenerated >= nHouses-1) & (nGenerated <= nDestroyed)) {
			ISchedule sched = RunEnvironment.getInstance().getCurrentSchedule();
			sched.setFinishing(true);
			sched.executeEndActions();
		}
	}
	
	public static void start() {
		startTime = System.currentTimeMillis();
	}
	
	/**
	 * Creates a unique identifier for an agent. IDs are created in ascending
	 * order.
	 * @return the unique identifier.
	 */
	public static int generateAgentID() {
		return ContextCreator.agentID++;
	}

	public static VehicleContext getVehicleContext() {
		return (VehicleContext) mainContext.findContext("VehicleContext");
	}

	public static Geography<Vehicle> getVehicleGeography() {
		return (Geography<Vehicle>) ContextCreator.getVehicleContext()
				.getProjection("VehicleGeography");
	}

	public static JunctionContext getJunctionContext() {
		return (JunctionContext) mainContext.findContext("JunctionContext");
	}

	public static Geography<Junction> getJunctionGeography() {
		return ContextCreator.getJunctionContext().getProjection(
				Geography.class, "JunctionGeography");
	}

	public static Network<Junction> getRoadNetwork() {
		return ContextCreator.getJunctionContext().getProjection(Network.class,
				"RoadNetwork");
	}

	public static RoadContext getRoadContext() {
		return (RoadContext) mainContext.findContext("RoadContext");
	}

	@SuppressWarnings("unchecked")
	public static Geography<Road> getRoadGeography() {
		return (Geography<Road>) ContextCreator.getRoadContext().getProjection(
				"RoadGeography");
	}

	public static LaneContext getLaneContext() {
		return (LaneContext) mainContext.findContext("LaneContext");
	}

	public static Geography<Lane> getLaneGeography() {
		return (Geography<Lane>) ContextCreator.getLaneContext().getProjection(
				"LaneGeography");
	}

	public static CityContext getCityContext() {
		return (CityContext) mainContext.findContext("CityContext");
	}

	public static ZoneContext getZoneContext() {
		return (ZoneContext) mainContext.findContext("ZoneContext");
	}

	public static Geography<Zone> getZoneGeography() {
		return (Geography<Zone>) ContextCreator.getZoneContext().getProjection(
				"ZoneGeography");
	}
	
	public static DataCollectionContext getDataCollectionContext() {
	    return (DataCollectionContext) mainContext.findContext("DataCollectionContext");
	}
	
	public static double convertToMeters(double dist) {

		double distInMeters = NonSI.NAUTICAL_MILE.getConverterTo(SI.METER)
				.convert(dist * 60);
		return distInMeters;
	}
	
	private double getDistance(Coordinate c1, Coordinate c2) {
		// GeodeticCalculator calculator = new GeodeticCalculator(ContextCreator
		// .getRoadGeography().getCRS());
		GeodeticCalculator calculator = new GeodeticCalculator(ContextCreator
				.getLaneGeography().getCRS());
		calculator.setStartingGeographicPoint(c1.x, c1.y);
		calculator.setDestinationGeographicPoint(c2.x, c2.y);
		double distance;
		try {
			distance = calculator.getOrthodromicDistance();
		} catch (AssertionError ex) {
			logger.error("Error with finding distance");
			distance = 0.0;
		}
		return distance;
	}
	
	/**
	 * LZ,RV: Create a logger for vehicle attributes during data collection
	 * as part of the analysis for shelter routing and to prevent reading the
	 * massive JSON trajectory files.
	 */
	public static void createVehicleLogger() {
		/*
		 * RV: get the output directory - either the default one or
		 * in case of running multiple scenarios, organize all files into
		 * a folder for each scenario based on its demand filename
		 */
		// get the default output directory
		String outDir = GlobalVariables.OUTPUT_DIR;
		
		// get the basename of the demand file -
		String basename = "";
		if (GlobalVariables.ORGANIZE_OUTPUT_BY_ACTIVITY_FNAME) {
			String[] temp = GlobalVariables.OUTPUT_DIR.split("/");
			basename = temp[temp.length - 1];
		}
		
		// get the current timestamp
		String timestamp = new SimpleDateFormat("YYYY-MM-dd-hh-mm-ss")
				.format(new Date());
		// create the overall file path, named after the demand filename
		String outpath = outDir + File.separatorChar + "logger-vehicles-" + 
				basename + "-" + timestamp + ".csv";
		// check the path will be a valid file
		try {
			FileWriter fw = new FileWriter(outpath, false);
			bw = new BufferedWriter(fw);
			bw.write("vehicleID,zoneType,startTime,endTime,originID,destID," + 
					"totalDistance,visitedShelters");
			bw.newLine();
			bw.flush();
			logger.info("Vehicle logger created");
		} catch (IOException e) {
			e.printStackTrace();
			logger.error("Vehicle logger failed");
		}
	}
	
	/**
	 * RV: Set the name of a simulation run (scenario) based on its
	 * demand file.
	 * @throws Exception when the processed name is null or empty
	 */
	public static String setScenarioName() {
		// get the base name of the demand file
		String[] temp1 = GlobalVariables.ACTIVITY_CSV.split("/");
        String temp2 = temp1[temp1.length - 1];
        String scenarioName = temp2.substring(0, temp2.lastIndexOf('.'));
        	// check for error
        	if (scenarioName == null || scenarioName.length() <= 0) {
        		// but do not break the code, but just print the error
        		System.err.println("Invalid scenario name");
        	}
		return scenarioName;
	}
	
	/**
	 * RV: Modify the default simulation output directory to be a subdirectory
	 * of the default output directory, named by its scenario name.
	 * @return outDir name of the (possibly updated) output directory
	 */
	public static String updateOutputDirectory() {
		String outDir;
		if (GlobalVariables.ORGANIZE_OUTPUT_BY_ACTIVITY_FNAME) {
			outDir = GlobalVariables.DEFAULT_OUTPUT_DIR;
	        	// set this base name as a subdirectory
	        	// do not use File.separatorChar here since it creates problems afterwards
	        	outDir = outDir + "/" + GlobalVariables.SCENARIO_NAME;
	        	// if the directory does not exist, create it
			try {
				Files.createDirectories(Paths.get(outDir));
			} catch (IOException e) {
				e.printStackTrace();
			}
			// set it as the default output directory
			GlobalVariables.OUTPUT_DIR = outDir;
		}
		else {
			outDir = GlobalVariables.DEFAULT_OUTPUT_DIR;
		}
		return outDir;
	}
	
	/**
	 * RV: Specify the properties of and set up a global simulation logger.
	 * This uses another properties file originally called "sim_logger.properties". 
	 */
	public void setSimulationLogger() {
        	// prepare the path of the log file
		String logFilePath = GlobalVariables.OUTPUT_DIR +
				File.separatorChar + "logger-sim-" +
				GlobalVariables.SCENARIO_NAME + "-" +
				new SimpleDateFormat("YYYY-MM-dd-hh-mm-ss").format(new Date()) +
				".log";
		System.setProperty("logFilePath", logFilePath);
		// create a logger configured in GlobalVariables.LOGGER_PROPERTIES
		logger = Logger.getLogger(this.getClass());
		PropertyConfigurator.configure(GlobalVariables.LOGGER_PROPERTIES);
		
		logger.info("Logging started");
	}
}