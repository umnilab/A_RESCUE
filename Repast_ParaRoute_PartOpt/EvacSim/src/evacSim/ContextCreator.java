package evacSim;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileInputStream;
import java.io.FileWriter;
import java.io.IOException;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.Properties;
//import java.util.concurrent.locks.ReentrantLock;

import javax.measure.unit.NonSI;
import javax.measure.unit.SI;

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
			System.err.println("Error with finding distance");
			distance = 0.0;
		}
		return distance;
	}
	/**
	 * The citycontext will create its own subcontexts (RoadContext,
	 * JunctionContext and HouseContext).
	 */
	/*
	 * (non-Javadoc)
	 * 
	 * @see
	 * repast.simphony.dataLoader.ContextBuilder#build(repast.simphony.context
	 * .Context)
	 */
	public Context<Object> build(Context<Object> context) {
		Properties propertyFile = new Properties();

		try {
//			propertyFile.load(new FileInputStream("config/Data.properties"));
			propertyFile.load(new FileInputStream("data/Data.properties"));
		} catch (Exception e) {
			e.printStackTrace();
		}

		ContextCreator.mainContext = context;

		while(GlobalVariables.SIMULATION_SLEEPS == 0){
			try {
				Thread.sleep(1000);
				System.out.println("Waiting for visualization");
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
		
		System.out.println("Create other context");
		VehicleContext vehicleContext = new VehicleContext();
		System.out.println("Create vehicleContext: "+vehicleContext.getTypeID());
		context.addSubContext(vehicleContext);
		System.out.println("Adding vehicleContext");
		
//		Geography<Vehicle> vehicleGeography = ContextCreator.getVehicleGeography();


		// Load the initial vehicles into the queue of each road
//		for (Vehicle v : getVehicleContext().getObjects(Vehicle.class)) {
////			Coordinate currentCoord = vehicleGeography.getGeometry(v)
////					.getCoordinate();
////			v.setCurrentCoord(currentCoord);
//			Road road = cityContext.findRoadAtCoordinates(v.getCurrentCoord(), false); 
//			if(road.getLinkid() == 104819){
//				road =  ContextCreator.getCityContext().findRoadWithLinkID(104818);
//			}
//			if(road.getLinkid() == 101235){
//				road =  ContextCreator.getCityContext().findRoadWithLinkID(101236);
//			}
//			road.addVehicleToNewQueue(v);
//		}
		
		DataCollectionContext dataContext = new DataCollectionContext();
		context.addSubContext(dataContext);
		

//		// Seting display updating interval
//		DisplayGIS.GIS_FRAME_UPDATE_INTERVAL=1000;
		
		// debug here
		for (Context<?> con : context.getSubContexts()) {
			System.out.println("SubContext typeIDs: " + con.getTypeID());
		}
		// debug ends
		
		// Check if link length and geometry are consistent, fix the inconsistency if there is.
		for(Lane lane: ContextCreator.getLaneGeography().getAllObjects()){
			Coordinate[] coords = ContextCreator.getLaneGeography().getGeometry(lane).getCoordinates();
			double distance = 0;
			for (int i = 0; i < coords.length - 1; i++) {
				distance += getDistance(coords[i], coords[i + 1]);
			}
//			if(Math.abs(distance-lane.getLength())>1){
//				System.out.println("Lane ID: " + lane.getLaneid() + "," + " Calculated distance: "+ distance+","+"Real distance: " + lane.getLength());
//			}
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
			System.out.println("TIME BETWEEN TWO TICKS " + delay);
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
				
	    if(GlobalVariables.ENABLE_DATA_COLLECTION){
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
		System.out.println("Tick: "
				+ RunEnvironment.getInstance().getCurrentSchedule().getTickCount());
	}
	
	public static void stopSim(Exception ex, Class<?> clazz){
		ISchedule sched = RunEnvironment.getInstance().getCurrentSchedule();
		sched.setFinishing(true);
		sched.executeEndActions();
		System.out.println("ContextCreator has been told to stop by  "
				+ clazz.getName()+ ex);
	}
	
	public static void end() {
		System.out.println("Finished sim: "
				+ (System.currentTimeMillis() - startTime));
//		System.out.println("Finished data collection: "
//				+ (GlobalVariables.datacollection_total));
		try{
			bw.close();
		} catch (IOException e){
			e.printStackTrace();
		}
	}

	/** RV: Prematurely end the simulation if there is no vehicle on the road network or
	 *  in the queue of loaded vehicles. This is achieved with a proxy and is not fool proof.
	 *  IF the no. of vehicles generated so far is the same as (or fewer in the pathological
	 *  case) the no. of vehicles destroyed so far (after they reach the destination).
	 *  This excludes the case in the beginning when both of these terms are zero.
	 *  This may not work in the special case when there
	 *  is a gap in the demand file so large that all vehicles reach their destination in that
	 *  time and there is no vehicle on the network.
	 *  */
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
	 * 
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
	
	/**
	 * LZ,RV: Create a logger for vehicle attributes during data collection
	 * as part of the analysis for shelter routing and to prevent reading the
	 * massive JSON trajectory files.
	 */
	public static void createVehicleLogger() {
		System.out.println("Vehicle logger creating...");
		// get the output directory - the same as the JSON output
		String dir = GlobalVariables.JSON_DEFAULT_PATH;
		if (dir == null || dir.trim().length() < 1) {
			dir = System.getProperty("user.dir");
		}
		// get the filename - same as the JSON output
		String basename = GlobalVariables.JSON_DEFAULT_FILENAME;
		// get the current timestamp
		String timestamp = new SimpleDateFormat("YYYY-MM-dd-hh-mm-ss")
				.format(new Date());
		// create the overall file path
		String outpath = dir + File.separatorChar + "vehicle-logger-" + 
				basename + "-" + timestamp + ".csv";
		// check the path will be a valid file
		try {
			FileWriter fw = new FileWriter(outpath, false);
			bw = new BufferedWriter(fw);
			bw.write("vehicleID,type,startTime,endTime,originID,destID," + 
					"totalDistance,visitedShelters");
			bw.newLine();
			bw.flush();
			System.out.println("Vehicle logger created!");
		} catch (IOException e) {
			e.printStackTrace();
			System.err.println("Vehicle logger failed.");
		}
	}
}