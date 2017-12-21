package evacSim;

import java.io.FileInputStream;
import java.util.Properties;
import java.util.concurrent.locks.ReentrantLock;

import javax.measure.unit.NonSI;
import javax.measure.unit.SI;

import com.vividsolutions.jts.geom.Coordinate;

import repast.simphony.context.Context;
import repast.simphony.dataLoader.ContextBuilder;
import repast.simphony.engine.environment.ControllerAction;
import repast.simphony.engine.environment.RunEnvironment;
import repast.simphony.engine.schedule.ISchedule;
import repast.simphony.engine.schedule.ScheduleParameters;
import repast.simphony.essentials.RepastEssentials;
import repast.simphony.space.gis.Geography;
import repast.simphony.space.graph.Network;
import repast.simphony.util.collections.Tree;
import repast.simphony.visualization.gis.DisplayGIS;
import evacSim.GlobalVariables;
import evacSim.citycontext.*;
import evacSim.routing.*;
import evacSim.vehiclecontext.*;
import evacSim.partition.*;


public class ContextCreator implements ContextBuilder<Object> {

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
	
	//Hemant, Wenbo: Reading background traffic file into treemap
	public static BackgroundTraffic backgroundtraffic = new BackgroundTraffic();
	
//	// Create a global lock to enforce concurrency
//	public static ReentrantLock lock = new ReentrantLock();
	
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
			propertyFile.load(new FileInputStream("config/Data.properties"));
		} catch (Exception e) {
			e.printStackTrace();
		}

		ContextCreator.mainContext = context;

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
		
		Geography<Vehicle> vehicleGeography = ContextCreator.getVehicleGeography();


		// Load the initial vehicles into the queue of each road
		for (Vehicle v : getVehicleContext().getObjects(Vehicle.class)) {
			Coordinate currentCoord = vehicleGeography.getGeometry(v)
					.getCoordinate();
			Road road = cityContext.findRoadAtCoordinates(currentCoord, false); 
			road.addVehicleToNewQueue(v);

		}
		

//		// Seting display updating interval
//		DisplayGIS.GIS_FRAME_UPDATE_INTERVAL=1000;
		
		// debug here
		for (Context con : context.getSubContexts()) {
			System.out.println("SubContext typeIDs: "+con.getTypeID());
		}
		// debug ends
		
		
		// Schedule simulation to stop at a certain time and also record the
		// runtime.
		RunEnvironment.getInstance()
				.endAt(GlobalVariables.SIMULATION_STOP_TIME);

		ISchedule schedule = RunEnvironment.getInstance().getCurrentSchedule();
		ScheduleParameters endParams = ScheduleParameters
				.createAtEnd(ScheduleParameters.LAST_PRIORITY);
		schedule.schedule(endParams, this, "end");
		ScheduleParameters startParams = ScheduleParameters.createOneTime(1);
		schedule.schedule(startParams, this, "start");
		
		// K: schedule parameter for network reloading
		ScheduleParameters agentParamsNW = ScheduleParameters.createRepeating(
				0, duration02_, 3);
		
		//BL: speedProfileParams is the schedule parameter to update free flow speed for each road every hour
		ScheduleParameters speedProfileParams = ScheduleParameters.createRepeating(0, duration_, 4);
		
		schedule.schedule(agentParamsNW, cityContext, "modifyRoadNetwork");

		for (Road r : getRoadContext().getObjects(Road.class)) {
			schedule.schedule(speedProfileParams, r, "updateFreeFlowSpeed");
		}
		
		// ZH: schedule the check of the supply side events
		ScheduleParameters supplySideEventParams = ScheduleParameters.createRepeating(0, GlobalVariables.EVENT_CHECK_FREQUENCY, 1);
		schedule.schedule(supplySideEventParams, eventHandler, "checkEvents");
		
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
			double delay = agentParams.getDuration();
			System.out.println("TIME BETWEEN TWO TICKS " + delay);
			
			for (Road r : getRoadContext().getObjects(Road.class)) {
				schedule.schedule(agentParams, r, "step");
			}
		}
		
//		ScheduleParameters printParams = ScheduleParameters.createRepeating(1,1,0);
//		Road r=getRoadContext().getRandomObject();
		//schedule.schedule(agentParams, r, "printTick");
		
		/* Schedule Parameters for the graph partitioning */
		if (GlobalVariables.MULTI_THREADING){
			ScheduleParameters partitionParams = ScheduleParameters.createRepeating(duration03_, duration03_, 2);
			ScheduleParameters initialPartitionParams = ScheduleParameters.createOneTime(0, 2);
			schedule.schedule(initialPartitionParams, partitioner, "first_run");
			schedule.schedule(partitionParams, partitioner, "check_run");
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
	
	public static double convertToMeters(double dist) {
		double distInMeters = NonSI.NAUTICAL_MILE.getConverterTo(SI.METER)
				.convert(dist * 60);
		return distInMeters;
	}
}