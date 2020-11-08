package evacSim.vehiclecontext;

import com.vividsolutions.jts.geom.Coordinate;
import com.vividsolutions.jts.geom.Geometry;
import com.vividsolutions.jts.geom.GeometryFactory;
import com.vividsolutions.jts.operation.distance.DistanceOp;

//import cern.colt.Arrays;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.Iterator;
import java.util.List;
import java.util.Map;
import java.util.Map.Entry;
import java.util.Queue;
//import java.util.concurrent.locks.ReentrantLock;

import org.opengis.referencing.operation.MathTransformFactory;
//import org.apache.commons.lang3.ArrayUtils;
import org.geotools.referencing.GeodeticCalculator;
import org.geotools.referencing.ReferencingFactoryFinder;
import java.awt.geom.Point2D;
import java.lang.Math;
//import java.io.IOException;
//import java.util.concurrent.locks.ReentrantLock;
 
import repast.simphony.engine.environment.RunEnvironment;
import repast.simphony.essentials.RepastEssentials;
import repast.simphony.space.gis.Geography;
import evacSim.ContextCreator;
import evacSim.GlobalVariables;
import evacSim.citycontext.CityContext;
import evacSim.citycontext.House;
import evacSim.citycontext.Junction;
import evacSim.citycontext.Lane;
import evacSim.citycontext.Plan;
import evacSim.citycontext.Road;
import evacSim.citycontext.Zone;
import evacSim.data.DataCollector;
import evacSim.routing.RouteV;
//import evacSim.citycontext.ZoneContext;
//import evacSim.routing.SOShelterRouting;

public class Vehicle {
	private int id;
	protected int vehicleID_;
	// private int startTime; //duplicated
	private int deptime;
	private int endTime;
	private int destinationZoneId;
	// private int evactime;
	protected int destRoadID;
	protected int lastRouteTime; // The time of getting the last routing information

	private Coordinate originalCoord;
	protected Coordinate destCoord;
	private Coordinate previousEpochCoord;//HGehlot: this variable stores the coordinates of the vehicle 
	                                      //when last time vehicle snapshot was recorded for visualization interpolation 
	private Coordinate currentCoord_; //LZ: this variable is created when the vehicle is initialized and used to store 
	                                 //vehicle location when it does not enter the network
	private float length;
	private double distance_;// distance from downstream junction
	private float currentSpeed_;
	private double accRate_;
	private float desiredSpeed_; // in meter/sec
	private  int regime_;
	protected float maxAcceleration_; // in meter/sec2
	private float normalDeceleration_; // in meter/sec2
	protected float maxDeceleration_; // in meter/sec2
	private float distanceToNormalStop_; // assuming normal dec is applied
	private double lastStepMove_;
	public double accummulatedDistance_;

	private double travelPerTurn;

	private boolean reachDest;
	private boolean reachActLocation;
//	private boolean moveVehicle = true;// LZ: Vehicle is moved or not, initial value is true to let vehicle enter the network
	private int lastMoveTick = -1; // LZ: A better solution to avoid double modification in single tick
	// the next step
	private boolean onlane = false;
	protected boolean atOrigin = true;

	// Need to use a better way in the future: currently use the house object to
	// load information to vehicle
	private House house;

	protected Road road;
	protected Road nextRoad_;
	private Lane lane;
	private Lane nextLane_;
	protected Zone destZone; // RMSA
	
	/* Zhan: For vehicle based routing */
	/* ZL: Update this from list to queue */
	protected Queue<Road> roadPath; // The route is always started with the current road, whenever entering the next road, the current road will be popped out

	private List<Coordinate> coordMap;

	private Geography<Lane> laneGeography;
//	private Geography<Vehicle> vehicleGeography;

	private Vehicle leading_; // leading vehicle in the lane
	private Vehicle trailing_; // trailing vehicle in the lane
	private Vehicle macroLeading_; // BL: leading vehicle on the road (all lane
	// combined)
	private Vehicle macroTrailing_; // BL: trailing vehicle on the road (all lane
	// combined)

	// BL: Variables for lane changing model
	private Lane targetLane_; // BL: this is the correct lane that vehicle
	// should change to.
//	private Lane tempLane_;// BL: this is the adjection lane toward the target
	// lane direction
	private boolean correctLane; // BL: to check if the vehicle is in the
	// correct lane
	private boolean nosingFlag;// BL: if a vehicle in MLC and it can't find gap
	// acceptance then nosing is true.
	private boolean yieldingFlag; // BL: the vehicle need to yield if true

	GeodeticCalculator calculator = new GeodeticCalculator(ContextCreator
			.getLaneGeography().getCRS());
	MathTransformFactory mtFactory = ReferencingFactoryFinder.getMathTransformFactory(null);
	
	// For adaptive network partitioning
	private int Nshadow; // Number of current shadow roads in the path
	private ArrayList<Road> futureRoutingRoad; // Can be slow
	
	// HG:For distinguishing between different classes
	private int vehicleClass;
	// Xue: The travel time stored for the last time the route was updated.
	protected double travelTimeForPreviousRoute;
	// Xue: the tick when the last routing was updated.
	protected int previousTick;
	// Rajat, Xue: the relative difference threshold of route time for the old route and new route.
	protected double indiffBand; 
	// LZ,RV:DynaDestTest: List of visited shelters along with the time of visit
	protected HashMap<Integer, Integer> visitedShelters;
	
	// Create a lock variable, this is to enforce concurrency within vehicle update computation
//	private ReentrantLock lock;
	// LZ, number of times the vehicle get stuck
	protected int stuck_time = 0;

	public Vehicle(House h) {
		this.id = ContextCreator.generateAgentID();
		this.house = h;
		this.currentCoord_ = new Coordinate();
//		System.out.println("Veh" + this.id + " from " + p.get(0).getLocation() +
//				" to " + p.get(1).getLocation() + " at t=" + p.get(0).getDuration() * 12000);

		this.length = GlobalVariables.DEFAULT_VEHICLE_LENGTH;
		this.travelPerTurn = GlobalVariables.TRAVEL_PER_TURN;
		this.maxAcceleration_ = GlobalVariables.MAX_ACCELERATION;
		this.maxDeceleration_ = GlobalVariables.MAX_DECELERATION;
		this.normalDeceleration_ = -0.5f;

		this.previousEpochCoord = new Coordinate();
		this.endTime = 0;
		// this.atOrigin = true;
		this.reachDest = false;
		this.reachActLocation = true;
		this.accRate_ = 0;
		this.nextLane_ = null;
		this.nosingFlag = false;
		this.yieldingFlag = false;
		this.macroLeading_ = null;
		this.macroTrailing_ = null;
		this.leading_ = null;
		this.trailing_ = null;
		this.nextRoad_ = null;
		this.laneGeography = ContextCreator.getLaneGeography();
		this.coordMap = new ArrayList<Coordinate>();
		this.destRoadID = 0;
		// upload the vehicle into the queue of the corresponding link
		this.lastStepMove_ = 0;
		this.vehicleID_ = h.getId();
		this.accummulatedDistance_ = 0;
		this.roadPath = null;
		this.lastRouteTime = -1;
//		this.lock = ContextCreator.lock;
		this.setNextPlan();
		
		// For adaptive network partitioning
		this.Nshadow = 0;
		this.futureRoutingRoad = new ArrayList<Road>();
		this.setVehicleClass(1);
		// Xue: For the first step, set the travel time for the previous route to be infinite.
		this.travelTimeForPreviousRoute = Double.MAX_VALUE;
		// Xue: The previous tick is set to be 0 at the beginning.
		this.previousTick = 0;
		
		// Xue: Generate the parameter of route selection from a distribution.
		this.indiffBand = assignIndiffBand();
		// RV:DynaDestTest: List of visited shelters along with the time of visit
		this.visitedShelters = new HashMap<Integer, Integer>();
		Plan startPlan = house.getActivityPlan().get(0);
		this.visitedShelters.put(startPlan.getLocation(), startPlan.getDuration());
		// RV
		GlobalVariables.NUM_GENERATED_VEHICLES++;
	}

	/* HG: This is a new subclass of Vehicle class that has some different 
	 * parameters like max acceleration and max deceleration */ 
	public Vehicle(House h, float maximumAcceleration, float maximumDeceleration) {
		this.id = ContextCreator.generateAgentID();
		this.house = h;
		this.currentCoord_ = new Coordinate();

		this.length = GlobalVariables.DEFAULT_VEHICLE_LENGTH;
		this.travelPerTurn = GlobalVariables.TRAVEL_PER_TURN;
		this.maxAcceleration_ = maximumAcceleration;
		this.maxDeceleration_ = maximumDeceleration;
		this.normalDeceleration_ = -0.5f;

		this.previousEpochCoord = new Coordinate();
		this.endTime = 0;
		// this.atOrigin = true;
		this.reachDest = false;
		this.reachActLocation = true;
		this.accRate_ = 0;
		this.nextLane_ = null;
		this.nosingFlag = false;
		this.yieldingFlag = false;
		this.macroLeading_ = null;
		this.macroTrailing_ = null;
		this.leading_ = null;
		this.trailing_ = null;
		this.nextRoad_ = null;
		this.laneGeography = ContextCreator.getLaneGeography();
		this.coordMap = new ArrayList<Coordinate>();
		this.destRoadID = 0;
		// upload the vehicle into the queue of the corresponding link
		this.lastStepMove_ = 0;
		this.vehicleID_ = h.getId();
		this.accummulatedDistance_ = 0;
		this.roadPath = null;
		this.lastRouteTime = -1;
//		this.lock = ContextCreator.lock;
		this.setNextPlan();
		
		// For adaptive network partitioning
		this.Nshadow = 0;
		this.futureRoutingRoad = new ArrayList<Road>();
		this.setVehicleClass(-1); // TODO HG: Change it later when use it
		// Xue: For the first step, set the travel time for the previous route to be infinite.
		this.travelTimeForPreviousRoute = Double.MAX_VALUE;
		// Xue: The previous tick is set to be 0 at the beginning.
		this.previousTick = 0;
		// Xue: generate the parameter of route selection from a distribution.
		this.indiffBand = assignIndiffBand();
		// LZ,RV: record the shelters visited by this
		this.visitedShelters = new HashMap<Integer, Integer>();
		Plan startPlan = house.getActivityPlan().get(0);
		this.visitedShelters.put(startPlan.getLocation(), startPlan.getDuration());
		// RV
		GlobalVariables.NUM_GENERATED_VEHICLES++;
	}
	 
	public void setNextPlan() {
		Plan current = house.getActivityPlan().get(0);
		Plan next = house.getActivityPlan().get(1);
		int destinationZone = next.getLocation();
		this.destinationZoneId = destinationZone;
		float duration = current.getDuration();
		int deptime = (int) ((duration * 60)
				/ GlobalVariables.SIMULATION_STEP_SIZE);
		this.setDepTime(deptime);
		CityContext cityContext = (CityContext) ContextCreator.getCityContext();
		this.destZone = cityContext.findHouseWithDestID(destinationZoneId);
		if (destZone == null) {
			System.out.println("helloooo");
		}
		this.destCoord = this.destZone.getCoord();
		this.originalCoord = cityContext.findHouseWithDestID(
				current.getLocation()).getCoord();
		this.destRoadID = cityContext.findRoadAtCoordinates(this.destCoord,
				true).getLinkid();
		this.atOrigin = true;
//		this.house.removePlan(current);
	}

	/**
	 * SH: This function enters the vehicles into the network
	 */

	public int enterNetwork(Road road) {
		// BL: Temporarily comment
		// out to test the route when vehicle re-enter network from activity
		// location
		Lane firstlane = road.firstLane();
		double gap = entranceGap(firstlane);
		int tickcount = (int) RepastEssentials.GetTickCount(); 
		if(gap>=this.length() && tickcount>firstlane.getLastEnterTick()){
			firstlane.updateLastEnterTick(tickcount); //LZ: Update the last enter tick for this lane
			this.updateLastMoveTick(tickcount);
//			this.distance_ = (float) firstlane.length();
			// For debug, if this is null, show it. Result, this cannot be null
//			System.out.println(this.distance_);
//			// Add vehicle to vehicle context
//			ContextCreator.getVehicleContext().add(this);
//		    System.out.println("A vehicle is entering the road.");
			float capspd = road.getFreeSpeed();// calculate the initial speed, to be consistent with changeRoad, use free speed
			currentSpeed_ = capspd; // have to change later
			desiredSpeed_ = this.road.getFreeSpeed(); //Initial value is 0.0, added Oct 2ed
			this.road.removeVehicleFromNewQueue(this);
			this.setRoad(road);
			this.setCoordMap(firstlane);
//			if(this.distance_<=0){
//				System.out.println("Here 2");
//			}
			this.append(firstlane);
//			while(this.road.isLocked()); // LZ: Also need a lock here
//			this.road.setLock();
			this.appendToRoad(this.road);
//			this.road.releaseLock();
			this.setNextRoad();
			this.assignNextLane();
//			if(firstlane.getLength()<GlobalVariables.NO_LANECHANGING_LENGTH){
//				this.distance_ = 0;
//				this.setCurrentCoord(this.coordMap.get(this.coordMap.size()-1));
//			}
			GlobalVariables.NUM_VEHICLES_ENTERED_ROAD_NETWORK++;
			return 1;
		}
		return 0;
	}

	public Road nextRoad() {
		return this.nextRoad_;
	}
	
	/* Clear the legacy impact from the shadow vehicles and future routing vehicles.
	 * Performed before next routing computation. */
	public void clearShadowImpact() {
		if (this.roadPath != null) {
			if (this.Nshadow > this.roadPath.size())
				this.Nshadow = this.roadPath.size();
			if (this.Nshadow > 0) {
				Iterator<Road> itr = this.roadPath.iterator();
				for (int i = 0; i < this.Nshadow; i++) {
					Road r = itr.next();
					r.decreaseShadowVehicleNum();
				}
			}
			this.Nshadow = 0;
			// Clear future routing road impact
			for (Road r : this.futureRoutingRoad) {
				r.decreaseFutureRoutingVehNum();
			}
			this.futureRoutingRoad.clear();
		}
	}
	
	/* Remove shadow vehicle count after the vehicle leaves the road */
	public void removeShadowCount(Road r) {
		if (this.Nshadow > 0) {
			r.decreaseShadowVehicleNum();
			this.Nshadow--;
		}
		
		// Remove the future routing road impact
		if (this.futureRoutingRoad.contains(r)){
			r.decreaseFutureRoutingVehNum();
		    this.futureRoutingRoad.remove(r);
		}
	}
	
	/* Set shadow vehicles and future routing road */
	public void setShadowImpact() {
		this.Nshadow = GlobalVariables.N_SHADOW;
		if (this.roadPath.size() < this.Nshadow)
			this.Nshadow = this.roadPath.size();
		if (this.Nshadow > 0) {
			int shadowCount = 1; // Count actual number of Nshadow vehicles added
			double cumlativeTT_Nshadow = 0.0; // Cumulative TT for Nshadow allocation
			double cumulativeTT = 0.0;
			int foundFutureRoutingRoad = 0; // Future routing road count: number of road found in shadow roads
			Iterator<Road> itr = this.roadPath.iterator();
			for (int i=0; i < this.Nshadow; i++) {
				Road r = itr.next();
				// Increase the shadow vehicle count: include current road
				if (i < 1) {
					// Current vehicle will always be added by default
					// Set the shadow vehicle count
					r.incrementShadowVehicleNum();
				} else {
					if (cumlativeTT_Nshadow <= GlobalVariables.SIMULATION_PARTITION_REFRESH_INTERVAL * GlobalVariables.SIMULATION_STEP_SIZE) {
						// Set the shadow vehicle count
						r.incrementShadowVehicleNum();
						cumlativeTT_Nshadow += r.getTravelTime();
						shadowCount += 1;
					}
				}
				
				cumulativeTT += r.getTravelTime();
				// Found the road with cumulative TT greater than than network refresh interval, use it as the future routing road
				if (foundFutureRoutingRoad < GlobalVariables.PART_REFRESH_MULTIPLIER) {
					if (cumulativeTT >= GlobalVariables.SIMULATION_NETWORK_REFRESH_INTERVAL * GlobalVariables.SIMULATION_STEP_SIZE){
						this.futureRoutingRoad.add(r);
						r.incrementFutureRoutingVehNum();
						// Update the future routing road count
						foundFutureRoutingRoad += 1;
						// Reset the cumulative TT
						cumulativeTT = 0.0;
					}
				}
			}
			
			// Reset the Nshadow count
			this.Nshadow = shadowCount;
			
		} else {
			this.Nshadow = 0;
		}
		
	}
	
	public void setNextRoad() {
		try {
			if (!this.atOrigin) { // Not at origin
				if (this.road.getLinkid() == this.destRoadID) { // Arrive the destination link //LZ: 20200607 Test, two variables lead to inconsistency
					this.nextRoad_ = null;
					return;
				}
				
				boolean flag = false; //LZ: successfully reroute 
				if (this.lastRouteTime < RouteV.getValidTime()) { // Path does not valid
					// The information are outdated, needs to be recomputed
					// Check if the current lane connects to the next road in the new path
					//Xue, Oct 2019: change the return type of RouteV.vehicleRoute to be a HashMap, and get the tempPathNew and pathTimeNew.
                                        Map<Double, Queue<Road>> tempPathMap = RouteV.vehicleRoute(this, this.destZone);   //Xue, Oct 2019: change the return type of RouteV.vehicleRoute to be a HashMap, and get the tempPathNew and pathTimeNew.
					for (Entry<Double, Queue<Road>> entry : tempPathMap.entrySet()) { // Only one element
						double pathTimeNew = entry.getKey();           // Calculate path time
						Queue<Road> tempPathNew = entry.getValue();    // Calculate path
						if (pathTimeNew == 0.0) {
							break;
						}
						int currentTick = (int) RepastEssentials.GetTickCount();  //Calculate tick.
						Queue<Road> tempPath = entry.getValue();
						
						double pathTimeOldPath = this.travelTimeForPreviousRoute - (currentTick-this.previousTick) * GlobalVariables.SIMULATION_STEP_SIZE;
						//Xue: make the comparison between the previous route time and the new route time. If the absolute and relative difference are both large 
						//the vehicle will shift to the new route (Mahmassani and Jayakrishnan(1991), System performance  and user  response  under  real-time  information  in a congested  traffic corridor).
						if (pathTimeOldPath - pathTimeNew > indiffBand * pathTimeOldPath) {  //Rajat, Xue
							//System.out.print("relativeDifference \n");
							if (pathTimeOldPath - pathTimeNew > GlobalVariables.TAU) {
								//System.out.print("AbsoluteDifference \n");
								tempPath = tempPathNew;                              // Update path.
								this.travelTimeForPreviousRoute = pathTimeNew;       // Update path time.
								this.previousTick =  currentTick;                    // Update tick.
							}
						}
						Iterator<Road> iter = tempPath.iterator();
						iter.next();
						if (this.checkNextLaneConnected(iter.next())){
							// If the next road is connected to the current lane, then we assign the path, otherwise, we use the old path
							// Clear legacy impact
							this.clearShadowImpact();
							this.roadPath = tempPath;
							this.setShadowImpact();
							this.lastRouteTime = (int) RepastEssentials.GetTickCount();
							Iterator<Road> itr = this.roadPath.iterator();
							itr.next();
							this.nextRoad_ = itr.next();
							flag = true;
						}
					}
				} 
                if (!flag) {
					// Route information is still valid
					// Remove the current road from the path
					this.removeShadowCount(this.roadPath.poll());
//					this.roadPath.remove(0);
					if(this.roadPath.size()==1) {
						this.nextRoad_ = null; // the other way to indicate vehicle has arrived
						return;
					}
					Iterator<Road> itr = this.roadPath.iterator();
					itr.next();
					this.nextRoad_ = itr.next();
				}	

//				if (nextRoad != null)
//					if (this.getVehicleID() == GlobalVariables.Global_Vehicle_ID)
//						System.out.println("Next Road ID for Vehicle: "
//								+ t his.getVehicleID() + " is "
//								+ nextRoad.getLinkid());
//				 if (nextRoad.getLinkid() != this.road.getLinkid()) {
//				 System.out.println("Next Road ID for Vehicle: " +
//				 this.getVehicleID() + " is " + nextRoad.getLinkid());
//				 this.nextRoad_ = nextRoad; } else {
//				 System.out.println("No next road found for Vehicle " +
//				 this.vehicleID_ + " on Road " + this.road.getLinkid());
//				 this.nextRoad_ = null;
//                }

			} else {
				// Clear legacy impact
				this.clearShadowImpact();
				// Compute new route

				//this.roadPath = RouteV.vehicleRoute(this, this.destCoord); 
				Map<Double, Queue<Road>> tempPathMap = RouteV.vehicleRoute(this, this.destZone);  //return the HashMap
				for (Entry<Double, Queue<Road>> entry : tempPathMap.entrySet()) {
					double dist = entry.getKey();
					Queue<Road> path = entry.getValue(); //get the route  
					this.roadPath = (Queue<Road>) path;  //store the route					
					this.setShadowImpact();
					this.lastRouteTime = (int) RepastEssentials.GetTickCount();
					this.atOrigin = false;
					if (dist != 0.0) {
						Iterator<Road> itr = roadPath.iterator();
						itr.next();
						this.nextRoad_ = itr.next(); // RV: null
					} else {
						System.out.println(this + " same road" + this.road.getLinkid() + " for next dest");
						this.nextRoad_ = null;
					}
				}
			}
		} catch (Exception e) {
			e.printStackTrace();
			System.out.println("No next road found for Vehicle "
					+ this.vehicleID_ + " on Road " + this.road.getLinkid());
			this.nextRoad_ = null; // LZ: Remove the vehicle, can we do something better? 
		}
	}

	// BL: Append a vehicle to vehicle list in plane
	public void append(Lane plane) {
		this.lane = plane;
		Vehicle v = plane.lastVehicle();
		plane.addVehicles();
		if (v != null) {
			this.leading(v);
//			For debugging, check if this.distance_ can be less than the front vehicle's
//			if(this.distance_<v.distance_){
//				System.out.println("Wow, " + this.distance_ + "," + v.distance_ + "," + this.lane.getLaneid() + "," + this.lane.getLength());
//			}
			v.trailing(this);
		} else {
			plane.firstVehicle(this);
			this.leading(null);
		}
		this.trailing(null);
		plane.lastVehicle(this);
		this.onlane = true;
	}

	public void setCoordMap(Lane plane) {
		Coordinate lastCoordinate;
		lastCoordinate = this.getCurrentCoord();
		
		if (plane != null) {
			Coordinate[] coords = laneGeography.getGeometry(plane).getCoordinates();
//			Coordinate start, end;
//			start = coords[0];
//			end = coords[coords.length - 1];
//			if (!start.equals(getNearestCoordinate(lastCoordinate, start, end))) {
//				ArrayUtils.reverse(coords);
//				if (this.id == GlobalVariables.Global_Vehicle_ID && !GlobalVariables.Debug_On_Road)
//					System.out.println("Reversed the coordinates for this vehicle's coordinate map");
//			}
			coordMap.clear();
			for (Coordinate coord : coords) {
				this.coordMap.add(coord);
			}
			
			this.setCurrentCoord(this.coordMap.get(0));//LZ: update the vehicle location to be the first pt in the coordMap
			this.coordMap.remove(0);
			this.distance_ = (float) plane.getLength();// - lastStepMove_ / 2; //LZ: lastStepMove_ does note make sense, should be this.length/2
		} else {
			this.coordMap.add(destCoord);
			this.distance_ = (float) distance(lastCoordinate, this.coordMap.get(0));// - lastStepMove_ / 2;
		}
		if (Double.isNaN(distance_)) {
			System.out.println("distance_ is NaN in setCoordMap for " + this);
		}
	}

	/**
	 * update the coordinates of vehicle to the lane coordinate
	 * 
	 * @param: lane
	 */
	private void updateCoordMap(Lane lane) {
		// double newdist_ = 0; // distance between downstream junction & 1st downstream control point
		// double adjustdist_ = 0; // distance between current coord & 1st downstream control point
		Coordinate[] coords = laneGeography.getGeometry(lane).getCoordinates(); // list of control points of new lane
		
		// LZ: This does not work properly, replace with new implementation
//		Coordinate juncCoordinate, nextLaneCoordinate, closeVehCoordinate;
//		juncCoordinate = coords[coords.length - 1]; //The last coordinate of the lane
//
//		// SH Temp
//		Coordinate vcoordinate = this.getCurrentCoord();
//		coordMap.clear();
//		for (int i = 0; i < coords.length - 1; i++) {
//			nextLaneCoordinate = getNearestCoordinate(juncCoordinate,
//					coords[i], coords[i + 1]); // Get the coord that closer to juncCoordinate, but why?
//			// nextLaneCoordinate = coords[i];
//			closeVehCoordinate = getNearestCoordinate(juncCoordinate,
//					vcoordinate, nextLaneCoordinate);
//			if (!closeVehCoordinate.equals(vcoordinate))
//				coordMap.add(nextLaneCoordinate);
//		}
		coordMap.clear();
		double accDist = lane.getLength();

		for (int i = 0; i < coords.length - 1; i++) {
			accDist-=distance(coords[i], coords[i+1]);
			if (this.distance_ >= accDist) { // Find the first pt in CoordMap that has smaller distance;
				double[] distAndAngle = new double[2];
				distance2(coords[i+1], coords[i], distAndAngle);
				Coordinate coord = coords[i+1];
				move2(coord, this.distance_- accDist, distAndAngle[1]); // Update vehicle location
				for (int j = i+1; j < coords.length; j++){ // Add the rest coords into the CoordMap
					coordMap.add(coords[j]);
				}
				break;
			}
		}
		if (coordMap.size() == 0) {
//			System.out.println("Too close to the junction, what is going on?");
			double[] distAndAngle = new double[2];
			distance2(coords[coords.length-1], coords[coords.length-2], distAndAngle);
			Coordinate coord = coords[coords.length-1];
			move2(coord, this.distance_, distAndAngle[1]); // Update vehicle location
			coordMap.add(coords[coords.length-1]);
		}
		// BL: update distance due to the additional length covers by lane width
		// LZ: This is wrong, directly calculate the distance
//		if (coordMap.size() > 0) {
//			double cos = (double) GlobalVariables.LANE_WIDTH / adjustdist_;
//			cos = Math.acos(cos); //Totally wrong!!!
//			adjustdist_ = adjustdist_ * (1 - Math.sin(cos));
		// } else { // if it is very close to an intersection
		// 	System.out.println("Too close to the junction, what is going on? " + this);
		// 	coordMap.add(coords[coords.length-1]);
		// 	newdist_ = 0; 
		// 	adjustdist_ = distance(currentCoord, coordMap.get(0));
		// }
		
		// this.distance_  = (float) (newdist_ + adjustdist_);
		// if (distance_ < 0) {
		// 	System.err.println("distance_ < 0 for " + this);
		// }
		// if (Float.isNaN(this.distance_)) {
		// 	System.err.println("distance_ is NaN for" + this);
		// }
//			//For debug, try to understand, what make adjustdist_ be null
//			if(Double.isNaN(adjustdist_)){
//				System.out.println(distance(vcoordinate, coordMap.get(0))+","+GlobalVariables.LANE_WIDTH / distance(vcoordinate, coordMap.get(0))+","+Math.acos(GlobalVariables.LANE_WIDTH / distance(vcoordinate, coordMap.get(0))));
//			}
//			if (this.id == 178) {
//				System.out.println("Distance adjusted for vehicle: " + this.id
//						+ " is " + adjustdist_);
//			}
//		}else{ 
//			System.out.println("Too close to the junction, what is going on?");
//			coordMap.add(coords[coords.length-1]);
//		}
		
		//For debug, it can be clear seen that adjustdist_ can be null!
//		if(Float.isNaN(this.distance_)){
//			System.out.println(adjustdist_+","+this.coordMap.get(0).x+","+this.coordMap.get(0).y+","+vcoordinate.x+","+vcoordinate.y);
//		}
	}

	public boolean calcState() {
		// SH-- right now there is only one function we may also invoke lane
		// changing decision here
		// SHinvoke accelerating decision
		if(this.road == null || this==null){
			return false;
		}
//		if(this.lane.getLength()>= GlobalVariables.NO_LANECHANGING_LENGTH) { //LZ: Nov 4, add NO_LANECHANGING_LENGTH
		this.makeAcceleratingDecision();
		if (this.road.getnLanes() > 1 && this.onlane && this.distance_>=GlobalVariables.NO_LANECHANGING_LENGTH) { 
				this.makeLaneChangingDecision();
		}
//		}
		return true;
	}

	/*
	 * -------------------------------------------------------------------- The
	 * Car-Following model calculates the acceleration rate based on interaction
	 * with other vehicles. The function returns a the most restrictive
	 * acceleration (deceleration if negative) rate among the rates given by
	 * several constraints. This function updates accRate_ at the end.
	 * --------------------------------------------------------------------
	 */

	public void makeAcceleratingDecision() {
		// int id = GlobalVariables.Global_Vehicle_ID;
		Vehicle front = this.vehicleAhead();
		double aZ = this.accRate_; /* car-following */
		double acc = this.maxAcceleration(); /* returned rate */
		/*
		 * BL: vehicle will have acceleration rate based on car following if it
		 * is not in yielding or nosing state
		 */
		if (!this.nosingFlag && !this.yieldingFlag) {
			aZ = this.calcCarFollowingRate(front);
		} else if (this.nosingFlag) {
			aZ = this.nosing();
		} else if (this.yieldingFlag) {
			aZ = this.yielding();
		}

		if (aZ < acc)
			acc = aZ; // car-following rate

		if (acc < maxDeceleration_) {
			acc = maxDeceleration_;
		}

		accRate_ = acc;
		if (Double.isNaN(accRate_)) {
			System.err.println("NaN acceleration rate for " + this);
		}
	}

	public double calcCarFollowingRate(Vehicle front) {
		// SH-if there is no front vehicle the car will be in free flow regime
		// and have max acceleration

		if (front == null) {
			this.regime_ = GlobalVariables.STATUS_REGIME_FREEFLOWING;
			return (this.maxAcceleration_);
		}

		double acc;

		double space = gapDistance(front);
		// if (ContextCreator.debug)
		// For debug, we can clear see that leading is corrupted
//		if(space <0){
//			System.out.println("Gap with front vehicle :"+ space+","+front.distance_+","+this.distance_+","+this.lane.getLaneid()+","+front.lane.getLaneid()+","+this.vehicleID_+","+front.vehicleID_);
//		}

		double speed = currentSpeed_ == 0f ? 0.00001f : currentSpeed_;

		double headway = 2.0f * space / (speed + currentSpeed_);
		float hupper, hlower;

		float AlphaDec = GlobalVariables.ALPHA_DEC;
		float BetaDec = GlobalVariables.BETA_DEC;
		float GammaDec = GlobalVariables.GAMMA_DEC;

		float AlphaAcc = GlobalVariables.ALPHA_ACC;
		float BetaAcc = GlobalVariables.BETA_ACC;
		float GammaAcc = GlobalVariables.GAMMA_ACC;

		// float dens = this.road.density();

		hupper = GlobalVariables.H_UPPER;
		hlower = GlobalVariables.H_LOWER;

		// There will be three regimes emergency/free-flow/car-following regime
		// depending on headway
		

		// Emergency regime
		if (headway < hlower) {
			float dv = currentSpeed_ - front.currentSpeed_;

			if (dv < GlobalVariables.SPEED_EPSILON) { // the leader is
				// decelerating
				acc = front.accRate_ + 0.25f * normalDeceleration_;
			} else {
				if (space > 0.01) {
					acc = front.accRate_ - 0.5f * dv * dv / space;
				} else {
					float dt = GlobalVariables.SIMULATION_STEP_SIZE;
					double v = front.currentSpeed_ + front.accRate_ * dt;
					space += 0.5f * (front.currentSpeed_ + v) * dt;
					acc = brakeToTargetSpeed(space, v);
				}
			}
			acc = Math.min(normalDeceleration_, acc);
			regime_ = GlobalVariables.STATUS_REGIME_EMERGENCY;
		}
		// Free-flow regime
		else if (headway > hupper) { // desired speed model will do
			if (space > distanceToNormalStop_) {
				acc = maxAcceleration_;
				regime_ = GlobalVariables.STATUS_REGIME_FREEFLOWING;
			} else {
				float dt = GlobalVariables.SIMULATION_STEP_SIZE;
				double v = front.currentSpeed_ + front.accRate_ * dt;
				space += 0.5 * (front.currentSpeed_ + v) * dt;
				acc = brakeToTargetSpeed(space, v);
				regime_ = GlobalVariables.STATUS_REGIME_FREEFLOWING;
			}
		}
		// SH: We are using Herman model
		else {
			float dv = front.currentSpeed_ - currentSpeed_;
			if (dv < 0) {
				acc = dv * AlphaDec * (float) Math.pow(currentSpeed_, BetaDec)
						/ (float) (Math.pow(space, GammaDec));

			} else if (dv > 0) {
				acc = dv * AlphaAcc * (float) Math.pow(currentSpeed_, BetaAcc)
						/ (float) (Math.pow(space, GammaAcc));
			} else { // uniform speed
				acc = 0.0f;
			}
			regime_ = GlobalVariables.STATUS_REGIME_CARFOLLOWING;
		}
		if (Double.isNaN(acc)) {
			System.err.println("acc is NaN for " + this);
		}
		return acc;
	}

	public double brakeToTargetSpeed(double space, double v) {
		if (space > GlobalVariables.FLT_EPSILON) {
			double v2 = v * v;
			double u2 = currentSpeed_ * currentSpeed_;
			double acc = (v2 - u2) / space * 0.5f;
			return acc;
		} else {
			float dt = GlobalVariables.SIMULATION_STEP_SIZE;
			if (dt <= 0.0)
				return maxAcceleration_;
			return (v - currentSpeed_) / dt;
		}
	}

	public Vehicle vehicleAhead() {
		if (leading_ != null) {
			return leading_;
		} else if (nextLane_ != null) {
			if (nextLane_.lastVehicle() != null)
				return nextLane_.lastVehicle();
			else
				return null;
		} else {
			return null;
		}
	}

	public Junction nextJuction() {
		Junction nextJunction;
		if (this.nextRoad() == null)
			return null;
		if (this.nextRoad().getJunctions().get(0).getID() == this.road
				.getJunctions().get(1).getID())
			nextJunction = this.road.getJunctions().get(1);
		else
			nextJunction = this.road.getJunctions().get(0);
		return nextJunction;
	}

	public double gapDistance(Vehicle front) {
		double headwayDistance;
		if (front != null) { /* vehicle ahead */	
			if (front.lane == null){ // Front vehicle is being killed
				headwayDistance = Float.MAX_VALUE;
			}
			else if (this.lane.getID() == front.lane.getID()) { /* same lane */
				headwayDistance = this.distance_ - front.distance(); //-front.length();
			} else { /* different lane */
				headwayDistance = this.distance_
						+ (float) front.lane.getLength() - front.distance(); //LZ: front vehicle is in the next road //-front.length();
			}
		} else { /* no vehicle ahead. */
			headwayDistance = Float.MAX_VALUE;
		}
		if (Double.isNaN(headwayDistance)) {
			System.out.println("headway is NaN");
		}
		return (headwayDistance);
	}

	public void makeLaneChangingDecision() {
		if (this.distFraction() < 0.5) { 
			// Halfway to the downstream intersection, only mantatory LC allowed, check the correct lane
			if (this.isCorrectLane() != true) { // change lane if not in correct
				// lane
				Lane plane = this.tempLane();
				if (plane != null)
					this.mandatoryLC(plane);
				else {
					//HG and XQ: commented out this because now we are allowing forced jump in 5 leg or irregular intersections for which 
					//it is not possible to assign lane information properly (as only left, right and through are allowed.)). Previously 5 leg intersections were
					//converted to 4 leg intersections in the shape file and then this comment was put to check if anything goes wrong. But we leave all intersections as it they are.
					//so, this error will get printed but forced jump will bypass blocking of vehicles.
//					System.out.println("Vehicle " + this.getId()
//							+ "has no lane to change");
//					System.out.println("this vehicle is on road "
//							+ this.road.getLinkid() + " which has "
//							+ this.road.getnLanes()
//							+ " lane(s) and I am on lane "
//							+ this.road.getLaneIndex(this.lane)
//							+" with Lane " + this.lane.getLaneid() +" and next Lane " + this.nextLane_.getLaneid());
				}
			}
		} else {
			if (this.distFraction() > 0.75) {
				// First 25% in the road, do discretionary LC with 100% chance
				double laneChangeProb1 = GlobalVariables.RandomGenerator.nextDouble();
				// The vehicle is at beginning of the lane, it is free to change
				// lane
				Lane tarLane = this.findBetterCorrectLane();
				if (tarLane != null) {
					if (laneChangeProb1 < 1.0)
						this.discretionaryLC(tarLane);
				}
			} else {
				// First 25%-50% in the road, we do discretionary LC but only to
				// correct lanes with 100% chance
				double laneChangeProb2 = GlobalVariables.RandomGenerator.nextDouble();
				// The vehicle is at beginning of the lane, it is free to change
				// lane
				Lane tarLane = this.findBetterCorrectLane();
				if (tarLane != null) {
					if (laneChangeProb2 < 1.0)
						this.discretionaryLC(tarLane);
				}
			}
		}
	}
	
	public void makeLaneChangingDecision_oldCode() { //OLD CODE
		// if not using dynamic routing alg, then check the correct lane
		if (this.distFraction() > 0.70) {
			double laneChangeProb = GlobalVariables.RandomGenerator
					.nextDouble();
			// The vehicle is at beginning of the lane, it is free to change
			// lane
			Lane tarLane = this.findBetterLane();
			if (tarLane != null) {
				// TODO input a PDF of normal distribution
				if (laneChangeProb > 0.5)
					this.discretionaryLC(tarLane);
			}
		} else if (this.distFraction() > 0.50) {
			// vehicle is half the lane length to downstream node
			// only DLC if the target lane is also correct lane
			// otherwise only MLC
			if (this.isCorrectLane() != true) {
				// change lane if not in correct lane
				Lane plane = this.tempLane();
				if (plane != null)
					this.mandatoryLC(plane);
				else {
					System.out.println("Vehicle " + this.getId()
							+ "has no lane to change");
				}
			} else {
				// if the vehicle is already in correct lane.
				// it seek for another connected lane with better traffic
				// condition
				Lane tarLane = this.findBetterCorrectLane();
				if (tarLane != null) {
					this.discretionaryLC(tarLane);
				}
			}
		} else {
			// when the vehicle is close to downstream node, only DLC is
			// accepted
			if (this.isCorrectLane() != true) {
				// change lane if not in correct lane
				Lane plane = this.tempLane();
				if (plane != null)
					this.mandatoryLC(plane);
				else {
					System.out.println("Vehicle " + this.getId()
							+ "has no lane to change");
				}
			}
		}
	}
	
	/**
	 * HGehlot: Record the vehicle snapshot if this tick corresponds to the
	 * required epoch that is needed for visualization interpolation.
	 * Note that this is recording is independent of snapshots of vehicles 
	 * whether they move or not in the current tick. 
	 * (So when vehicles do not move in a tick but we need to record positions
	 * for visualization interpolation then recVehSnaphotForVisInterp is useful). 
	 * Also, we update the coordinates of the previous epoch in the end of the
	 * function.
	 */ 
	public void recVehSnaphotForVisInterp(){
		Coordinate currentCoord = this.getCurrentCoord();
		if( currentCoord != null){
			try {
				/*
				 * HG: the following condition can be put to reduce the data when 
				 * the output of interest is the final case when vehicles reach 
				 * close to destination
				 * 
				 * Update: I use currentCoord rather than the targeted coordinates
				 * (as in move() function) and this is an approximation but anyway
				 * if the vehicle moves then it will get overridden.
				 */
				DataCollector.getInstance().recordVehicleTickSnapshot(this, currentCoord);
			}
			catch (Throwable t) {
			    // could not log the vehicle's new position in data buffer!
			    DataCollector.printDebug("ERR" + t.getMessage());
			}
			// update the previous coordinate as the current coordinate
			setPreviousEpochCoord(currentCoord);
		}
	}
	
	public Coordinate getPreviousEpochCoord() {
		return this.previousEpochCoord;
	}
	
	private void setPreviousEpochCoord(Coordinate newCoord){
		this.previousEpochCoord.x = newCoord.x;
		this.previousEpochCoord.y = newCoord.y;
	}
	
	/*
	 * Calculate new location and speed after an iteration based on its current
	 * location, speed and acceleration. The vehicle will be removed from the
	 * network if it arrives its destination.
	 */
	public void travel() {
		this.endTime++;
		try {
			if (!this.reachDest && !this.reachActLocation) {
				this.move(); // move the vehicle towards their destination
				
				this.advanceInMacroList(); // BL: if the vehicle travel too fast, it will change the marcroList of the road.
				
				// up to this point this.reachDest == false;
				if (this.nextRoad() == null) {
					this.checkAtDestination();
				}
			}

		} catch (Exception e) {
			try { // print the error-causing vehicle during move()
				System.err.println("Vehicle " + this.getVehicleID()
						+ " had an error while travelling on road: "
						+ this.road.getLinkid() + "with next road: "
						+ this.nextRoad().getLinkid());
				e.printStackTrace();
				RunEnvironment.getInstance().pauseRun();
			}
			catch (NullPointerException exc) { // LZ,RV: in case next road is null
				System.err.println("Vehicle " + this.getVehicleID()
				+ " had an error while travelling on road: "
				+ this.road.getLinkid() + "with next road: ");
				e.printStackTrace();
				RunEnvironment.getInstance().pauseRun();
			}
		}
	}

	public void move() {
		//HG:update current speed based on freeflow speed
//		if(this.currentSpeed_ > this.road.getFreeSpeed())
//			this.currentSpeed_ = (float) this.road.getFreeSpeed();
		
//		int[] selectVehicleIds = {131876};//, 96280, 371677, 102308, 96223};
//		for (int vehId : selectVehicleIds) {
//			if (this.vehicleID_ == vehId) {
//				int x=0;
//			}
//			break;
//		}
		// LZ: The vehicle is close enough to the intersection/destination
		double distance = this.distance_;
		if (distance < GlobalVariables.INTERSECTION_BUFFER_LENGTH) { 
			if (this.nextRoad() != null) { // Has next road to go, otherwise it reaches the destination
				if (this.isOnLane()) { // On lane
					this.coordMap.add(this.getCurrentCoord()); // Stop and wait
					if (this.appendToJunction(nextLane_) == 0) { // This will make this.isOnLane becomes false, return 0 means the vehicle cannot enter the next road
						this.lastStepMove_ = 0;
					} else {
						this.lastStepMove_ = distance; // Successfully entered the next road, update the lastStepMove and accumulatedDistance
						this.accummulatedDistance_ += this.lastStepMove_;
					}
					return; // move finished
				} else { // not on lane, directly changing road
					if (this.changeRoad() == 0) { // 0 means the vehicle cannot enter the next road
						stuck_time+=1;
						this.lastStepMove_ = 0;
					} else {
						stuck_time=0;
						this.lastStepMove_ = distance; //Successfully entered the next road, update the lastStepMove and accumulatedDistance
						this.accummulatedDistance_ += this.lastStepMove_;
					}
					return;// move finished
				}
			} else{
				return; // do nothing since the vehicle reached destination
			}
		}

		Coordinate currentCoord = null;
		Coordinate target = null;
		double dx = 0; // LZ: travel distance calculated by physics
		
		boolean travelledMaxDist = false; // True when traveled with maximum distance, which is dx.
		double distTravelled = 0; // The distance traveled so far.
		
		/*
		 * For debuging: print out the current road and next junction ID of the
		 * vehicle
		 */
		// System.out.println("Vehicle: " + this.vehicleID_ + " on Road: "
		// + this.road.getID() + " next Road: " + this.nextRoad().getLinkid());

		/*
		 * Calling the static method in Signal class If you comment out this
		 * line it would be just vehicle movement without signals
		 */
		
		float step = GlobalVariables.SIMULATION_STEP_SIZE; // 0.3
//		if (currentSpeed_ < GlobalVariables.SPEED_EPSILON && accRate_ < GlobalVariables.ACC_EPSILON) { //0.001
//			return; // Does not move
//		}
		// For debugging, check if the inputs for calculating dx can be null, not here
//		if(Float.isNaN(currentSpeed_)|| Float.isNaN(accRate_) ){
//			System.out.println(currentSpeed_+","+accRate_+","+this.distance_);
//		}
		float oldv = currentSpeed_; // Velocity at the beginning

		double dv = accRate_ * step; // Change of speed

		if (dv > -currentSpeed_) { // still moving at the end of the cycle
			dx = currentSpeed_ * step + 0.5f * dv * step;

		} else { // stops before the cycle end
			dx = -0.5f * currentSpeed_ * currentSpeed_ / accRate_;
			if (currentSpeed_ == 0.0f && accRate_ == 0.0f) {
				dx = 0.0f;
			}
//			else if (accRate_ == 0.0f) {
//				System.out.println("speed not 0 but accel = 0 & still vehicle stops " + this);
//				dx = 0.0f;
//			}
		}
		if (Double.isNaN(dx)) {
			System.out.println("dx is NaN in move() for " + this);
		}

		// Solve the crash problem 
		Vehicle front = this.vehicleAhead();
		if (front != null) {
			double gap = gapDistance(front);
			if (gap > this.length()) {
				dx = Math.min(dx, gap - this.length());
			} else {
				dx = 0.0f;
			}
		}
		
		// actual acceleration rate applied in last time interval.
		accRate_ = (float) (2.0f * (dx - oldv * step) / (step * step));
//
//		// update speed
		currentSpeed_ += accRate_ * step;
		if (currentSpeed_ < GlobalVariables.SPEED_EPSILON) {
			currentSpeed_ = 0.0f;
			accRate_ = 0.0f; // no back up allowed
		} else if (currentSpeed_ > this.road.getFreeSpeed() && accRate_ > GlobalVariables.ACC_EPSILON) {
			currentSpeed_ = (float) this.road.getFreeSpeed();
//			accRate_ = (currentSpeed_ - oldv) / step;
//			if (accRate_ == 0) {
//				int x = 0;
//			}
//		}
//		if (accRate_ == 0) {
//			int x = 0;
			accRate_ = (float) ((currentSpeed_ - oldv) / step);
		}
//
		if (dx < 0.0f) { // Negative dx is not allowed
			lastStepMove_ = 0;
			return;
		}
		
//		// update position
//		distance_ -= dx;
		/*
		 * Check if the vehicle's dx is under some threshold, i.e. it
		 * will move to the next road 1. Search for the junction 2. Search for
		 * road id 3. Find the first vehicle of all the lanes of those roads.
		 */
		while (!travelledMaxDist) {
			
			// Current location
			currentCoord = this.getCurrentCoord();

			/*
			 * Solve the no coordinate problem (10/23/2012) need to recall
			 * last coordinate if it was deleted.
			 */
			
			target = this.coordMap.get(0);
//			if (Double.isNaN(target.x) || Double.isNaN(target.y)) {
//				System.err.println("NaN target during move() for " + this + " currently at (" +
//			        currentCoord.x + ", " + currentCoord.y + ")");
//			}
//			if (target.x == currentCoord.x && target.y == currentCoord.y) {
////				System.out.println("Current coord same as target for " + this);
//			}

			// Geometry currentGeom = geomFac.createPoint(currentCoord);
			
			// SH: new block of code for using new distance
//			double[] distAndAngle = new double[2];
//			double distToTarget =  this.distance(currentCoord, target, distAndAngle);
			
			// LZ: replace previous vehicle movement function
			double[] distAndAngle = new double[2]; // the first element is the distance, and the second is the radius
			double distToTarget = this.distance2(currentCoord, target, distAndAngle);
			
			//For debug, print out the vehicle's linkID and speed, acc, ...
//			if(dx <= 0.1){
//				System.out.println("Behind: "+this.road.getLinkid()+","+this.getLane().getID()+","+this.getVehicleID()+"," + this.currentSpeed_+","+this.accRate_+","+this.distance_+","+distToTarget);
//				if(this.macroLeading_!=null){
//					System.out.println("Front: "+this.road.getLinkid()+","+this.getLane().getID()+","+this.macroLeading_.getVehicleID()+"," + this.macroLeading_.currentSpeed_+","+this.macroLeading_.accRate_+","+this.macroLeading_.distance_+","+this.macroLeading_.lastStepMove_);
//				}
//			}
			
			// TODO: Vehicle can stop at the intersection for long time, how to fix it?
			// If we can get all the way to the next coords on the route then, just go there
			if (distTravelled + distToTarget <= dx) {
				distTravelled += distToTarget;
				this.setCurrentCoord(target);
				// LZ: Oct 31, the distance and calculated value is not consistent (Vehicle reached the end of the link does not mean Vehicle.distance_ <= 0), therefore, use a intersection buffer （see the changeRoad block above)
				this.coordMap.remove(0);
				if (this.coordMap.isEmpty()) {
					if (this.nextRoad() != null) { // has next road
						if (this.isOnLane()) {
							this.coordMap.add(this.getCurrentCoord()); // Stop and wait
							if(this.appendToJunction(nextLane_)==0){
								stuck_time+=1;
							}else{
								stuck_time = 0;
							}
							lastStepMove_ = distTravelled;
							accummulatedDistance_ += distTravelled;
							break; 
						} else {
							if(this.changeRoad()==0){
								stuck_time+=1;
							}else{
								stuck_time=0;
							}
							lastStepMove_ = distTravelled;
							accummulatedDistance_ += distTravelled;
							break;
						}
					} else { // no next road, the vehicle arrived at the destination
						this.coordMap.clear();
						this.coordMap.add(this.currentCoord_);
						break;
					}
				}
			}

			// Otherwise move as far as we can towards the target along the road
			// we're on get the angle between the two points
			// (current and target)
			// (http://forum.java.sun.com/thread.jspa?threadID=438608&messageID=1973655)
			// LZ: replaced the complicated operation with a equivalent but simpler one
			else {
				// double angle = angle(target, currentCoord) + Math.PI;
				// angle() returns range from -PI->PI,
				// but moveByVector wants range 0->2PI
//				vehicleGeography.moveByVector(this, dx, angle);
				
//				System.out.println("dx: "+ dx + " angle: " + distAndAngle[1]);

//				// Old implementation: This approach seems to be fast but still have some issues.
//				vehicleGeography.moveByVector(this, dx, distAndAngle[1]);
				
				/*// Zhan: implementation 1: adding concurrency locks
				this.lock.lock();
				vehicleGeography.moveByVector(this, dx, distAndAngle[1]);
				this.lock.unlock();*/
				
				// Zhan: implementation 2: thread safe version of the moveByVector
//				 this.moveVehicleByVector(dx, distAndAngle[1]);
				// LZ
//				double alpha = (dx-distTravelled)/distToTarget;
//				if (Double.isNaN(alpha) || Double.isNaN(deltaXY[0]) || Double.isNaN(deltaXY[0])) {
//					System.err.println("alpha or deltaXY NaN in move() for " + this);
//				}
//				move2(alpha*deltaXY[0], alpha*deltaXY[1]);
				//double alpha = (dx-distTravelled)/distToTarget;
				move2(currentCoord, dx-distTravelled, distAndAngle[1]); // move by distance in the calculated direction
//				currentCoord.x += alpha*deltaXY[0];
//				currentCoord.y += alpha*deltaXY[1];
//				this.setCurrentCoord(currentCoord);
				distTravelled = dx;
				this.accummulatedDistance_ += dx;
				lastStepMove_ = dx;
				// SH: Trying to remove this function context creator but this
				// is not working either
				travelledMaxDist = true;
			} // else

//			printGlobalVehicle(dx);
			// Handle lane changing behavior
			
		}
		// update the position of vehicles, 0<=distance_<=lane.length()
		distance_ -= distTravelled;
//		if(distTravelled<dx){
//			System.out.println("Previous distance: "+ distance+","+ this.distance_ + ","+ distTravelled+","+dx+","+this.lane.getLaneid());
//		}
		if (distance_ < 0) {
			distance_=0;
		}
		//LZ: For debugging, here we observed that this.distance_ can be set to NaN, but other values are valid (even in the first time this message occured)
//		if(Float.isNaN(distance_) ){
//			System.out.println(dx+","+currentSpeed_+","+accRate_+","+this.distance_);
//		}
//		this.currentSpeed_ = (float) (distTravelled/step);
//		this.accRate_ = (this.currentSpeed_-oldv)/step;
		return;
	}

	public void printGlobalVehicle(float dx) {
		if (this.vehicleID_ == GlobalVariables.Global_Vehicle_ID) {
			System.out.println("Next Road ID for vhielc: " + this.vehicleID_
					+ " is: " + this.nextRoad().getLinkid() + " step size is: "
					+ dx + " distance to downstream: " + this.distance_ + 
					" next lane: " + this.nextLane_
					+ " current speed: " + this.currentSpeed_);
		}
	}

	public void primitiveMove() {
		Coordinate currentCoord = null;
		Coordinate target = null;
		if (this.atDestination()) {
			return;
		}
//		boolean travelledMaxDist = false; // True when traveled max dist this
		// iteration
		
		// SH Temp
		// Geography<Vehicle> vehicleGeography = ContextCreator
		// .getVehicleGeography();
		// GeometryFactory geomFac = new GeometryFactory();
		currentCoord = this.getCurrentCoord();
		// The first list of coordinates for the vehicle to follow
		if (this.coordMap.size() > 0) {
			target = this.coordMap.get(0);
		} else {
			lane = this.road.firstLane();
			Coordinate[] coords = laneGeography.getGeometry(lane).getCoordinates();
			for(Coordinate coord: coords){
				this.coordMap.add(coord);
			}
			//this.setCoordMap(lane);
			target = this.coordMap.get(0);
		}

		// target = this.route.routeMap().get(0);

		// Geometry currentGeom = geomFac.createPoint(currentCoord);
		// Geometry targetGeom = geomFac.createPoint(target);

		// double distToTarget = DistanceOp.distance(currentGeom,
		// targetGeom);

		// double[] distAndAngle = new double[2];
		// double distToTarget = this.distance(currentCoord, target,
		// distAndAngle);

		double[] distAndAngle = new double[2];
		double distToTarget;
		distToTarget = this.distance2(currentCoord, target, distAndAngle);
		
		if (distToTarget <= travelPerTurn) { // Include equal, which is important
			// this.lock.lock();
			// vehicleGeography.move(this, targetGeom);
			this.setCurrentCoord(target);
			
//			System.out.println("This is called.");
			// try {
			// //HG: the following condition can be put to reduce the data when
			// the output of interest is the final case when vehicles reach
			// close to destination
			//// if(this.nextRoad() == null){
			// DataCollector.getInstance().recordSnapshot(this, target);
			//// }
			// }
			// catch (Throwable t) {
			// // could not log the vehicle's new position in data buffer!
			// DataCollector.printDebug("ERR" + t.getMessage());
			// }
			// this.lock.unlock();
			// this.accummulatedDistance_+=ContextCreator.convertToMeters(distToTarget);
			// if(this.vehicleID_ == GlobalVariables.Global_Vehicle_ID)
			// System.out.println("distToTarget=
			// "+ContextCreator.convertToMeters(distToTarget));
			// System.out.println(this.coordMap.size());
//			this.coordMap.remove(0);
			// this.route.remove();
		}
		// Otherwise move as far as we can towards the target along the road
		// we're on
		// Get the angle between the two points (current and target)
		// (http://forum.java.sun.com/thread.jspa?threadID=438608&messageID=1973655)

		else {
			// double angle = angle(target, currentCoord) + Math.PI; //
			// angle()
			// returns range from -PI->PI, but moveByVector wants range
			// 0->2PI
			double distToTravel = travelPerTurn;
			// Need to convert distance from long/lat degrees to meters
//			double distToTravelM = ContextCreator.convertToMeters(distToTravel);
			// System.out.println("Angle: "+angle);
			this.accummulatedDistance_ += distToTravel;
			// if(this.vehicleID_ == GlobalVariables.Global_Vehicle_ID)
			// System.out.println("Vehicle ID: " + this.getVehicleID()
			// +" disToTravelM= "+distToTravelM);

			// Zhan: implementation 1: add locks to enforce concurrency
			/*
			 * this.lock.lock(); vehicleGeography.moveByVector(this,
			 * distToTravelM, distAndAngle[1]); this.lock.unlock();
			 */
			// vehicleGeography.moveByVector(this,
			// distToTravelM,distAndAngle[1]);
			// Zhan: implementation 2: thread safe version of the moveByVector
			// this.moveVehicleByVector(distToTravelM, distAndAngle[1]);

			// LZ: implementation 3: drop the geom class and change the coordinates
			//double alpha = distToTravelM / distToTarget;
			move2(currentCoord, distToTravel, distAndAngle[1]);
			// currentCoord.x += alpha*deltaXY[0];
			// currentCoord.y += alpha*deltaXY[1];
			// this.setCurrentCoord(currentCoord);

//			travelledMaxDist = true;
		}
		return;
	}

	/**
	 * SH: This function change the vehicle from its current road to the next
	 * road.
	 * 
	 * @return
	 */

	public int changeRoad() {
		// SH- check if the vehicle has reached the destination or not
		if (this.atDestination()) {
//			this.removeFromLane();
//			this.removeFromMacroList();
			this.clearShadowImpact(); // ZH: Clear shadow impact if already reaches destination
			return 0; // only one will reach destination once
		} else if (this.nextRoad_ != null) {
			// BL: check if there is enough space in the next road to change to
			int tickcount = (int) RepastEssentials.GetTickCount(); 
			// LZ: Nov 4, short lane and the vehicle can move freely
			// Check if the target long road has space
			
			if (this.entranceGap(nextLane_) >= this.length()  && (tickcount>this.nextLane_.getLastEnterTick())) { //LZ: redundant if condition
//				if (this.coordMap.isEmpty()) {
//					Coordinate coor = null;
//					Coordinate[] coords = laneGeography.getGeometry(nextLane_)
//							.getCoordinates();
//					Coordinate lastCoordinate = null, start, end;
//					// SH Temp
//					// Geography<Vehicle> vehicleGeography;
//					// vehicleGeography = ContextCreator.getVehicleGeography();
//					lastCoordinate = this.getCurrentCoord();
//					start = coords[0];
//					end = coords[coords.length - 1];
//					coor = this.getNearestCoordinate(lastCoordinate, start, end);
//					coordMap.add(coor);
//				}
//				return 0;
//			} else {
//				float maxMove = GlobalVariables.FREE_SPEED
//						* GlobalVariables.SIMULATION_STEP_SIZE;
				// if (distance_ < maxMove && !onlane) {
				this.nextLane_.updateLastEnterTick(tickcount); //LZ: update enter tick so other vehicle cannot enter this road in this tick
				this.removeFromLane();
				this.removeFromMacroList();
				this.setCoordMap(nextLane_);
//				this.lock.lock();
//				while(this.nextRoad().isLocked());//LZ: Wait until the lock is released
//				this.nextRoad().setLock();
				this.appendToRoad(this.nextRoad());
//				this.nextRoad().releaseLock();
//				if(this.distance_<=0){
//					System.out.println("Here 1");
//				}
				this.append(nextLane_); // LZ: Two vehicles entered the same lane, then messed up.
//				this.lock.unlock();  
				this.setNextRoad();
				this.assignNextLane();
//				if(this.lane.getLength()<GlobalVariables.NO_LANECHANGING_LENGTH){ //move to the end of the lane
//					this.distance_ = 0;
//					this.setCurrentCoord(this.coordMap.get(this.coordMap.size()-1));
//				}
				// Reset the desired speed according to the new road
				this.desiredSpeed_ =  this.road.getFreeSpeed();
				//HG: Need to update current speed according to the new free speed 
				//LZ: Use current speed instead of the free speed, be consistent with the setting in enteringNetwork
				if(this.currentSpeed_ > this.road.getFreeSpeed())
					this.currentSpeed_ = this.road.getFreeSpeed();
					//this.currentSpeed_ = (float) (this.road.getFreeSpeed());
				return 1;
//				}
			}
		else if(this.stuck_time>400){ // Stuck for 2 minutes, try another downward lane
//			System.out.println("THIS IS CALLED");
			for(Lane dnlane: this.lane.getDnLanes()){
				if (this.entranceGap(dnlane) >= this.length() && (tickcount>dnlane.getLastEnterTick())) {
					dnlane.updateLastEnterTick(tickcount);
					this.setCoordMap(dnlane);
					this.removeFromLane();
					this.removeFromMacroList();
					this.appendToRoad(dnlane.road_());
					this.append(dnlane); 
					this.lastRouteTime=-1; // Old route is not valid, for sure
					this.setNextRoad();
					this.assignNextLane();
//					if(this.lane.getLength()<GlobalVariables.NO_LANECHANGING_LENGTH){ //move to the end of the lane
//						this.distance_ = 0;
//						this.setCurrentCoord(this.coordMap.get(this.coordMap.size()-1));
//					}
					this.desiredSpeed_ =  this.road.getFreeSpeed();
					//HG: Need to update current speed according to the new free speed 
					//LZ: Use current speed instead of the free speed, be consistent with the setting in enteringNetwork
					if(this.currentSpeed_ > this.road.getFreeSpeed())
						this.currentSpeed_ = this.road.getFreeSpeed();
						//this.currentSpeed_ = (float) (this.road.getFreeSpeed());
					return 1;
				}
			}
		}
		}
		coordMap.clear();
		coordMap.add(this.getCurrentCoord());//LZ: Fail to enter next link, try again in the next tick
		return 0;
	}

	public int closeToRoad(Road road) {
		// SH Temp
		Coordinate currentCoord = this.getCurrentCoord();
//		GeometryFactory geomFac = new GeometryFactory();
		Coordinate nextCoord = null;

		if (this.coordMap == null)
			return 0;
		else if (this.coordMap.size() == 0)
			return 0;
//			System.out.println("Zero size coordMap for " + this);
		else
			nextCoord = this.coordMap.get(0);

//		Geometry geom1 = geomFac.createPoint(currentCoord);
//		Geometry geom2 = geomFac.createPoint(nextCoord);
//		DistanceOp dist1 = new DistanceOp(geom1, geom2);

		if (distance(currentCoord, nextCoord) < GlobalVariables.TRAVEL_PER_TURN) {
			return 1;
		} else
			return 0;
	}

	public boolean atDestination() {
		return this.reachDest;
	}

	public boolean atActivityLocation() {
		return this.reachActLocation;
	}

	public boolean checkAtDestination() throws Exception { // Close to the last intersection
		// double maxMove = this.road.getFreeSpeed() * GlobalVariables.SIMULATION_STEP_SIZE;

		if (distance_ < GlobalVariables.INTERSECTION_BUFFER_LENGTH) {
			this.setReachDest();
			return true;
		}
		return false;
	}

	public float maxAcceleration() {
		return maxAcceleration_;
	}

	public void appendToRoad(Road road) {
		this.road = road;
		this.appendToMacroList(road);
		this.reachActLocation = false;
	}

	public void appendToMacroList(Road road) {
		macroTrailing_ = null;
		//LZ: Oct 14, 2020 update
		//This has trouble with the advanceInMacroList 
		//If the macroLeading is modified in advanceInMacroList by other thread
		//Then this vehicle will be misplaced in the Linked List
		if (road.lastVehicle() != null){
			road.lastVehicle().macroTrailing_ = this; 
			macroLeading_ = road.lastVehicle();
		}
		else{
			macroLeading_ = null;
			road.firstVehicle(this);
		}
		road.lastVehicle(this);
		// macroLeading_ = road.lastVehicle();
		// road.lastVehicle(this);
		// if (macroLeading_ != null) // there is a vehicle ahead
		// {
		// macroLeading_.macroTrailing_ = this;
		// } else {
		// road.firstVehicle(this);
		// }
		// after this appending, update the number of vehicles
		road.changeNumberOfVehicles(1);
		// LZ: Oct 14, 2020 update, this part is unnecessary
		// we just need to increase the nVehicles_ by 1, as above
		// Vehicle pv = road.firstVehicle();
		// int nVehicles_ = 0;
		// while (pv != null) {
		// nVehicles_++;
		// pv = pv.macroTrailing_;
		// }
		// road.setNumberOfVehicles(nVehicles_);
	}

	public void leading(Vehicle v) {
		if (v != null)
			this.leading_ = v;
		else
			this.leading_ = null;
	}
	
	public void clearMacroLeading(){
		this.macroLeading_ = null;
	}

	public Vehicle leading() {
		return leading_;
	}

	public Vehicle macroLeading() {
		return macroLeading_;
	}

	public Vehicle trailing() {
		return trailing_;
	}

	public Vehicle macroTrailing() {
		return macroTrailing_;
	}

	public void trailing(Vehicle v) {
		if (v != null)
			this.trailing_ = v;
		else
			this.trailing_ = null;
	}

	public void setDepTime(int time) {
		this.deptime = time;
	}

	public int getDepTime() {
		return this.deptime;
	}

	public int getEndTime() {
	    return this.endTime;
	}
	
	public void setRoad(Road road) {
		this.road = road;
		this.currentSpeed_ = (float) this.road.getFreeSpeed();
	}

	public Road getRoad() {
		return road;
	}

	public double distance() {
		return distance_;
	}

	public double distFraction() {
		if (distance_ > 0)
			return distance_ /  this.lane.getLength();
		else
			return 0;
	}

	public float length() {
		return length;
	}

//	public void setGeography() {
//		// SH temp
//		vehicleGeography = ContextCreator.getVehicleGeography();
//
//	}

	public int getId() {
		return id;
	}

	public void setId(int id) {
		this.id = id;
	}

	public Lane getLane() {
		return lane;
	}

	public int getVehicleID() {
		return this.vehicleID_;
	}

	public int getDestinationID() {
		return this.destZone.getIntegerId();
	}

	public House getHouse() {
		return this.house;
	}
	
	// LZ: reset house to update plan
	public void setHouse(House h){
		this.house = h;
	}

	public Coordinate getOriginalCoord() {
		return originalCoord;
	}

	public Coordinate getDestCoord() {
		return this.destCoord;
	}

	public void setOriginalCoord(Coordinate coord) {
		this.originalCoord = coord;
	}
	
	public Coordinate getCurrentCoord() {
		Coordinate coord = new Coordinate();
		coord.x = this.currentCoord_.x;
		coord.y = this.currentCoord_.y;
		coord.z = this.currentCoord_.z;
		return coord;
	}
	
	public void setCurrentCoord(Coordinate coord) {
		this.currentCoord_.x = coord.x;
		this.currentCoord_.y = coord.y;
		this.currentCoord_.z = coord.z;
	}

	public int nearlyArrived(){//HG: If nearly arrived then return 1 else 0
		if(this.nextRoad_ == null){
			return 1;
		}else{
			return 0;
		}
	}
	
	public void setReachDest() throws Exception {
		// get the current zone (i.e., destination zone prior to relocation)
		Zone destinationZone = ContextCreator.getCityContext().findHouseWithDestID(
				this.getDestinationID());
		// if the destination is a shelter
		if (destinationZone.getType() == 1) {
			System.out.println(this + " reached shelter " + destinationZone);
			this.findNextShelter(destinationZone);
		}
		// if the current destination is not a shelter but a regular CBG zone
		else {
			// remove the vehicle
			this.removeFromLane();
			this.removeFromMacroList();
			this.setCurrentCoord(this.destCoord);
			this.endTime = (int) RepastEssentials.GetTickCount();
			this.reachActLocation = false;
			this.reachDest = true;
//			System.out.println(this + " reached dest zone " + destinationZone);
			this.killVehicle();
		}
	}
	
	public void setLastRouteTime(int routeTime) {
		this.lastRouteTime= routeTime;
	}

	public float currentSpeed() {
		return currentSpeed_;
	}

	public void resetVehicle() {
		this.setNextPlan();
		CityContext cityContext = (CityContext) ContextCreator.getCityContext();
		Coordinate currentCoord = this.getCurrentCoord();
		Road road = cityContext.findRoadAtCoordinates(currentCoord, false); // todest=false
		road.addVehicleToNewQueue(this);
		this.nextLane_ = null;
		this.nosingFlag = false;
		this.yieldingFlag = false;
		this.macroLeading_ = null;
		this.macroTrailing_ = null;
		this.leading_ = null;
		this.trailing_ = null;
		this.coordMap = new ArrayList<Coordinate>();
		this.accummulatedDistance_ = 0;
	}

	public void killVehicle() {
		this.road = null;
		this.lane = null;
		this.laneGeography = null;
		this.nextLane_ = null;
		this.nosingFlag = false;
		this.yieldingFlag = false;
		this.macroLeading_ = null;
		this.macroTrailing_ = null;
		this.leading_ = null;
		this.trailing_ = null;
		this.coordMap = null;
		this.currentSpeed_ = 0.0f;
//		this.moveVehicle = false;
		this.destCoord = null;
		this.destZone = null;
		this.targetLane_ = null;
		this.currentCoord_ = null;
//		this.tempLane_ = null;
		this.house = null;
		this.clearShadowImpact(); // ZH: clear any remaining shadow impact
		GlobalVariables.NUM_KILLED_VEHICLES++; //HG: Keep increasing this variable to count the number of vehicles that have reached destination.
//		ContextCreator.getVehicleContext().remove(this); // RV: Remove the vehicle from the quadtree structure
//		if(!GlobalVariables.DISABLE_GEOMETRY){
//			ContextCreator.getVehicleContext().remove(this);
//		}
	}

	/*
	 * ----------------------------------------------------------------------
	 * BL: From this we build functions that are used to incorporate lane
	 * object.
	 * ----------------------------------------------------------------------
	 */
	// BL: In one road find the front leader in the same lane of the road, if
	// there is no front, then return null.
	public Vehicle findFrontBumperLeaderInSameRoad(Lane plane) {
		Vehicle front = this.macroLeading_;
		while (front != null && front.lane != plane) {
			front = front.macroLeading_;
		}
		return front;
	}

	public void removeFromLane() {
		this.lane.removeVehicles();
		if (this.trailing_ != null) {
			this.lane.firstVehicle(this.trailing_);
			this.trailing_.leading(null);
		} else {
			this.lane.firstVehicle(null); 
			this.lane.lastVehicle(null);
		}
	}
	
	
	public void updateLastMoveTick(int current_tick){
		this.lastMoveTick = current_tick;
	}
	
	public int getLastMoveTick(){
		return this.lastMoveTick;
	}

//	public boolean getMoveVehicleFlag() {
//		return this.moveVehicle;
//	}

	// BL: remove a vehicle from the macro vehicle list in the current road
	// segment.
	public void removeFromMacroList() {
		// Current road of this vehicle
		Road pr = this.getRoad();
		// if this is not the first vehicle on the road
		if (this.macroLeading_ != null) {
			this.macroLeading_.macroTrailing_ = this.macroTrailing_;
		} else { // this is the first vehicle on the road
			pr.firstVehicle(this.macroTrailing_);
		}
		if (macroTrailing_ != null) {
			macroTrailing_.macroLeading_ = macroLeading_;
		} else {
			pr.lastVehicle(macroLeading_);
		}
		pr.changeNumberOfVehicles(-1); // LZ: Oct 19, replaced the redundant operations below with this
//		Vehicle pv = pr.firstVehicle();
//		int nVehicles_ = 0;
//		while (pv != null) {
//			nVehicles_++;
//			pv = pv.macroTrailing_;
//		}
//		pr.setNumberOfVehicles(nVehicles_);
	}

	/*
	 * ------------------------------------------------------------------- BL:
	 * Advance a vehicle to the position in macro vehicle list that
	 * corresponding to its current distance. This function is invoked whenever
	 * a vehicle is moved (including moved into a downstream segment), so that
	 * the vehicles in macro vehicle list is always sorted by their position.
	 * The function is called in travel()
	 * -------------------------------------------------------------------
	 */
	/**
	 * @param: distance_ is with respect to the end of road
	 */
	// BL: changed from distance_ to realDistance_ (Jul 27/2012)
	public void advanceInMacroList() {
		Road pr = this.road;
//		while(pr.isLocked()); //LZ: Locked when other thread is modifying the macroTrailing and front, does not help
//		pr.setLock();
		// (0) check if vehicle should be advanced in the list
		if (macroLeading_ == null || this.distance_ >= macroLeading_.distance_) {
			// no macroLeading or the distance to downstream node is greater
			// than marcroLeading
			// no need to advance this vehicle in list
//			pr.releaseLock();
			return;
		}
		// (1) find vehicle's position in the list
		// now this vehicle has a macroLeading that has the higher distance to
		// downstream node.
		// that should not be the vehicle marcroLeading anymore. Need to find
		// new marcroLeading.
		Vehicle front = macroLeading_;
		while (front != null && this.distance_ < front.distance_) {
			front = front.macroLeading_;
		}
		// (2) Take this vehicle out from the list
		// this macroLeading now will be assigned to be macroLeading of this
		// vehicle marcroTrailing
		
		this.macroLeading_.macroTrailing_ = this.macroTrailing_;
		if (this.macroTrailing_ != null) {
			macroTrailing_.macroLeading_ = this.macroLeading_;
		} else {
			pr.lastVehicle(this.macroLeading_);
		}
		// (3) Insert this vehicle after the front
		// (3.1) Point to the front
		this.macroLeading_ = front;
		if (this.macroLeading_ != null) {
			this.macroTrailing_ = macroLeading_.macroTrailing_;
			this.macroLeading_.macroTrailing_ = this;
		} else { 
			this.macroTrailing_ = pr.firstVehicle();
			pr.firstVehicle(this);
		}
		// (3.2) Point to the trailing vehicle
		if (macroTrailing_ != null) {
			macroTrailing_.macroLeading_ = this;
		} else {
			pr.lastVehicle(this);
		}
//		pr.releaseLock();
	}

	/*
	 * function: checkCorrectLane() BL: this function will check if the current
	 * lane connect to a lane in the next road if yes then it gives the
	 * checkLaneFlag true value if not then the checkLaneFlag has false value
	 * the function will be called after the vehicle updates its route i.e. the
	 * routeUpdateFlag has true value
	 */

	public boolean isCorrectLane() {
		// Lane curLane = this.lane;
		if (nextRoad() == null)
			return true;
		Lane nextLane = this.getNextLane();
		// if using dynamic shortest path then we need to check lane only after
		// the route is updated
		this.correctLane = false;
		if (nextLane.getUpLanes().size() > 0)
			for (Lane pl : nextLane.getUpLanes()) {
				if (pl.equals(this.lane)) {
					this.correctLane = true;
					break;
				}
			}
		// if (this.id == GlobalVariables.Global_Vehicle_ID)
		// System.out.println("Am I in correct lane? :" + this.correctLane);
		return this.correctLane;
	}

	// Find if the potential next road and current lane are connected
	public boolean checkNextLaneConnected(Road nextRoad) {
		boolean connected = false;
		Lane curLane = this.lane;
//		Road curRoad = this.getRoad();
		
		if (nextRoad != null) {
			for (Lane dl : curLane.getDnLanes()) {
				if (dl.road_().equals(nextRoad)) {
					// if this lane already connects to downstream road then
					// assign to the connected lane
					connected = true;
					break;
				}
			}
		}
		
		return connected;
	}
	
	
	public void assignNextLane() {
		boolean connected = false;
		Lane curLane = this.lane;
		Road curRoad = this.getRoad();

		if (this.nextRoad() == null) {
			if (this.getVehicleID() == GlobalVariables.Global_Vehicle_ID)
				System.out.println("Assign next lane: current link ID= "
						+ curRoad.getLinkid() + " current lane ID: "
						+ curLane.getLaneid() + " next link ID="
						+ this.nextRoad());
			this.nextLane_=null;
			return;
		} else {
//			Junction curUpJunc = this.road.getJunctions().get(0);
//			Junction curDownJunc = this.road.getJunctions().get(1);
//			Junction nextUpJunc = this.nextRoad_.getJunctions().get(0);
//			Junction nextDownJunc = this.nextRoad_.getJunctions().get(1);
//			if (this.getVehicleID() == GlobalVariables.Global_Vehicle_ID)
//				System.out.println("Assign next lane for Vehicle: "
//						+ this.getVehicleID() + " --current link ID= "
//						+ curRoad.getLinkid() + " --from Node: "
//						+ curUpJunc.getJunctionID() + " --to Node: "
//						+ curDownJunc.getJunctionID() + " --next link ID= "
//						+ this.nextRoad().getLinkid() + " --from Node: "
//						+ nextUpJunc.getJunctionID() + " --to Node: "
//						+ nextDownJunc.getJunctionID());
			for (Lane dl : curLane.getDnLanes()) {
				if (dl.road_().equals(this.nextRoad())) {
					this.nextLane_ = dl;
					// if this lane already connects to downstream road then
					// assign to the connected lane
					connected = true;
					break;
				}
			}
			if (!connected) {
				for (Lane pl : this.nextRoad().getLanes()) {
					for (Lane ul : pl.getUpLanes()) {
						if (ul.road_().getID() == curRoad.getID()) {
							this.nextLane_ = pl;
							break; // assign the next lane to the 1st connected
							// lane
						}
					}
				}
				this.nextLane_ = this.nextRoad().getLane(0);//HG and XQ: force movement at a 5 leg or irregular intersection
			}

			if (this.nextLane_ == null)
				System.err.println("No next lane found for vehicle: "
						+ this.vehicleID_ + " moving on the road: "
						+ this.getRoad().getLinkid() + " lane: "
						+ this.getLane().getLaneid() + " heading to location "
						+ this.getDestinationID()
						+ " while looking for next lane on road: "
						+ this.nextRoad().getLinkid() + " that has "
						+ this.nextRoad().getnLanes() + " lanes");
		}

		// this.updateCoorMapWithNewLane();
	}

	/*
	 * BL: nextLane -> get next lane of a vehicle From the next Road of a
	 * vehicle. Check each lane and see which lane connects to vehicle's current
	 * Road
	 */
	public Lane getNextLane() {
		return this.nextLane_;
	}

	/*
	 * BL: return the target lane (the lane that connect to the downstream Road)
	 */
	public Lane targetLane() {
		Road curRoad = this.getRoad();
		Lane nextLane = this.getNextLane();
		if (nextLane != null) {
			for (Lane pl : nextLane.getUpLanes()) {
				if (pl.road_().equals(curRoad)) {
					this.targetLane_ = pl;
					break;
				}
			}
		}
		return this.targetLane_;
	}

	/*
	 * BL: return the next lane that the vehicle need to change to in order to
	 * reach the correct lane
	 */
	public Lane tempLane() {
		Lane plane = this.targetLane();
		Lane tempLane_ = null;
		if (this.road.getLaneIndex(plane) > this.road.getLaneIndex(this.lane)) {
			tempLane_ = this.rightLane();
		}
		if (this.road.getLaneIndex(plane) < this.road.getLaneIndex(this.lane)) {
			tempLane_ = this.leftLane();
		}
		return tempLane_;
	}

	// get left lane
	public Lane leftLane() {
		Lane leftLane = null;
		if (this.road.getLaneIndex(this.lane) > 0) {
			leftLane = this.road.getLane(this.road.getLaneIndex(this.lane) - 1);
		}
		return leftLane;
	}

	// get right lane
	public Lane rightLane() {
		Lane rightLane = null;
		if (this.road.getLaneIndex(this.lane) < this.road.getnLanes() - 1) {
			rightLane = this.road
					.getLane(this.road.getLaneIndex(this.lane) + 1);
		}
		return rightLane;
	}

	/*
	 * BL: This function change the lane of a vehicle regardless it is MLC or
	 * DLC state. The vehicle change lane when its lead and lag gaps are
	 * acceptable. This will not change the speed of the vehicle, the only
	 * information updated in this function is as follow: remove the vehicle
	 * from old lane and add to new lane. Re-assign the leading and trailing
	 * sequence of the vehicle.
	 */
	public void changeLane(Lane plane, Vehicle leadVehicle, Vehicle lagVehicle) {
		Vehicle curLeading = this.leading();
		Vehicle curTrailing = this.trailing();
		if (curTrailing != null) {
			if (curLeading != null) {
				curLeading.trailing(curTrailing);
				curTrailing.leading(curLeading);
			} else {
				curTrailing.leading(null);
				this.lane.firstVehicle(curTrailing);
			}
		} else if (curLeading != null) {
			this.lane.lastVehicle(curLeading);
			curLeading.trailing(null);
		} else {
			this.lane.firstVehicle(null);
			this.lane.lastVehicle(null);
		}

		this.lane.removeVehicles();
		/*
		 * BL: after change the lane the vehicle updates its leading and
		 * trailing in the target lanes. and also the lead and lag vehicle have
		 * to update its leading and trailing.
		 */
		this.lane = plane;// vehicle move to the target lane
		if (leadVehicle != null) {
			if(this.distance_<leadVehicle.distance_){
				System.out.println("Wow2, " + this.distance_ + "," + leadVehicle.distance_ + "," + this.lane.getLaneid() + "," + this.lane.getLength());
			}
			
			this.leading_ = leadVehicle;
			this.leading_.trailing(this);
			if (lagVehicle != null) {
				if(lagVehicle.distance_<this.distance_){
					System.out.println("Wow3, " + this.distance_ + "," + lagVehicle.distance_ + "," + this.lane.getLaneid() + "," + this.lane.getLength());
				}
				this.trailing_ = lagVehicle;
				this.trailing_.leading(this);
			} else {
				this.trailing(null);
				this.lane.lastVehicle(this);
			}
		} else if (lagVehicle != null) {
			if(lagVehicle.distance_<this.distance_){
				System.out.println("Wow4, " + this.distance_ + "," + lagVehicle.distance_ + "," + this.lane.getLaneid() + "," + this.lane.getLength());
			}
			this.lane.firstVehicle(this);
			this.trailing_ = lagVehicle;
			this.trailing_.leading(this);
		} else {
			this.lane.firstVehicle(this);
			this.lane.lastVehicle(this);
			this.leading(null);
			this.trailing(null);
		}
		this.lane.addVehicles();
		this.updateCoordMap(this.lane); // Update CoordMap
		// this.tempLane_ = null;
	}

	/*
	 * BL: Following we implement mandatory lane changing. The input parameter
	 * is the temporary lane.
	 */
	public void mandatoryLC(Lane plane) {
		Vehicle leadVehicle = this.leadVehicle(plane);
		Vehicle lagVehicle = this.lagVehicle(plane);
		/*
		 * BL: Consider the condition to change the lane as follow: If there are
		 * leading and trailing vehicle then the vehicle will check for gap
		 * acceptance as usual. However, if there is no leading or no trailing,
		 * the leadGap or the lagGap should be neglected. In the case the
		 * vehicle cannot change the lane and the distance to downstream is less
		 * than some threshold then the vehicle starts nosing.
		 */
		if (leadVehicle != null) {
			if (lagVehicle != null) {
				if (this.leadGap(leadVehicle) > this.critLeadGapMLC(leadVehicle)
						&& this.lagGap(lagVehicle) > this.critLagGapMLC(lagVehicle)) {
					// BL: debug the error that vehicle changes the lane
					// internally but not on the GUI
					this.changeLane(this.tempLane(), leadVehicle, lagVehicle);
					this.nosingFlag = false;
				} else if (this.distFraction() < GlobalVariables.critDisFraction) {
					this.nosingFlag = true;
				}
			} else {
				if (this.leadGap(leadVehicle) >= this.critLeadGapMLC(leadVehicle)) {
					this.changeLane(this.tempLane(), leadVehicle, null);
				} else if (this.distFraction() < GlobalVariables.critDisFraction) {
					this.nosingFlag = true;
				}

			}
		} else {
			if (lagVehicle != null) {
				if (this.lagGap(lagVehicle) >= this.critLagGapMLC(lagVehicle)) {
					this.changeLane(this.tempLane(), null, lagVehicle);
				} else if (this.distFraction() < GlobalVariables.critDisFraction) {
					this.nosingFlag = true;
				}
			} else
				this.changeLane(this.tempLane(), null, null);
		}
		// if (this.id == GlobalVariables.Global_Vehicle_ID) {
		// System.out.println("I am in MLC");
		// }
	}

	/*
	 * BL: if the vehicle with MLC state can't change the lane after some
	 * distance. The vehicle need to nose and yield the lag Vehicle of the
	 * target lane in order to have enough gap to change the lane This function
	 * is called only when nosingFlag is true and must be recalled until
	 * nosingFlag receive false value after the vehicle nosed, tag the lag
	 * vehicle in target lane to yielding status. This function will be called
	 * in makeAccelerationDecision
	 */
	public double nosing() //
	{
		double acc = 0;
		Lane tarLane = this.tempLane();
		Vehicle leadVehicle = this.leadVehicle(tarLane);
		Vehicle lagVehicle = this.lagVehicle(tarLane);
		/*
		 * BL: if there is a lag vehicle in the target lane, the vehicle will
		 * yield that lag vehicle however, the yielding is only true if the
		 * distance is less than some threshold
		 */
		if (lagVehicle != null) {
			if (this.lagGap(lagVehicle) < GlobalVariables.minLag) {
				this.yieldingFlag = true; // LZ: Nov 3, based on the description, should not be lagVehicle.yieldingFlag = true
			}
			// System.out.println("Vehicle " + this.id + " has lag gap " +
			// lagGap
			// + " with lag vehicle " + lagVehicle.id);
		}
		Vehicle front = this.leading();
		/*
		 * BL: 1. if there is a lead and a lag vehicle in the target lane. the
		 * vehicle will check the lead gap before decide to decelerate. if the
		 * lead gap is large, then the subject vehicle will be assigned with the
		 * accelerate rate as in car following. 2. if there is no lead vehicle
		 * in the target lane. the subject vehicle will max accelerate.
		 */
		if (leadVehicle != null) {
			if (this.leadGap(leadVehicle) < this.critLeadGapMLC(leadVehicle)) {
				if (this.currentSpeed_ > 12.2f) {
					acc = -1.47f;// meters/sec^2
				} else if (this.currentSpeed_ > 6.1f)
					acc = -2.04f;
				else
					acc = -2.4f;
			} else {
				if (front != null)
					acc = this.calcCarFollowingRate(front);
				else
					acc = this.maxAcceleration_;
			}
		} else {
			if (front != null)
				acc = this.calcCarFollowingRate(front);
			else
				acc = this.maxAcceleration_;
		}
		this.nosingFlag = false;
		// if (this.distFraction() < 0.25 && this.onlane)
		// lagVehicle.yieldingFlag = true;
		if (Double.isNaN(acc)) {
			System.err.println("acc is NaN for " + this);
		}
		return acc;
	}

	/*
	 * BL: While moving, the vehicle will checks if the vehicles in adjection
	 * lanes are nosing to its lane or not after some distance to the downstream
	 * node If the nosing is true then it will be tagged in yielding state to
	 * slow down.
	 */

	public float yielding() {
		float acc = 0;
		if (this.currentSpeed_ > 24.3f)
			acc = -2.44f;
		else if (this.currentSpeed_ > 18.3f)
			acc = -2.6f;
		else if (this.currentSpeed_ > 12.2f)
			acc = -2.74f;
		else if (this.currentSpeed_ > 6.1f)
			acc = -2.9f;
		else
			acc = -3.05f;
		this.yieldingFlag = false;
		return acc;
	}

	/*
	 * BL: when change lane, distance need to be adjusted with the lane width.
	 */

	// Calculate critical lead gap of the vehicle with the lead vehicle in the
	// target lane.
	public double critLeadGapMLC(Vehicle leadVehicle) {
		double critLead = 0;
		double minLead_ = GlobalVariables.minLead;
		double betaLead01 = GlobalVariables.betaLeadMLC01;
		double betaLead02 = GlobalVariables.betaLeadMLC02;
		double gama = GlobalVariables.gama;
		if (leadVehicle != null)
			critLead = minLead_
					+ (betaLead01 * this.currentSpeed() + betaLead02
							* (this.currentSpeed() - leadVehicle.currentSpeed()))
					* (1 - Math.exp(-gama * this.distance()));
		if (critLead < minLead_)
			critLead = minLead_;
		return critLead;
	}

	// BL: Calculate lead gap of the vehicle with the lead vehicle in the target
	// lane.
	public double leadGap(Vehicle leadVehicle) {
		double leadGap = 0;
		if (leadVehicle != null){
			leadGap = this.distance() - leadVehicle.distance() - leadVehicle.length(); // leadGap>=-leadVehicle.length()
		}
		else{
			leadGap = this.distance(); // LZ: Nov 3, change this.distance() to 9999999
		}
		return leadGap;
	}

	// BL: Calculate critical lag gap of the vehicle with the lag vehicle in the
	// target lane.
	public double critLagGapMLC(Vehicle lagVehicle) {
		double critLag = 0;
		double betaLag01 = GlobalVariables.betaLagMLC01;
		double betaLag02 = GlobalVariables.betaLagMLC02;
		double gama = GlobalVariables.gama;
		double minLag_ = GlobalVariables.minLag;
		if (lagVehicle != null) {
			critLag = minLag_
					+ (betaLag01 * this.currentSpeed() + betaLag02
							* (this.currentSpeed() - lagVehicle.currentSpeed()))
					* (1 - Math.exp(-gama * this.distance()));
		}
		if (critLag < minLag_)
			critLag = minLag_;
		return critLag;
	}

	// BL: Calculate lag gap of the vehicle with the lag vehicle in the target
	// lane.
	public double lagGap(Vehicle lagVehicle) {
		double lagGap = 0;
		if (lagVehicle != null)
			lagGap = lagVehicle.distance() - this.distance() - this.length();
		else
			lagGap = this.lane.getLength() - this.distance() - this.length(); // Nov 3, still need to -this.length()
		return lagGap;
	}

	// BL: find the lead vehicle in target lane
	public Vehicle leadVehicle(Lane plane) {
		Vehicle leadVehicle = this.macroLeading_;
		while (leadVehicle != null && leadVehicle.lane != plane) {
			leadVehicle = leadVehicle.macroLeading_;
		}
		return leadVehicle;
	}

	// BL: find lag vehicle in target lane
	public Vehicle lagVehicle(Lane plane) {
		Vehicle lagVehicle = this.macroTrailing_;
		while (lagVehicle != null && lagVehicle.lane != plane) {
			lagVehicle = lagVehicle.macroTrailing_;
		}
		return lagVehicle;
	}

	/*
	 * BL: Following we will implement discretionary LC model At current stage,
	 * the DLC is implementing as follow: 1. If the vehicle is not close to
	 * downstream node 2. and it finds a correct lane with better traffic
	 * condition -> then it will change lane
	 */
	/*
	 * BL: if the vehicle is in correct lane then we find a better lane that is
	 * also connected to downstream line this function is called at the
	 * makeLaneChangingDecision
	 */
	public Lane findBetterLane() {
		Lane curLane = this.lane;
		Lane targetLane = null;
		Lane rightLane = this.rightLane();
		Lane leftLane = this.leftLane();
		// If left and right lane exist then check if they are both connect to
		// next lane or not
		if (this.equals(curLane.firstVehicle())) {
			return null;
		} else {
			if (leftLane != null && rightLane != null) {
				Lane tempLane = leftLane.betterLane(rightLane);
				targetLane = curLane.betterLane(tempLane); // get the lane that
				// has best traffic condition
			} else if (leftLane != null)
				targetLane = curLane.betterLane(leftLane);
			else if (rightLane != null) {
				targetLane = curLane.betterLane(rightLane);
			}
			// if we have a target lane, then compare the speed of
			// front bumper leader in the lane with current leader
			if (targetLane != null && !targetLane.equals(curLane)) {
				Vehicle front = this
						.findFrontBumperLeaderInSameRoad(targetLane);
				if (front == null) {
					return targetLane;
					// TODO: fix the null for leading vehicle
				} else if (this.leading_ != null
						&& this.leading_.currentSpeed_ < this.desiredSpeed_
						&& this.currentSpeed_ < this.desiredSpeed_) {
					if (front.currentSpeed_ > this.currentSpeed_
							&& front.accRate_ > 0)
						return targetLane;
				}
			}
			return null;
		}
	}

	public Lane findBetterCorrectLane() {
		Lane curLane = this.lane;
		Lane targetLane = null;
		Lane rightLane = this.rightLane();
		Lane leftLane = this.leftLane();
		// If left and right lane exist then check if they are both connect to
		// next lane or not
		if (this.equals(curLane.firstVehicle())) {
			return null;
		} else {
			if (leftLane != null && rightLane != null) {
				// if both left and right lanes are connected to downstream lane
				if (leftLane.isConnectToLane(this.nextLane_)
						&& rightLane.isConnectToLane(this.nextLane_)) {
					Lane tempLane = leftLane.betterLane(rightLane);
					targetLane = curLane.betterLane(tempLane); // get the lane that
					// has best traffic condition
				}
				// if only left lane connects to downstream lane
				else if (leftLane.isConnectToLane(this.nextLane_)) {
					targetLane = curLane.betterLane(leftLane);
				}
				// if only right lane connects to downstream lane
				else if (rightLane.isConnectToLane(this.nextLane_)) {
					targetLane = curLane.betterLane(rightLane);
				}
			} else if (leftLane != null
					&& leftLane.isConnectToLane(this.nextLane_))
				targetLane = curLane.betterLane(leftLane);
			else if (rightLane != null
					&& rightLane.isConnectToLane(this.nextLane_)) {
				targetLane = curLane.betterLane(rightLane);
			}
			// if we have a target lane, then compare the speed of
			// front bumper leader in the lane with current leader
			if (targetLane != null && !targetLane.equals(curLane)) {
				Vehicle front = this
						.findFrontBumperLeaderInSameRoad(targetLane);
				if (front == null) {
					return targetLane;
				} else if (this.leading_ != null
						&& this.leading_.currentSpeed_ < this.desiredSpeed_
						&& this.currentSpeed_ < this.desiredSpeed_) {
					if (front.currentSpeed_ > this.currentSpeed_
							&& front.accRate_ > 0)
						return targetLane;
					
				}
			}
			return null;
		}

	}

	// once the vehicle finds a better lane. It changes to that lane
	// discretionarily.
	public void discretionaryLC(Lane plane) {
		Vehicle leadVehicle = this.leadVehicle(plane);
		Vehicle lagVehicle = this.lagVehicle(plane);
		double leadGap = this.leadGap(leadVehicle);
		double lagGap = this.lagGap(lagVehicle);
		double critLead = this.criticalLeadDLC(leadVehicle);
		double critLag = this.criticalLagDLC(lagVehicle);
		if (leadGap > critLead && lagGap > critLag) {
			this.changeLane(plane, leadVehicle, lagVehicle);
		}
	}

	public double criticalLeadDLC(Vehicle pv) {
		double critLead = 0;
		double minLead = GlobalVariables.minLeadDLC;
		// TODO: change betaLead01 and 02 to DLC value.
		if (pv != null) {
			critLead = minLead + GlobalVariables.betaLeadDLC01
					* this.currentSpeed_ + GlobalVariables.betaLeadDLC02
					* (this.currentSpeed_ - pv.currentSpeed_);
		}
		critLead = Math.max(minLead, critLead);
		return critLead;
	}

	public double criticalLagDLC(Vehicle pv) {
		double critLag = 0;
		double minLag = GlobalVariables.minLagDLC;
		// TODO: change betaLag01 and 02 to DLC value
		if (pv != null) {
			critLag = minLag + GlobalVariables.betaLagDLC01
					* this.currentSpeed_ + GlobalVariables.betaLagDLC02
					* (this.currentSpeed_ - pv.currentSpeed_);
		}
		critLag = Math.max(minLag, critLag);
		return critLag;
	}

	/*
	 * After appending vehicle to next road, a new nextlane being assigned. The
	 * coordinate map will be updated with the last point is the starting point
	 * of the next lane
	 */
	public void updateCoorMapWithNewLane() {
		Coordinate coor1, coor2;
		if (!this.coordMap.isEmpty()) {
			int end = this.coordMap.size() - 1;
			coor1 = this.coordMap.get(end);
			if (this.vehicleID_ == GlobalVariables.Global_Vehicle_ID
					&& !GlobalVariables.Debug_On_Road) {
				System.out
						.println("=====I already had my coordinate map but need to update for moving through junction ");
				System.out.println("which connects to the point: " + coor1);
			}
		} else {
			// SH Temp
			// Geography<Vehicle> vehicleGeography;
			// vehicleGeography = ContextCreator.getVehicleGeography();
			coor1 = this.getCurrentCoord();
		}
		if (this.nextLane_ != null) {
			Lane plane = this.nextLane_;
			Coordinate c1, c2;
			Coordinate[] coords = laneGeography.getGeometry(plane)
					.getCoordinates();
			c1 = coords[0];
			c2 = coords[coords.length - 1];
			coor2 = getNearestCoordinate(coor1, c1, c2);
			if (this.vehicleID_ == GlobalVariables.Global_Vehicle_ID
					&& !GlobalVariables.Debug_On_Road) {
				System.out.println("The adding coordinate is from the lane: "
						+ plane.getLaneid());
			}
		} else
			coor2 = this.destCoord;
		if (!coor2.equals(coor1)) {
			this.coordMap.add(coor2);
			if (this.vehicleID_ == GlobalVariables.Global_Vehicle_ID
					&& !GlobalVariables.Debug_On_Road) {
				System.out
						.println("@@@@@@@@@@@@@@ New coordinate added @@@@@@@@@@@@@@@@@@@");
			}
		} else {
			if (this.vehicleID_ == GlobalVariables.Global_Vehicle_ID
					&& !GlobalVariables.Debug_On_Road) {
				System.out.println("+++++++++++No coordinate added+++++++++ ");
			}
		}
		if (this.id == GlobalVariables.Global_Vehicle_ID
				&& !GlobalVariables.Debug_On_Road) {
			System.out
					.println("My coordinate map after update the next lane: ");
			int end = this.coordMap.size() - 1;
			System.out.println("Distance to added point: "
					+ distance(this.coordMap.get(end - 1),
							this.coordMap.get(end)));
		}
	}

	/*
	 * BL: when vehicle approach junction, need new coordinate and distance
	 */
	public int appendToJunction(Lane nextlane) {
//		coordMap.clear();
//		if (this.getVehicleID() == GlobalVariables.Global_Vehicle_ID
//				&& this.nextRoad().getLinkid() == this.destRoadID) {
//			System.out.println("Vehicle " + this.getVehicleID()
//					+ " is appending to a junction of the final Road");
//		}
		if (this.atDestination()) {
//			this.removeFromLane();
//			this.removeFromMacroList();
			return 0;
		} else{ // LZ: want to change to next lane
//			if (this.getVehicleID() == GlobalVariables.Global_Vehicle_ID
//					&& this.nextRoad().getLinkid() == this.destRoadID)
//				System.out.println("Vehicle " + this.getVehicleID()
//						+ " has final next lane " + nextlane.getLaneid());
//			Coordinate[] coords = laneGeography.getGeometry(nextlane)
//					.getCoordinates();
//			Coordinate start = coords[0];
//			Coordinate end = coords[coords.length - 1];
//			Coordinate coor = this.getNearestCoordinate(lastCoordinate, start, end);
			coordMap.clear();
			coordMap.add(this.getCurrentCoord());
		}

		//this.distance_ = 0; // (float) distance(lastCoordinate, coordMap.get(0)); // LZ: End of the link
				//- lastStepMove_ / 2;
		this.onlane = false;
		
//		if (this.distance_ <= 0) {
		if (this.changeRoad() == 0){
			return 0;
		}
			
//		}
		
		return 1;
	}

	public boolean isOnLane() {
		return onlane;
	}

	public double entranceGap(Lane nextlane) {
		double gap = 0;
		if (nextlane != null) {
			Vehicle newleader = nextlane.lastVehicle();
			if (newleader != null) {
				gap = nextlane.getLength() - newleader.distance_ - newleader.length();
			} else
				gap = 9999999;
		}
		return gap;
	}

	/**
	 * @param c
	 * @param c1
	 * @param c2
	 * @return
	 */
	private Coordinate getNearestCoordinate(Coordinate c, Coordinate c1,
			Coordinate c2) {
		/*
		 * GeometryFactory geomFac = new GeometryFactory(); Geometry coordGeom =
		 * geomFac.createPoint(c); Geometry geom1 = geomFac.createPoint(c1);
		 * Geometry geom2 = geomFac.createPoint(c2);
		 */
		double dist1 = distance(c, c1);
		double dist2 = distance(c, c2);

		if (dist1 < dist2)
			return c1;
		else
			return c2;

	}

	private double distance(Coordinate c1, Coordinate c2) {
		// GeodeticCalculator calculator = new GeodeticCalculator(ContextCreator
		// .getRoadGeography().getCRS());
		calculator.setStartingGeographicPoint(c1.x, c1.y);
		calculator.setDestinationGeographicPoint(c2.x, c2.y);
		double distance;
		try {
			distance = calculator.getOrthodromicDistance();
		} catch (AssertionError ex) {
			System.err.println("Error with finding distance");
			distance = 0.0;
		}
		if (Double.isNaN(distance)) {
			System.err.println("distance is NaN in Vehicle.distance() for " + this);
		}
		return distance;
	}

//	private double distance(Coordinate c1, Coordinate c2, double[] returnVals) {
//		calculator.setStartingGeographicPoint(c1.x, c1.y);
//		calculator.setDestinationGeographicPoint(c2.x, c2.y);
//		double distance;
//		try {
//			distance = calculator.getOrthodromicDistance();
//
//		} catch (AssertionError ex) {
//			System.err.println("Error with finding distance");
//			distance = 0.0;
//		}
//		if (returnVals != null && returnVals.length == 2) {
//			returnVals[0] = distance;
//			double angle = Math.toRadians(calculator.getAzimuth()); // Angle in
//			// range -PI to PI
//			// Need to transform azimuth
//			// (in range -180 -> 180 and where 0 points north)
//			// to standard mathematical (range 0 -> 360 and 90 points north)
//			if (angle > 0 && angle < 0.5 * Math.PI) { // NE Quadrant
//				angle = 0.5 * Math.PI - angle;
//			} else if (angle >= 0.5 * Math.PI) { // SE Quadrant
//				angle = (-angle) + 2.5 * Math.PI;
//			} else if (angle < 0 && angle > -0.5 * Math.PI) { // NW Quadrant
//				angle = (-1 * angle) + 0.5 * Math.PI;
//			} else { // SW Quadrant
//				angle = -angle + 0.5 * Math.PI;
//			}
//			returnVals[1] = angle;
//		}
//		return distance;
//	}
	
	/**
	 * 
	 * @param c1 current coordinate
	 * @param c2 next coordinate
	 * @param returnVals a mutable 
	 * @return
	 */
	private double distance2(Coordinate c1, Coordinate c2, double[] returnVals) {
		double distance;
//		double min_dx_dy = GlobalVariables.MIN_DX_DY_PER_TURN;
//		Coordinate c1Copy = new Coordinate(c1.x, c1.y);
//		Coordinate c2Copy = new Coordinate(c2.x, c2.y);
//		double dx = c2Copy.x - c1Copy.x;
//		double dy = c2Copy.y - c1Copy.y;
//		if (Math.abs(dx) < min_dx_dy) {
//			c1Copy.x = c2Copy.x;
//		}
//		if (Math.abs(dy) < min_dx_dy) {
//			c1Copy.y = c2Copy.y;
//		}
//		try {
//			calculator.setStartingGeographicPoint(c1Copy.x, c1Copy.y);
//			calculator.setStartingGeographicPoint(c1Copy.x, c1Copy.y);
//			calculator.setDestinationGeographicPoint(c2Copy.x, c2Copy.y);
//		} catch (Exception e) {
//			System.out.println("Coordinate format error in " + this);
//		}
		double radius;
//		double min_dx_dy = GlobalVariables.MIN_DX_DY_PER_TURN;
//		Coordinate c1Copy = new Coordinate(c1.x, c1.y);
//		Coordinate c2Copy = new Coordinate(c2.x, c2.y);
//		double dx = c2Copy.x - c1Copy.x;
//		double dy = c2Copy.y - c1Copy.y;
//		if (Math.abs(dx) < min_dx_dy) {
//			c1Copy.x = c2Copy.x;
//		}
//		if (Math.abs(dy) < min_dx_dy) {
//			c1Copy.y = c2Copy.y;
//		}
		calculator.setStartingGeographicPoint(c1.x, c1.y);
		calculator.setDestinationGeographicPoint(c2.x, c2.y);
		try {
			distance = calculator.getOrthodromicDistance();
			radius = calculator.getAzimuth(); // the azimuth in degree, value from -180-180
		} catch (AssertionError e) {
			System.err.println("Error with finding distance: " + e);
			distance = 0.0;
			radius = 0.0;
		}
		if (returnVals != null && returnVals.length == 2) {
			returnVals[0] = distance;
			returnVals[1] = radius;
		}
		if (Double.isNaN(distance)) {
			// RV: Check if this condition ever occurs
			System.err.println("Geodetic distance is NaN for " + this);
			distance = 0.0;
			radius = 0.0;
		}
		if (distance == 0.0) {
//			System.out.println("For some reason the distance is zero in distance2() for " + this);
		}
		return distance;
	}
	
	/**
	 * A light weight move function based on moveVehicleByVector, and is much faster than the one using geom 
	 * @param coord
	 * @param distance
	 * @param angleInDegrees
	 */
	private void move2(Coordinate coord, double distance, double angleInDegrees){
		this.calculator.setStartingGeographicPoint(coord.x, coord.y);
		this.calculator.setDirection(angleInDegrees, distance);
		Point2D p = this.calculator.getDestinationGeographicPoint();
		if(p!=null){
			this.setCurrentCoord(new Coordinate(p.getX(),p.getY()));
		}
		else{
			System.out.println("Cannot move vehicle from " + coord.x + "," + coord.y +" by " + distance + "," + angleInDegrees);
		}
	}
	
	/**
	 * Thread safe version of the moveByVector, replace the one in the DefaultGeography class
	 * Creating a new Geometry point given the current location of the vehicle as well as the distance and angle.
	 * In the end, move the vehicle to the new geometry point.
	 * @return the new Geometry point
	 */
//	public void moveVehicleByVector(double distance, double angleInRadians) {
//		Geometry geom = ContextCreator.getVehicleGeography().getGeometry(this);
//		if (geom == null) {
//			System.err.println("Error moving object by vector");
//		}
//
//		if (angleInRadians > 2 * Math.PI || angleInRadians < 0) {
//			throw new IllegalArgumentException(
//					"Direction cannot be > PI (360) or less than 0");
//		}
//		double angleInDegrees = Math.toDegrees(angleInRadians);
//		angleInDegrees = angleInDegrees % 360;
//		angleInDegrees = 360 - angleInDegrees;
//		angleInDegrees = angleInDegrees + 90;
//		angleInDegrees = angleInDegrees % 360;
//		if (angleInDegrees > 180) {
//			angleInDegrees = angleInDegrees - 360;
//		}
//		Coordinate coord = geom.getCoordinate();
//		AffineTransform transform;
//		
//		try {
//			if (!ContextCreator.getVehicleGeography().getCRS().equals(DefaultGeographicCRS.WGS84)) {
//				// System.out.println("Here 1");
//				MathTransform crsTrans = CRS.findMathTransform(ContextCreator.getVehicleGeography().getCRS(),
//						DefaultGeographicCRS.WGS84);
//				Coordinate tmp = new Coordinate();
//				JTS.transform(coord, tmp, crsTrans);
//				this.calculator.setStartingGeographicPoint(tmp.x, tmp.y);
//			} else {
//				// System.out.println("Here 2");
//				this.calculator.setStartingGeographicPoint(coord.x, coord.y);
//			}
//			this.calculator.setDirection(angleInDegrees, distance);
//			Point2D p = this.calculator.getDestinationGeographicPoint();
//			if (!ContextCreator.getVehicleGeography().getCRS().equals(DefaultGeographicCRS.WGS84)) {
//				MathTransform crsTrans = CRS.findMathTransform(
//						DefaultGeographicCRS.WGS84, ContextCreator.getVehicleGeography().getCRS());
//				JTS.transform(new Coordinate(p.getX(), p.getY()), coord,
//						crsTrans);
//			}
//
//			transform = AffineTransform.getTranslateInstance(
//					p.getX() - coord.x, p.getY() - coord.y);
//
//			MathTransform mt = mtFactory
//					.createAffineTransform(new GeneralMatrix(transform));
//			geom = JTS.transform(geom, mt);
//		} catch (Exception ex) {
//			System.err.println("Error moving object by vector");
//		}
////		this.setCurrentCoord(geom.getCoordinate());
//		ContextCreator.getVehicleGeography().move(this, geom);
//
//		try {
//		    Coordinate geomCoord = geom.getCoordinate();
//		  //HG: the following condition can be put to reduce the data when the output of interest is the final case when vehicles reach close to destination
////		    if(this.nextRoad() == null){
//		    	DataCollector.getInstance().recordSnapshot(this, geomCoord);
////		    }
//		}
//		catch (Throwable t) {
//		    // Could not record this vehicle move in the data buffer!
//		    DataCollector.printDebug("ERR", t.getMessage());
//		}
//	}

	public int getVehicleClass() {
		return vehicleClass;
	}

	public void setVehicleClass(int vehicleClass) {
		this.vehicleClass = vehicleClass;
	}
	
	/** Rajat, Xue: Assign the indifference band (eta) as found in Mahmassani & 
	 * Jayakrishnan (1991) [doi.org/10.1016/0191-2607(91)90145-G].
	 * It is assumed to be distributed as an isosceles triangle,
	 * centered at the mean eta and having base width 0.5*eta.
	 * @return the eta value
	 * */
	public double assignIndiffBand(){
		double mean = GlobalVariables.ETA;
		double base = 0.5 * mean; // base width of the triangle
		double start = mean - 0.5 * base; // starting point of the distribution
		double end = mean + 0.5 * base; // end point ""
		double rand = GlobalVariables.RandomGenerator.nextDouble();
		double indiffbandvalue;
		if (rand <= 0.5) {
			indiffbandvalue = start + base * Math.sqrt(0.5 * rand);
		} else {
			indiffbandvalue = end - base * Math.sqrt(0.5 * (1 - rand));
		}
		return indiffbandvalue;	
	}
	
	/** LZ,RV:DynaDestTest: find next shelter destination */
	public void findNextShelter(Zone curDest) throws Exception {
		// get the current plans
		ArrayList<Plan> plans = house.getActivityPlan();
		
		// if the shelter has enough capacity for this vehicle
		// (currently only supports one person per vehicle)
		if (curDest.receiveEvacuee() == true) {
			// modify the departure time of the last plan
			int curTime = (int) RepastEssentials.GetTickCount();
			plans.get(plans.size() - 1).setDuration(curTime);
			
			// remove it from the simulator
			Coordinate target = this.destCoord;
			this.removeFromLane();
			this.removeFromMacroList();
			this.setCurrentCoord(target);
			this.endTime = (int) RepastEssentials.GetTickCount();
			this.reachActLocation = false;
			this.reachDest = true;
			System.out.println(this + " reached dest shelter " + curDest);
			this.killVehicle();
		}
		// else if current shelter is not available, reroute this to next shelter
		else {
			// reset the location & geometry
			this.removeFromLane(); // Remove from Lane
			this.removeFromMacroList(); // Remove from Road
			Coordinate target = this.destCoord;
			this.setCurrentCoord(target);
			CityContext cityContext = (CityContext) ContextCreator.getCityContext();
			Coordinate currentCoord = this.getCurrentCoord();
			Road road = cityContext.findRoadAtCoordinates(currentCoord, false); // Can this be null?
			this.setRoad(road);
			
			// mark this shelter visited
			visitedShelters.put(curDest.getIntegerId(), (int) RepastEssentials.GetTickCount());
			
			// if 3rd shelter routing strategy (SO matching) is used,
			// update the number of people waiting at this shelter & return
			if (GlobalVariables.DYNAMIC_DEST_STRATEGY == 3) {
				curDest.addWaiting(this);
				return;
			}
			
			// find the next shelter
			Zone nextShelter = ContextCreator.getCityContext().getClosestShelter(this);
			
			// process if next shelter is not null
			if (nextShelter != null) {
				// create & activate the new plan
				setNewHouse(nextShelter.getIntegerId());
//				this.resetVehicle();
			}
			// if all shelters have rejected this vehicle, kill it
			else {
				System.out.println("Veh#" + id + ": All shelters exhausted; killing self");
				this.killVehicle();
			}
		}
	}
	
	/** RV:DynaDestTest: Set new plan for a vehicle by creating a new house */ 
	public void setNewHouse(int nextDestID) {
		// create the inputs for setting up a new house
		ArrayList<Integer> locations = new ArrayList<Integer>();
		ArrayList<Integer> durations = new ArrayList<Integer>();
		int curDestID = getDestinationID();
		locations.add(curDestID);
		locations.add(nextDestID);
		durations.add(0);
		durations.add(0);

		// set up a new house
		House new_house = new House(getVehicleID(), getDestinationID());
		new_house.setActivityPlan(locations, durations);
		this.setHouse(new_house);
		
		int tick = (int) RepastEssentials.GetTickCount();
		System.out.println(String.format("Rerouting %s [%d->%d] at t=%d: %s",
				this, locations.get(0), locations.get(1), tick, visitedShelters));
		this.resetVehicle();
	}
	
	/** LZ,RV:DynaDestTest: Additional getters required for dynamic destination */
	public Coordinate getCoord() {
		return this.getCurrentCoord();
	}

	public HashMap<Integer, Integer> getVisitedShelters() {
		return visitedShelters;
	}
	
	public int getRegime() {
		return regime_;
	}
	
	@Override // RV:DynaDestTest: Mainly for debugging
	public String toString() {
		return "<Veh" + this.vehicleID_ + ">";
	}
	
	public boolean isAtOrigin(){ //LZ: whether this vehicle is at origin
		return this.atOrigin;
	}
}
