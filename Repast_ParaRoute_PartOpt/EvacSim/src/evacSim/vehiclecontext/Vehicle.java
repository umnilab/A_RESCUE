package evacSim.vehiclecontext;

import com.vividsolutions.jts.geom.Coordinate;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.Iterator;
import java.util.List;
import java.util.Map;
import java.util.Queue;

import org.opengis.referencing.operation.MathTransformFactory;
import org.apache.log4j.Logger;
import org.geotools.referencing.GeodeticCalculator;
import org.geotools.referencing.ReferencingFactoryFinder;
import java.io.IOException;
import java.lang.Math;

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

public class Vehicle {
	/** Automatically generated instance ID. */
	private int id;
	protected int vehicleID_;
	private int deptime;
	private int endTime;
	private int destinationZoneId;
	protected int destRoadID;
	/** The time of getting the last routing information */
	protected int lastRouteTime;
	private Coordinate originalCoord;
	protected Coordinate destCoord;
	/** HG: This variable stores the coordinates of the vehicle when last time
	 * vehicle snapshot was recorded for visualization interpolation. */
	private Coordinate previousEpochCoord;
	/** LZ: this variable is created when the vehicle is initialized and used
	 * to store vehicle location when it does not enter the network. */
	private Coordinate currentCoord_;
	private float length;
	/** Distance from downstream junction */
	private float distance_;
	/** Distance from the start point of next line segment */
	private float nextDistance_;
	
	/**
	 * Car following model state parameters
	 * (speed and acceleration measures in SI units).
	 */
	private float currentSpeed_;
	private float accRate_;
	private float bearing_;
	private float desiredSpeed_;
	private  int regime_;
	protected float maxAcceleration_;
	private float normalDeceleration_;
	protected float maxDeceleration_;
	/** assuming normal dec is applied */
	private float distanceToNormalStop_; 
	private float lastStepMove_;
	public float accummulatedDistance_;
	private float travelPerTurn;
	private boolean reachDest;
	private boolean reachActLocation;
	/** LZ: A better solution to avoid double modification in single tick */
	private int lastMoveTick = -1;
	private boolean onlane = false;
	protected boolean atOrigin = true;
	private House house;
	protected Road road;
	protected Road nextRoad_;
	private Lane lane;
	private Lane nextLane_;
	protected Zone destZone; // RMSA
	// RV: Track vehicles that have stopped for a long time
	private int lastPositiveMovedTick = 0;

	/** 
	 * Zhan: For vehicle based routing
	 * The route is always started with the current road, whenever entering
	 * the next road, the current road will be popped out
	 */
	protected Queue<Road> roadPath; 
	private List<Coordinate> coordMap;
	private Geography<Lane> laneGeography;
	/** Leading vehicle in the lane */
	private Vehicle leading_;
	/** Trailing vehicle in the lane */
	private Vehicle trailing_;
	/** BL: Leading vehicle on the road (all lanes combined) */
	private Vehicle macroLeading_;
	/** BL: Trailing vehicle on the road (all lanes combined) */
	private Vehicle macroTrailing_; 
	
	/** 
	 * BL: Variables for lane changing model
	 * */
	/** Correct lane that vehicle should change to. */
	private Lane targetLane_;
	/** To check if the vehicle is in the correct lane. */
	private boolean correctLane;
	/** If a vehicle in MLC and it can't find gap acceptance then nosing is true. */
	private boolean nosingFlag;
	/** The vehicle need to yield if true. */
	private boolean yieldingFlag;

	GeodeticCalculator calculator = new GeodeticCalculator(ContextCreator
			.getLaneGeography().getCRS());
	MathTransformFactory mtFactory = ReferencingFactoryFinder.getMathTransformFactory(null);

	/** 
	 * For adaptive network partitioning
	 * */
	/** Number of current shadow roads in the path */
	private int Nshadow;
	/** List of roads that will be used in future; can be slow. */
	private ArrayList<Road> futureRoutingRoad; // 

	/**
	 * HG: For distinguishing between different classes.
	 * */
	private int vehicleClass;
	/** JX: Travel time stored for the last time the route was updated. */
	protected float travelTimeForPreviousRoute;
	/** JX: The tick when the last routing was updated. */
	protected int previousTick;
	/** RV,JX: The relative difference threshold of route time for the old 
	 * route and new route. */
	protected float indiffBand;
	/** LZ,RV: List of visited shelters along with the time of visit */
	protected HashMap<Integer, Integer> visitedShelters;
	/** LZ: Number of times the vehicle get stuck */
	protected int stuck_time = 0;
	/** LZ: Whether the vehicle has been moved */
	protected boolean movingFlag = false;
	/** LZ: Record when and where the vehicle enters each link, and store it 
	 * whenever the vehicle is arrived */
	protected ArrayList<Integer> linkHistory;
	protected ArrayList<Integer> linkTimeHistory;
	
	private Logger logger = ContextCreator.logger;

	public Vehicle(House h) {
		this.id = ContextCreator.generateAgentID();
		this.house = h;
		this.currentCoord_ = new Coordinate();
		this.length = GlobalVariables.DEFAULT_VEHICLE_LENGTH;
		this.travelPerTurn = GlobalVariables.TRAVEL_PER_TURN;
		this.maxAcceleration_ = GlobalVariables.MAX_ACCELERATION;
		this.maxDeceleration_ = GlobalVariables.MAX_DECELERATION;
		this.normalDeceleration_ = -0.5f;
		this.previousEpochCoord = new Coordinate();
		this.endTime = 0;
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
		this.setNextPlan();
		// for adaptive network partitioning
		this.Nshadow = 0;
		this.futureRoutingRoad = new ArrayList<Road>();
		this.setVehicleClass(1);
		// Xue: For the first step, set the travel time for the previous route to be infinite.
		this.travelTimeForPreviousRoute = Float.MAX_VALUE;
		// Xue: The previous tick is set to be 0 at the beginning.
		this.previousTick = 0;
		// Xue: Generate the parameter of route selection from a distribution.
		this.indiffBand = assignIndiffBand();
		// RV: List of visited shelters along with the time of visit
		this.visitedShelters = new HashMap<Integer, Integer>();
		Plan startPlan = house.getActivityPlan().get(0);
		this.visitedShelters.put(startPlan.getLocation(), startPlan.getDuration());
		this.linkHistory = new ArrayList<Integer>();
		this.linkTimeHistory = new ArrayList<Integer>();
		GlobalVariables.NUM_GENERATED_VEHICLES++;
	}

	/**
	 * HG: This is a new subclass of Vehicle class that has some different 
	 * parameters like max acceleration and max deceleration
	 */ 
	public Vehicle(House h, float maximumAcceleration, float maximumDeceleration) {
		this(h);
		this.maxAcceleration_ = maximumAcceleration;
		this.maxDeceleration_ = maximumDeceleration;
		this.setVehicleClass(-1);
	}

	/* Getters ------------------------------------------------------------- */

	public int getId() {
		return id;
	}

	public House getHouse() {
		return house;
	}

	public int getDepTime() {
		return deptime;
	}

	public int getEndTime() {
		return endTime;
	}

	public Road getRoad() {
		return road;
	}

	public Lane getLane() {
		return lane;
	}

	public int getVehicleID() {
		return vehicleID_;
	}

	public int getDestinationID() {
		return destZone.getIntegerId();
	}

	public int getLastMoveTick(){
		return lastMoveTick;
	}
	
	public HashMap<Integer, Integer> getVisitedShelters() {
		return visitedShelters;
	}

	public int getRegime() {
		return regime_;
	}

	public Coordinate getOriginalCoord() {
		return originalCoord;
	}

	public Coordinate getDestCoord() {
		return this.destCoord;
	}

	public Coordinate getCurrentCoord() {
		Coordinate coord = new Coordinate();
		if (currentCoord_ == null)
			throw new NullPointerException(this+".coordinate = null");
		coord.x = this.currentCoord_.x;
		coord.y = this.currentCoord_.y;
		coord.z = this.currentCoord_.z;
		return coord;
	}

	public Coordinate getPreviousEpochCoord() {
		return this.previousEpochCoord;
	}

	public int getVehicleClass() {
		return vehicleClass;
	}

	public boolean isAtOrigin(){
		return this.atOrigin;
	}

	public float getBearing() {
		return bearing_;
	}

	public Road nextRoad() {
		return nextRoad_;
	}

	public float length() {
		return length;
	}

	public float distance() {
		return distance_;
	}

	public float currentSpeed() {
		return currentSpeed_;
	}

	public boolean isOnLane() {
		return onlane;
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

	public boolean atActivityLocation() {
		return reachActLocation;
	}

	public boolean atDestination() {
		return reachDest;
	}

	public boolean getMovingFlag(){
		return this.movingFlag;
	}
	
	public float maxAcceleration() {
		return maxAcceleration_;
	}

	/** BL: nextLane -> get next lane of a vehicle From the next Road of a veh.
	 * Check each lane and see which lane connects to vehicle's current road. */
	public Lane getNextLane() {
		return this.nextLane_;
	}


	/** BL: Get the target lane (that connects to the downstream Road). */
	public Lane targetLane() {
		Road curRoad = this.getRoad();
		Lane nextLane = this.getNextLane();
		if (nextLane != null) {
			for (Lane pl : nextLane.getUpLanes()) {
				if (pl.getRoad().equals(curRoad)) {
					this.targetLane_ = pl;
					break;
				}
			}
		}
		return this.targetLane_;
	}


	/** BL: Get the next lane that the vehicle needs to change to in order to
	 * reach the correct lane. */
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

	/** Get left lane */

	public Lane leftLane() {
		Lane leftLane = null;
		if (this.road.getLaneIndex(this.lane) > 0) {
			leftLane = this.road.getLane(this.road.getLaneIndex(this.lane) - 1);
		}
		return leftLane;
	}

	/** Get right lane */

	public Lane rightLane() {
		Lane rightLane = null;
		if (this.road.getLaneIndex(this.lane) < this.road.getnLanes() - 1) {
			rightLane = this.road
					.getLane(this.road.getLaneIndex(this.lane) + 1);
		}
		return rightLane;
	}


	/* Setters ------------------------------------------------------------- */
	/* Setters ------------------------------------------------------------- */


	public void setId(int id) {
		this.id = id;
	}


	public void setHouse(House h){
		// LZ: reset house to update plan
		this.house = h;
	}


	public void setDepTime(int time) {
		this.deptime = time;
	}


	public void setRoad(Road road) {
		this.road = road;
		this.currentSpeed_ = (float) this.road.getFreeSpeed();
	}


	public void setLastRouteTime(int routeTime) {
		this.lastRouteTime= routeTime;
	}


	public void setBearing(float bearing_) {
		this.bearing_ = bearing_;
	}
	
	public void setCurrentCoord(Coordinate coord) {
		if (coord == null) 
			logger.warn("Setting null coordinate on " + this);
		this.currentCoord_.x = coord.x;
		this.currentCoord_.y = coord.y;
		this.currentCoord_.z = coord.z;
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
			logger.info("helloooo");
		}
		this.destCoord = this.destZone.getCoord();
		this.originalCoord = cityContext.findHouseWithDestID(
				current.getLocation()).getCoord();
		this.destRoadID = cityContext.findRoadAtCoordinates(this.destCoord,
				true).getLinkid();
		this.atOrigin = true;
	}
	
	public int getLastPositiveMovedTick() {
		return lastPositiveMovedTick;
	}
	
	private void updateLastMovedTick() {
		lastPositiveMovedTick = (int) RepastEssentials.GetTickCount();
	}
	
	private void setMovingFlag(boolean flag) {
		this.movingFlag = flag;
		// if the vehicle has now started moving, update its last moved tick
		if (movingFlag == true)
			updateLastMovedTick();
	}
	
	private void accumulateDistanceTraveled(float dist) {
		this.accummulatedDistance_ += dist;
		GlobalVariables.TOT_DIST_TRAVELED += dist;
	}

	/** Set shadow vehicles and future routing road. */

	public void setShadowImpact() {
		this.Nshadow = GlobalVariables.N_SHADOW;
		if (this.roadPath.size() < this.Nshadow)
			this.Nshadow = this.roadPath.size();
		if (this.Nshadow > 0) {
			// count actual number of Nshadow vehicles added
			int shadowCount = 1;
			// cumulative TT for Nshadow allocation
			double cumlativeTT_Nshadow = 0.0;
			double cumulativeTT = 0.0;
			// future routing road count: number of road found in shadow roads
			int foundFutureRoutingRoad = 0;
			Iterator<Road> itr = this.roadPath.iterator();
			for (int i=0; i < this.Nshadow; i++) {
				Road r = itr.next();
				// increase the shadow vehicle count: include current road
				if (i < 1) {
					// current vehicle will always be added by default;
					// set the shadow vehicle count
					r.incrementShadowVehicleNum();
				} else {
					if (cumlativeTT_Nshadow <= GlobalVariables.
							SIMULATION_PARTITION_REFRESH_INTERVAL *
							GlobalVariables.SIMULATION_STEP_SIZE) {
						// set the shadow vehicle count
						r.incrementShadowVehicleNum();
						cumlativeTT_Nshadow += r.getTravelTime();
						shadowCount += 1;
					}
				}
				cumulativeTT += r.getTravelTime();
				// found the road with cumulative TT greater than than network
				// refresh interval, use it as the future routing road
				if (foundFutureRoutingRoad < GlobalVariables.PART_REFRESH_MULTIPLIER) {
					if (cumulativeTT >= GlobalVariables
							.SIMULATION_NETWORK_REFRESH_INTERVAL *
							GlobalVariables.SIMULATION_STEP_SIZE) {
						this.futureRoutingRoad.add(r);
						r.incrementFutureRoutingVehNum();
						// update the future routing road count
						foundFutureRoutingRoad += 1;
						// reset the cumulative TT
						cumulativeTT = 0.0;
					}
				}
			}
			// reset the Nshadow count
			this.Nshadow = shadowCount;
		} else {
			this.Nshadow = 0;
		}
	}


	public void setNextRoad() {
		try {
			if (!this.atOrigin) { // Not at origin
				// Arrive the destination link
				//LZ: 2020-06-07. Test, two variables lead to inconsistency
				if (this.road.getLinkid() == this.destRoadID || this.roadPath.size() == 2) {
					this.roadPath.remove();
					this.nextRoad_ = null;
					return;
				}
				if (this.roadPath.size() == 1) {
					this.nextRoad_ = null;
					return;
				}
				// if the travel times have changed on the road network since 
				// the last time this vehicle was rerouted, try to reroute it
				if (this.lastRouteTime < RouteV.getValidTime()) {
					// get the new (shortest) path & its travel time
					/* JX: 2019-10-05. change the return type of RouteV.vehicleRoute 
					 * to be a single-entry hashmap valued by best (shortest) path 
					 * computed as of now & keyed by its travel time */
					Map<Float, Queue<Road>> newPathMap;
					try {
						newPathMap = RouteV.vehicleRoute(this, this.destZone);
					} catch (NullPointerException e) {
						logger.error("Impossible routing due to previously greedy rerouting in "
								+ this + ".changeRoad() after long stuckTime.");
						this.nextRoad_ = null;
						GlobalVariables.NUM_FAILED_VEHICLES++;
						return;
					}
					Queue<Road> newPath = newPathMap.values().iterator().next();
					float newTravTime = newPathMap.keySet().iterator().next();
					// get time difference since last routing
					int currentTick = (int) RepastEssentials.GetTickCount();
					float timeSinceLastRerouting = (currentTick - this.lastRouteTime)
							* GlobalVariables.SIMULATION_STEP_SIZE;
					// get current path's remaining travel time
					float curTravTime = this.travelTimeForPreviousRoute - timeSinceLastRerouting;
					// time difference of current remaining path with best path
					float travTimeDiff = curTravTime - newTravTime;
					/* JX, RV: if the difference in current path's travel time 
					 * with the new shortest path is above a certain threshold, 
					 * update the path of the vehicle to the (new) shortest path, 
					 * otherwise not. This threshold is chosen according to a 
					 * fixed minimum gap and the vehicle's indifference band. 
					 * Reference: doi.org/10.1016/0191-2607(91)90145-G */
					if (travTimeDiff <= this.indiffBand * 
							curTravTime && travTimeDiff > GlobalVariables.TAU) {
						newPath = this.roadPath; // keep the old path
						newTravTime = curTravTime;
					}
					// remove the current road from the chosen path
					Road curRoad = newPath.remove();
					// and remove its shadow impact
					this.removeShadowCount(curRoad);
					// there is a problem if chosen path minus current road is 
					// empty, since that is only possible when $this is on 
					// destination road, which is already accounted for above
					if (newPath.size() == 0) {
						System.err.println("Empty new path for " + this + 
								" after removing current road " + curRoad);
						this.nextRoad_ = null;
						GlobalVariables.NUM_FAILED_VEHICLES++;
						return;
					}
//					if (this.checkNextLaneConnected(nextRoad)){
					// set the next road
					this.clearShadowImpact(); // clear legacy impact
					// set the path (with the current road removed)
					this.roadPath = newPath;
					// update the next road
					Iterator<Road> iter = this.roadPath.iterator();
					// skip the road $this is just entering
					iter.next();
					this.nextRoad_ = iter.hasNext() ? iter.next() : null;
					if (nextRoad_ == this.road)
						throw new IllegalStateException();
					this.setShadowImpact(); // set new path's impact
					this.lastRouteTime = currentTick;
					this.travelTimeForPreviousRoute = curTravTime;
				} else { // if the network travel times have not updated
					// remove the current road from the path & its shadow impact
					Road curRoad = this.roadPath.remove();
					this.removeShadowCount(curRoad);
					// set the next road
					Iterator<Road> iter = this.roadPath.iterator();
					iter.next();
					this.nextRoad_ = iter.hasNext() ? iter.next() : null;
					if (nextRoad_ == this.road)
						throw new IllegalStateException();
				}	
			} else { // if the vehicle is at origin
				this.clearShadowImpact(); // clear legacy impact
				// compute the route & its travel time
				Map<Float, Queue<Road>> newPathMap = RouteV.vehicleRoute(this, this.destZone);
				Queue<Road> newPath = newPathMap.values().iterator().next();
				float newTravTime = newPathMap.keySet().iterator().next();
				// set this path
				this.roadPath = newPath;
				this.setShadowImpact(); // set new legacy impact
				// remove this vehicle from origin
				this.atOrigin = false;
				// set the last routing time to now
				this.lastRouteTime = (int) RepastEssentials.GetTickCount();
				// if travel time is positive, set the next road
				if (newTravTime > 0) {
					Iterator<Road> iter = roadPath.iterator();
					// skip the current road (but do not remove it)
					iter.next();
					// set the next road if it exists
					this.nextRoad_ = iter.hasNext() ? iter.next() : null;
					if (nextRoad_ == this.road)
						throw new IllegalStateException();
				}
			}
		} catch (Exception e) {
			e.printStackTrace();
			logger.error("No next road found for Vehicle "
					+ this.vehicleID_ + " on Road " + this.road.getLinkid());
			GlobalVariables.NUM_FAILED_VEHICLES += 1;
			// LZ: Remove the vehicle, can we do something better?
			this.nextRoad_ = null; 
		}
	}


	public void setCoordMap(Lane plane) {
		if (plane != null) {
			coordMap.clear();
			for (Coordinate coord : plane.getCoordinates()) {
				this.coordMap.add(coord);
			}
			//LZ: update the vehicle location to be the first pt in the coordMap
			this.setCurrentCoord(this.coordMap.get(0));
			this.coordMap.remove(0);
			//LZ: lastStepMove_ does note make sense, should be this.length/2
			this.distance_ = (float) plane.getLength();// - lastStepMove_ / 2;

			float[] distAndAngle = new float[2];
			// LZ: replace previous vehicle movement function
			// the first element is the distance, and the second is the radius
			this.distance2(this.getCurrentCoord(), this.coordMap.get(0), distAndAngle);
			this.nextDistance_ = distAndAngle[0];
			this.setBearing(distAndAngle[1]);
		} 
		else {
			logger.error("There is no target lane to set!");
		}
		if (Double.isNaN(distance_)) {
			logger.error("distance_ is NaN in setCoordMap for " + this);
		}
	}


	public void setReachDest() throws Exception {
		// get the current zone (i.e., destination zone prior to relocation)
		Zone destinationZone = ContextCreator.getCityContext().findHouseWithDestID(
				this.getDestinationID());
		// if the destination is a shelter
		if (destinationZone.getType() == 1) {
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
			// LZ: Log the vehicle information here
			String formated_msg = (this.getVehicleID() + 
					";" + 0 +
					";" + this.getDepTime() +
					";" + this.getEndTime() +
					";" + this.getHouse().getZoneId() +
					";" + this.getDestinationID() +
					";" + this.accummulatedDistance_ +
					";" + this.visitedShelters.size() +
					";" + this.linkHistory.toString() +
					";" + this.linkTimeHistory.toString()
					);
			try {
				ContextCreator.bw.write(formated_msg);
				ContextCreator.bw.newLine();
			} catch (IOException e) {
				e.printStackTrace();
			}
			this.killVehicle();
		}
	}


	public void setOriginalCoord(Coordinate coord) {
		this.originalCoord = coord;
	}

	private void setPreviousEpochCoord(Coordinate newCoord){
		this.previousEpochCoord.x = newCoord.x;
		this.previousEpochCoord.y = newCoord.y;
	}

	/**
	 * RV: Set new plan for a vehicle by creating a new house
	 */ 
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
		logger.info(
				"Rerouting" + this +
				"from " + locations.get(0) +
				" to " + locations.get(1) +
				" at t=" + tick +
				": " + visitedShelters);
		this.resetVehicle();
	}
	
	public void updateLastMoveTick(int current_tick) {
		this.lastMoveTick = current_tick;
	}

	/* Others -------------------------------------------------------------- */

	/* Others -------------------------------------------------------------- */

	/** SH: This function enters the vehicles into the network. */
	public int enterNetwork(Road road) {
		Lane firstlane = road.firstLane();
		float gap = entranceGap(firstlane);
		int tickcount = (int) RepastEssentials.GetTickCount(); 
		if (gap >= 1.2*this.length() && tickcount > firstlane.getLastEnterTick()) {
			// LZ: Update the last enter tick for this lane
			firstlane.updateLastEnterTick(tickcount);
			this.updateLastMoveTick(tickcount);
			// calculate the initial speed
			// to be consistent with changeRoad, use free speed
			float capspd = road.getFreeSpeed();
			currentSpeed_ = capspd; // have to change later
			desiredSpeed_ = this.road.getFreeSpeed(); // initial value is 0.0, added Oct 2
			this.road.removeVehicleFromNewQueue(this);
			this.setRoad(road);
			this.setCoordMap(firstlane);
			this.appendToLane(firstlane);
			this.appendToRoad(this.road);
			// Record the vehicle movement here
			this.linkHistory.add(this.road.getLinkid());
			this.linkTimeHistory.add(tickcount);
			this.setNextRoad();
			this.assignNextLane();
			GlobalVariables.NUM_VEHICLES_ENTERED_ROAD_NETWORK++;
			return 1;
		}
		return 0;
	}

	
	/**
	 * Clear the legacy impact from the shadow vehicles and future routing 
	 * vehicles. Performed before next routing computation.
	 */
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

	
	/**
	 * Remove shadow vehicle count after the vehicle leaves the road.
	 */
	public void removeShadowCount(Road r) {
		if (this.Nshadow > 0) {
			r.decreaseShadowVehicleNum();
			this.Nshadow--;
		}
		// remove the future routing road impact
		if (this.futureRoutingRoad.contains(r)){
			r.decreaseFutureRoutingVehNum();
			this.futureRoutingRoad.remove(r);
		}
	}

	
	/**
	 * BL: Append a vehicle to vehicle list in plane.
	 */
	public void appendToLane(Lane plane) {
		this.lane = plane;
		Vehicle v = plane.getLastVehicle();
		plane.increaseNumVehicles();
		if (v != null && v != this) {
			this.leading(v);
			v.trailing(this);
		} else {
			plane.setFirstVehicle(this);
			this.leading(null);
		}
		this.trailing(null);
		plane.setLastVehicle(this);
		this.onlane = true;
	}

	
	/**
	 * Update the coordinates of vehicle to the lane coordinate.
	 */
	private void updateCoordMap(Lane lane) {
		// list of control points of given lane
		Coordinate[] coords = lane.getCoordinates();

		// LZ: This does not work properly, replace with new implementation
		//		Coordinate juncCoordinate, nextLaneCoordinate, closeVehCoordinate;
		//		juncCoordinate = coords[coords.length - 1]; //The last coordinate of the lane
		coordMap.clear();
		float accDist = lane.getLength();

		for (int i = 0; i < coords.length - 1; i++) {
			accDist-=distance(coords[i], coords[i+1]);
			if (this.distance_ >= accDist) { // Find the first pt (coords[i+1]) in CoordMap that has smaller distance;
				float[] distAndAngle = new float[2];
				distance2(coords[i], coords[i+1], distAndAngle);
				move2(coords[i], coords[i+1], distAndAngle[0], distAndAngle[0] - (this.distance_- accDist)); // Update vehicle location
				this.nextDistance_ = this.distance_- accDist;
				this.setBearing(distAndAngle[1]);
				for (int j = i+1; j < coords.length; j++){ // Add the rest coords into the CoordMap
					coordMap.add(coords[j]);
				}
				break;
			}
		}
		if (coordMap.size() == 0) { // perhaps when $this is too close to a junction
			float[] distAndAngle = new float[2];
			distance2(coords[coords.length-2], coords[coords.length-1], distAndAngle);
			move2(coords[coords.length-2], coords[coords.length-1], distAndAngle[0], distAndAngle[0] - this.distance_); // Update vehicle location
			this.nextDistance_ = this.distance_;
			this.setBearing(distAndAngle[1]);
			coordMap.add(coords[coords.length-1]);
		}
	}

	
	public boolean calcState() {
		// SH: right now there is only one function we may also invoke lane
		// changing decision here
		if(this.road == null || this==null){
			return false;
		}
		this.makeAcceleratingDecision();
		if (this.road.getnLanes() > 1 &&
				this.onlane &&
				this.distance_ >= GlobalVariables.NO_LANECHANGING_LENGTH
				) { 
			this.makeLaneChangingDecision();
		}
		return true;
	}

	
	/**
	 * The car-following model calculates the acceleration rate based on
	 * interaction with other vehicles. The function returns a the most
	 * restrictive acceleration (deceleration if negative) rate among the
	 * rates given by several constraints. This function updates accRate_ at
	 * the end.
	 */
	public void makeAcceleratingDecision() {
		Vehicle front = this.vehicleAhead();
		float aZ = this.accRate_; /* car-following */
		float acc = this.maxAcceleration(); /* returned rate */
		// BL: vehicle will have acceleration rate based on car following if it
		// is not in yielding or nosing state
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
			logger.error("Vehicle.makeAcceleratingDecision: " + 
					"acc=NaN for "+this);
		}
	}


	public float calcCarFollowingRate(Vehicle front) {
		if (this == front)
			throw new IllegalArgumentException(this+".calcCarFollowingRate(): this = frontVehicle");
		// SH-if there is no front vehicle the car will be in free flow regime
		// and have max acceleration
		if (front == null) {
			this.regime_ = GlobalVariables.STATUS_REGIME_FREEFLOWING;
			return (this.maxAcceleration_);
		}
		float acc;
		float space = gapDistance(front);
		float speed = currentSpeed_ == 0f ? 0.00001f : currentSpeed_;
		float headway = 2.0f * space / (speed + currentSpeed_);
		float hupper, hlower;
		float AlphaDec = GlobalVariables.ALPHA_DEC;
		float BetaDec = GlobalVariables.BETA_DEC;
		float GammaDec = GlobalVariables.GAMMA_DEC;
		float AlphaAcc = GlobalVariables.ALPHA_ACC;
		float BetaAcc = GlobalVariables.BETA_ACC;
		float GammaAcc = GlobalVariables.GAMMA_ACC;
		hupper = GlobalVariables.H_UPPER;
		hlower = GlobalVariables.H_LOWER;

		/* There will be three regimes emergency/free-flow/car-following regime
		 * depending on headway */
		// 1. emergency regime
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
					float v = front.currentSpeed_ + front.accRate_ * dt;
					space += 0.5f * (front.currentSpeed_ + v) * dt;
					acc = brakeToTargetSpeed(space, v);
				}
			}
			acc = Math.min(normalDeceleration_, acc);
			regime_ = GlobalVariables.STATUS_REGIME_EMERGENCY;
		}
		// 2. free-flow regime
		else if (headway > hupper) { // desired speed model will do
			if (space > distanceToNormalStop_) {
				acc = maxAcceleration_;
				regime_ = GlobalVariables.STATUS_REGIME_FREEFLOWING;
			} else {
				float dt = GlobalVariables.SIMULATION_STEP_SIZE;
				float v = front.currentSpeed_ + front.accRate_ * dt;
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
			logger.error("Vehicle.calcCarFollowingRate(): "+
					" acc=NaN for " + this);
		}
		return acc;
	}


	public float brakeToTargetSpeed(float space, float v) {
		if (space > GlobalVariables.FLT_EPSILON) {
			float v2 = v * v;
			float u2 = currentSpeed_ * currentSpeed_;
			float acc = (v2 - u2) / space * 0.5f;
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
			if (leading_ == this)
				throw new IllegalStateException(
						this+".vehicleAhead(): this.leading = this");
			return leading_;
		} else if (nextLane_ != null) {
			if (nextLane_.getLastVehicle() != null) {
				Vehicle front = nextLane_.getLastVehicle();
				if (front == this)
					throw new IllegalStateException(
							this+".vehicleAhead(): this.nextLane.last = this");
				return front;
			} else
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


	public float gapDistance(Vehicle front) {
		float headwayDistance;
		if (front != null) { // front = vehicle ahead	
			if (front.lane == null){ // front vehicle is in the intersection
				headwayDistance = this.distance_ - front.length();
			}
			else if (this.lane.getID() == front.lane.getID()) {// if front vehicle is on the same lane as $\this
				headwayDistance = this.distance_ - front.distance_ - front.length();
				// if front vehicle is on different lane
			} else {
				// LZ: front vehicle is in the next road
				headwayDistance = this.distance_
						+ front.lane.getLength()
						- front.distance(); //-front.length();
			}
		} // if no vehicle ahead
		else {
			headwayDistance = Float.MAX_VALUE;
		}
		if (Double.isNaN(headwayDistance))
			logger.error(this+".gapDistance(): headway=NaN");
		if (headwayDistance < -0.001)
			logger.error(this+".gapDistance(): headway is negative");
		return (headwayDistance);
	}


	public void makeLaneChangingDecision() {
		if (this.distFraction() < 0.5) { 
			// Halfway to the downstream intersection,
			// only mantatory LC allowed, check the correct lane
			if (this.isCorrectLane() != true) { // change lane if not in correct
				// lane
				Lane plane = this.tempLane();
				if (plane != null)
					this.mandatoryLC(plane);
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
			// $this is at beginning of the lane, it is free to change lane
			Lane tarLane = this.findBetterLane();
			if (tarLane != null) {
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
					logger.info("Vehicle " + this.getId()
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
					logger.info("Vehicle " + this.getId()
					+ "has no lane to change");
				}
			}
		}
	}


	/**
	 * HG: Record the vehicle snapshot if this tick corresponds to the
	 * required epoch that is needed for visualization interpolation.
	 * Note that this is recording is independent of snapshots of vehicles 
	 * whether they move or not in the current tick. 
	 * (So when vehicles do not move in a tick but we need to record positions
	 * for visualization interpolation then recVehSnaphotForVisInterp is useful). 
	 * Also, we update the coordinates of the previous epoch in the end of the
	 * function.
	 */ 
	public void recVehSnaphotForVisInterp() {
		Coordinate currentCoord = this.getCurrentCoord();
		if (currentCoord != null) {
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



	/**
	 * Calculate new location and speed after an iteration based on its current
	 * location, speed and acceleration. The vehicle will be removed from the
	 * network if it arrives its destination.
	 */
	public void travel() {
		this.endTime++;
		try {
			if (!this.reachDest && !this.reachActLocation) {
				// move the vehicle towards their destination
				this.move();
				// BL: if the vehicle travel too fast,
				// it will change the marcroList of the road.
				this.advanceInMacroList();
				// up to this point this.reachDest == false;
				if (this.nextRoad() == null) {
					this.checkAtDestination();
				}
			}
		} catch (Exception e) {
			try { // print the error-causing vehicle during move()
				logger.error("Vehicle " + this.getVehicleID()
				+ " had an error while travelling on road: "
				+ this.road.getLinkid() + "with next road: "
				+ this.nextRoad().getLinkid());
				e.printStackTrace();
				RunEnvironment.getInstance().pauseRun();
			}
			catch (NullPointerException exc) { // LZ,RV: in case next road is null
				logger.error("Vehicle.travel(): " + this
						+ " had an error while travelling on road: "
						+ road.getLinkid() + " with next road: ");
				e.printStackTrace();
				RunEnvironment.getInstance().pauseRun();
			}
		}
	}


	public void move() {
		// validation checks
		if (distance_ < 0 || Double.isNaN(distance_))
			logger.error("Vehicle.move(): distance_="+distance_+" "+this);
		if (currentSpeed_ < 0 || Float.isNaN(currentSpeed_))
			logger.error("Vehicle.move(): currentSpeed_="+currentSpeed_+" "+this);
		if (Double.isNaN(accRate_))
			logger.error("Vehicle.move(): accRate_="+accRate_+" "+this);

		// intialization
//		Coordinate currentCoord = null;
		Coordinate target = null;
		float dx = 0; // LZ: travel distance calculated by physics
		boolean travelledMaxDist = false; // true when traveled with maximum distance (=dx).
		float distTravelled = 0; // the distance traveled so far.
		float oldv = currentSpeed_; // speed at the beginning
		float step = GlobalVariables.SIMULATION_STEP_SIZE; // 0.3 s
		float minSpeed = GlobalVariables.SPEED_EPSILON; // min allowed speed (m/s)
		float minAcc = GlobalVariables.ACC_EPSILON; // min allowed acceleration (m/s2)
		float maxSpeed = this.road.getFreeSpeed();
		
		// check if this vehicle's movingFlag has been false for a long time (>2000 ticks)
		int curTick = (int) RepastEssentials.GetTickCount();
		if (lastPositiveMovedTick - curTick > 2000) {
			System.out.println(this + " stuck at " + this.currentCoord_ + " for " + 
					(lastPositiveMovedTick - curTick) + " ticks!");
		}
		
		// LZ: Check if this is close to intersection or destination.
		if (distance_ < GlobalVariables.INTERSECTION_BUFFER_LENGTH) { 
			//			System.out.println("Vehicles2 " + this.distance_ + " " + this.nextDistance_ + " " + dx);
			if (this.nextRoad() != null) { // this has not reached destination
				if (this.isOnLane()) { // this is still on the road
					this.coordMap.add(this.getCurrentCoord()); // stop and wait
					// make this.isOnLane become false
					int canEnterNextRoad = this.appendToJunction(nextLane_); 
					if (canEnterNextRoad == 0) { // cannot enter next road
						this.lastStepMove_ = 0;
						this.setMovingFlag(false);
					} else { // successfully entered the next road
						// update the lastStepMove and accumulatedDistance
						this.lastStepMove_ = distance_;
						accumulateDistanceTraveled(this.lastStepMove_);
						this.setMovingFlag(true);
					}
					return; // move finished
				} else { // not on lane, directly changing road
					if (this.changeRoad() == 0) {
						// 0 means the vehicle cannot enter the next road
						stuck_time += 1;
						this.lastStepMove_ = 0;
						this.setMovingFlag(false);
					} else {
						// successfully entered the next road, update the
						// lastStepMove and accumulatedDistance
						stuck_time = 0;
						this.lastStepMove_ = distance_;
						accumulateDistanceTraveled(this.lastStepMove_);
						this.setMovingFlag(true);
					}
				}
			}
			return; // move finished; this has reached destination
		}

		double dv = accRate_ * step; // Change of speed
		if (dv > -currentSpeed_) { // still moving at the end of the cycle
			dx = (float) (currentSpeed_ * step + 0.5f * dv * step);

		} else { // stops before the cycle end
			dx = -0.5f * currentSpeed_ * currentSpeed_ / accRate_;
			if (currentSpeed_ < minSpeed && accRate_ < minAcc) {
				dx = 0.0f;
			}
		}
		if (Double.isNaN(dx)) {
			logger.info("dx is NaN in move() for " + this);
		}

		// solve the crash problem by making sure 0 <= dx <= gap with its front vehicle
		float gap = gapDistance(this.vehicleAhead());
		dx = Math.max(0, Math.min(dx, gap));

		// actual acceleration rate applied in last time interval.
		accRate_ = (float) (2.0f * (dx - oldv * step) / (step * step));
		// update speed
		currentSpeed_ += accRate_ * step;
		if (currentSpeed_ < minSpeed) {
			currentSpeed_ = 0.0f;
			accRate_ = 0.0f; // no back up allowed
		} else if (currentSpeed_ > maxSpeed && accRate_ > minAcc) {
			currentSpeed_ = (float) maxSpeed;
			accRate_ = (float) ((currentSpeed_ - oldv) / step);
		}
		if (dx < 0.0f) { // negative dx is not allowed
			lastStepMove_ = 0;
			this.setMovingFlag(false);
			return;
		}

		/* Check if the vehicle's dx is under some threshold, i.e. it
		 * will move to the next road
		 * 1. Search for the junction
		 * 2. Search for road id
		 * 3. Find the first vehicle of all the lanes of those roads.
		 */
		float[] distAndAngle = new float[2];
		while (!travelledMaxDist) {
			//			currentCoord = this.getCurrentCoord(); // current location
			target = this.coordMap.get(0);
			// LZ: replace previous vehicle movement function
			// the first element is the distance, and the second is the radius
			//			double distToTarget = this.distance2(currentCoord, target, distAndAngle);

			/* if the distance $this can travel (dx) exceeds the distance (distToTarget)
			 * to the next control point on the route (target), then directly move it to
			 * the target point
			 */
			if (distTravelled + nextDistance_ <= dx) {
				distTravelled += nextDistance_;
				this.setCurrentCoord(target);
				/* LZ: Oct 31, the distance and calculated value is not consistent
				 * (Vehicle reached the end of the link does not mean Vehicle
				 * .distance_ <= 0), therefore, use a intersection buffer（see
				 * the changeRoad block above) */
				this.coordMap.remove(0);
				if (this.coordMap.isEmpty()) { // this is close to intersection
					this.distance_ -= nextDistance_;
					this.nextDistance_ = 0;
					//					System.out.println("Vehicles " + this.distance_ + " " + this.nextDistance_ + " " + dx);
					if (this.nextRoad() != null) { // has a next road
						if (this.isOnLane()) { // is still on the road
							this.coordMap.add(this.getCurrentCoord()); // stop and wait
							if (this.appendToJunction(nextLane_) == 0) {
								stuck_time+=1;
							} else {
								stuck_time = 0;
							}
							lastStepMove_ = distTravelled;
							accumulateDistanceTraveled(distTravelled);
							break;
						} else {
							if (this.changeRoad() ==0 ){
								stuck_time+=1;
							} else {
								stuck_time=0;
							}
							lastStepMove_ = distTravelled;
							accumulateDistanceTraveled(distTravelled);
							break;
						}
					} else { // no next road, the vehicle arrived at the destination
						this.coordMap.clear();
						this.coordMap.add(this.currentCoord_);
						break;
					}
				} else {
					Coordinate currentCoord = this.getCurrentCoord();
					this.distance2(currentCoord, this.coordMap.get(0), distAndAngle);
					this.distance_ -= this.nextDistance_;
					this.nextDistance_ = distAndAngle[0];
					this.setBearing(distAndAngle[1]);
				}
			}
			/* otherwise, the distance to travel does not require changing the control
			 * point and thus, use linear interpolation to decide the final coordinates
			 * within the same link segment
			 * (http://forum.java.sun.com/thread.jspa?threadID=438608&messageID=1973655)
			 * LZ: replaced the complicated operation with a equivalent but simpler one
			 */
			else {
				// move by distance in the calculated direction
				float distanceToTarget = this.nextDistance_;
				this.distance_ -= dx - distTravelled;
				this.nextDistance_ -= dx - distTravelled;
				Coordinate currentCoord = this.getCurrentCoord();
				move2(currentCoord, this.coordMap.get(0), distanceToTarget, dx-distTravelled);
				distTravelled = dx;
				accumulateDistanceTraveled(dx);
				lastStepMove_ = dx;
				travelledMaxDist = true;
			}
		}
		// reduce the distance to junction by the amount moved
		//		distance_ -= distTravelled;
		if (distTravelled > 0) {
			this.setMovingFlag(true);
		} else {
			this.setMovingFlag(false);
		}
		// make sure the distance_ to junction is not negative
		if (distance_ < 0) {
			distance_ = 0;
		}
		return;
	}


	public void printGlobalVehicle(float dx) {
		if (this.vehicleID_ == GlobalVariables.GLOBAL_VEHICLE_ID) {
			logger.info("Next Road ID for vhielc: " + this.vehicleID_
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
		currentCoord = this.getCurrentCoord();
		// The first list of coordinates for the vehicle to follow
		if (this.coordMap.size() > 0) {
			target = this.coordMap.get(0);
		} else {
			lane = this.road.firstLane();
			Coordinate[] coords = laneGeography.getGeometry(lane).getCoordinates();
			for (Coordinate coord: coords){
				this.coordMap.add(coord);
			}
			//this.setCoordMap(lane);
			target = this.coordMap.get(0);
		}

		float[] distAndAngle = new float[2];
		float distToTarget;
		distToTarget = this.distance2(currentCoord, target, distAndAngle);

		if (distToTarget <= travelPerTurn) { // Include equal, which is important
			// this.lock.lock();
			// vehicleGeography.move(this, targetGeom);
			this.setCurrentCoord(target);
		}
		// Otherwise move as far as we can towards the target along the road
		// we're on
		// Get the angle between the two points (current and target)
		// (http://forum.java.sun.com/thread.jspa?threadID=438608&messageID=1973655)
		else {
			float distToTravel = travelPerTurn;
			accumulateDistanceTraveled(distToTravel);
			// LZ: implementation 3: drop the geom class and change the coordinates
			move2(currentCoord, target, distToTarget, distToTravel);
		}
		this.updateLastMovedTick();
		return;
	}


	public int changeRoad() {
		// check if the vehicle has reached the destination or not
		if (this.atDestination()) {
			// ZH: Clear shadow impact if already reaches destination
			this.clearShadowImpact();
			// only one will reach destination once
			return 0;
		} else if (this.nextRoad_ != null) {
			// BL: check if there is enough space in the next road to change to
			int tickcount = (int) RepastEssentials.GetTickCount(); 
			// LZ: Nov 4, short lane and the vehicle can move freely
			// check if the target long road has space
			if ((this.entranceGap(nextLane_) >= 1.2*this.length())
					&& (tickcount > this.nextLane_.getLastEnterTick())) {
				//LZ: update enter tick so other vehicle cannot enter this road in this tick
				this.nextLane_.updateLastEnterTick(tickcount);
				this.removeFromLane();
				this.removeFromMacroList();
				this.setCoordMap(nextLane_);
				this.appendToRoad(this.nextRoad_);
				this.linkHistory.add(this.nextRoad().getLinkid());
				this.linkTimeHistory.add(tickcount);
				this.appendToLane(nextLane_); // LZ: Two vehicles entered the same lane, then messed up.
				this.setNextRoad();
				this.assignNextLane();
				// Reset the desired speed according to the new road
				this.desiredSpeed_ =  this.road.getFreeSpeed();
				// HG: Need to update current speed according to the new free speed 
				// LZ: Use current speed instead of the free speed, be
				// consistent with the setting in enteringNetwork
				if(this.currentSpeed_ > this.road.getFreeSpeed())
					this.currentSpeed_ = this.road.getFreeSpeed();
				return 1;
			}
			// this vehicle has been stuck for more than 5 minutes, if both of the followings does not work, then failed vehicle number +=1
			else if (this.stuck_time > GlobalVariables.MAX_STUCK_TIME) {
				// if $this waited for more than 5 but less than 60 minutes, reroute itself every 1 minutes
				if ((this.stuck_time <= GlobalVariables.MAX_STUCK_TIME2) && 
						(tickcount % (int)(GlobalVariables.REROUTE_FREQ/
								GlobalVariables.SIMULATION_STEP_SIZE)) == 0) {
					this.lastRouteTime = -1; //old route is not valid 
					this.setNextRoad();
					this.assignNextLane();
				}
				else {
					// Wait for more than 60 minutes, go to the connected empty lane and reroute itself.
					for (Lane dnlane: this.lane.getDnLanes()) {
						if (this.entranceGap(dnlane) >= 1.2*this.length() && 
								(tickcount > dnlane.getLastEnterTick())) {
							dnlane.updateLastEnterTick(tickcount);
							this.removeFromLane();
							this.removeFromMacroList();
							this.setCoordMap(dnlane);
							this.appendToRoad(dnlane.getRoad());
							this.appendToLane(dnlane);
							this.linkHistory.add(dnlane.getRoad().getID());
							this.linkTimeHistory.add(tickcount);
							this.lastRouteTime = -1; // old route is not valid for sure
							this.setNextRoad();
							this.assignNextLane();
							this.desiredSpeed_ = this.road.getFreeSpeed();
							// LZ: Use current speed instead of the free speed, be consistent with the setting in enteringNetwork
							if (this.currentSpeed_ > this.road.getFreeSpeed())
								this.currentSpeed_ = this.road.getFreeSpeed();
							return 1;
						}
					}
				}
			}
		}
		coordMap.clear();
		// LZ: if $this fails to enter next link, try again in the next tick
		coordMap.add(this.getCurrentCoord());
		return 0;
	}


	public int closeToRoad(Road road) {
		Coordinate currentCoord = this.getCurrentCoord();
		Coordinate nextCoord = null;
		if (this.coordMap == null)
			return 0;
		else if (this.coordMap.size() == 0)
			return 0;
		else
			nextCoord = this.coordMap.get(0);
		if (distance(currentCoord, nextCoord) < GlobalVariables.TRAVEL_PER_TURN)
			return 1;
		else
			return 0;
	}


	/**
	 * When close to the last intersection, check if $this has reached destination
	 */
	public boolean checkAtDestination() throws Exception {
		if (distance_ < GlobalVariables.INTERSECTION_BUFFER_LENGTH) {
			this.setReachDest();
			return true;
		}
		return false;
	}

	
	public void appendToRoad(Road road) {
		this.road = road;
		this.appendToMacroList(road);
		this.reachActLocation = false;
	}


	public void appendToMacroList(Road road) {
		macroTrailing_ = null;
		// LZ: Oct 14, 2020 update
		// This has trouble with the advanceInMacroList 
		// if the macroLeading is modified in advanceInMacroList by other thread
		// then this vehicle will be misplaced in the Linked List
		if (road.getLastVehicle() != null){
			road.getLastVehicle().macroTrailing_ = this; 
			macroLeading_ = road.getLastVehicle();
		}
		else{
			macroLeading_ = null;
			road.setFirstVehicle(this);
		}
		road.setLastVehicle(this);
		// after this appending, update the number of vehicles
		road.changeNumberOfVehicles(1);
	}


	public void leading(Vehicle v) {
		if (v == this)
			throw new IllegalStateException(this+".leading(): this.leading = this");
		this.leading_ = v;
	}


	public void clearMacroLeading(){
		this.macroLeading_ = null;
	}


	public void trailing(Vehicle v) {
		if (v != null)
			this.trailing_ = v;
		else
			this.trailing_ = null;
	}


	public float distFraction() {
		if (distance_ > 0)
			return distance_ /  this.lane.getLength();
		else
			return 0;
	}


	/** HG: If nearly arrived then return 1 else 0. */
	public int nearlyArrived() {
		return (this.nextRoad_ == null) ? 1 : 0;
	}

	public void resetVehicle() {
		this.setNextPlan();
		CityContext cityContext = (CityContext) ContextCreator.getCityContext();
		Coordinate currentCoord = this.getCurrentCoord();
		Road road = cityContext.findRoadAtCoordinates(currentCoord, false);
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
		//		this.destCoord = null;
		//		this.destZone = null;
		this.targetLane_ = null;
//		this.currentCoord_ = null;
		//		this.house = null;
		// ZH: clear any remaining shadow impact
		this.clearShadowImpact();
		/* HG: Keep increasing this variable to count the number of vehicles 
		 * that have reached destination. */
		GlobalVariables.NUM_KILLED_VEHICLES++;
	}


	/**
	 * BL: From this we build functions that are used to incorporate lane
	 * object.
	 * In one road find the front leader in the same lane of the road, if
	 * there is no front, then return null
	 */
	public Vehicle findFrontBumperLeaderInSameRoad(Lane plane) {
		Vehicle front = this.macroLeading_;
		while (front != null && front.lane != plane) {
			front = front.macroLeading_;
		}
		return front;
	}


	public void removeFromLane() {
		this.lane.decreaseNumVehicles();
		this.leading_ = null;
		if (this.trailing_ != null) {
			this.lane.setFirstVehicle(this.trailing_);
			this.trailing_.leading(null);
		} else {
			this.lane.setFirstVehicle(null); 
			this.lane.setLastVehicle(null);
		}
		this.trailing_ = null;
	}



	/**
	 * BL: remove a vehicle from the macro vehicle list in the current road segment.
	 */
	public void removeFromMacroList() {
		// current road of this vehicle
		Road pr = this.getRoad();
		// if this is not the first vehicle on the road
		if (this.macroLeading_ != null) {
			this.macroLeading_.macroTrailing_ = this.macroTrailing_;
		} else { // this is the first vehicle on the road
			pr.setFirstVehicle(this.macroTrailing_);
		}
		if (macroTrailing_ != null) {
			macroTrailing_.macroLeading_ = macroLeading_;
		} else {
			pr.setLastVehicle(macroLeading_);
		}
		// LZ: Oct 19, 2020. Replaced the redundant operations below with this
		pr.changeNumberOfVehicles(-1); 
	}

	/**
	 * BL: Advance a vehicle to the position in macro vehicle list that
	 * corresponding to its current distance. This function is invoked whenever
	 * a vehicle is moved (including moved into a downstream segment), so that
	 * the vehicles in macro vehicle list is always sorted by their position.
	 * The function is called in travel()
	 * 
	 * Changed from distance_ to realDistance_ (Jul 27/2012)
	 * 
	 * @param: distance_ is with respect to the end of road
	 */
	
	public void advanceInMacroList() {
		Road pr = this.road;
		/* if no macroLeading or the distance to downstream node is greater
		 * than marcro leading, then no need to advance this veh in list */
		if (macroLeading_ == null || this.distance_ >= macroLeading_.distance_) {
			return;
		}
		/* (1) find vehicle's position in the list now this vehicle has a
		 * macroLeading that has the higher distance to downstream node that
		 * should not be the vehicle marcroLeading anymore. Need to find
		 * new marcroLeading. */
		Vehicle front = macroLeading_;
		while (front != null && this.distance_ < front.distance_) {
			front = front.macroLeading_;
		}
		/* (2) Take this vehicle out from the list
		 * this macroLeading now will be assigned to be macroLeading of this
		 * vehicle marcroTrailing */
		this.macroLeading_.macroTrailing_ = this.macroTrailing_;
		if (this.macroTrailing_ != null) {
			macroTrailing_.macroLeading_ = this.macroLeading_;
		} else {
			pr.setLastVehicle(this.macroLeading_);
		}
		// (3) Insert this vehicle after the front
		// (3.1) Point to the front
		this.macroLeading_ = front;
		if (this.macroLeading_ != null) {
			this.macroTrailing_ = macroLeading_.macroTrailing_;
			this.macroLeading_.macroTrailing_ = this;
		} else { 
			this.macroTrailing_ = pr.getFirstVehicle();
			pr.setFirstVehicle(this);
		}
		// (3.2) Point to the trailing vehicle
		if (macroTrailing_ != null) {
			macroTrailing_.macroLeading_ = this;
		} else {
			pr.setLastVehicle(this);
		}
	}

	/**
	 * BL: this function will check if the current
	 * lane connect to a lane in the next road if yes then it gives the
	 * checkLaneFlag true value if not then the checkLaneFlag has false value
	 * the function will be called after the vehicle updates its route i.e. the
	 * routeUpdateFlag has true value
	 */
	
	public boolean isCorrectLane() {
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
		return this.correctLane;
	}

	/** Find if the potential next road and current lane are connected. */
	
	
	public boolean checkNextLaneConnected(Road nextRoad) {
		boolean connected = false;
		Lane curLane = this.lane;
		if (nextRoad != null) {
			for (Lane dl : curLane.getDnLanes()) {
				if (dl.getRoad().equals(nextRoad)) {
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
			if (this.getVehicleID() == GlobalVariables.GLOBAL_VEHICLE_ID)
				logger.info("Assign next lane: current link ID= "
						+ curRoad.getLinkid() + " current lane ID: "
						+ curLane.getLaneid() + " next link ID="
						+ this.nextRoad());
			this.nextLane_ = null;
			return;
		} else {
			for (Lane dl : curLane.getDnLanes()) {
				if (dl.getRoad().equals(this.nextRoad())) {
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
						if (ul.getRoad().getID() == curRoad.getID()) {
							this.nextLane_ = pl;
							break; // assign the next lane to the 1st connected lane
						}
					}
				}
				// HG and XQ: force movement at a 5 leg or irregular intersection
				this.nextLane_ = this.nextRoad().getLane(0);
			}
			if (this.nextLane_ == null)
				logger.error("No next lane found for vehicle: "
						+ this.vehicleID_ + " moving on the road: "
						+ this.getRoad().getLinkid() + " lane: "
						+ this.getLane().getLaneid() + " heading to location "
						+ this.getDestinationID()
						+ " while looking for next lane on road: "
						+ this.nextRoad().getLinkid() + " that has "
						+ this.nextRoad().getnLanes() + " lanes");
			if (nextLane_ == this.lane)
				throw new IllegalStateException(
						this+".assignNextLane(): this.nextLane = this");
		}
	}

	/**
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
		if (curLeading == this || curTrailing == this)
			throw new IllegalStateException(
					this+".changeLane(): this.leading/trailing = this");
		if (curTrailing != null) {
			if (curLeading != null) {
				curLeading.trailing(curTrailing);
				curTrailing.leading(curLeading);
			} else {
				curTrailing.leading(null);
				this.lane.setFirstVehicle(curTrailing);
			}
		} else if (curLeading != null) {
			this.lane.setLastVehicle(curLeading);
			curLeading.trailing(null);
		} else {
			this.lane.setFirstVehicle(null);
			this.lane.setLastVehicle(null);
		}

		this.lane.decreaseNumVehicles();
		/*
		 * BL: after change the lane the vehicle updates its leading and
		 * trailing in the target lanes. and also the lead and lag vehicle have
		 * to update its leading and trailing.
		 */
		this.lane = plane; // vehicle move to the target lane
		if (leadVehicle != null) {
			if (this.distance_ < leadVehicle.distance_) {
				logger.error("Vehicle.changeLane(): distance_=" + distance_ + 
						" is less than leadVehicle.distance_=" + leadVehicle.distance_ +
						" on lane=" + lane.getLaneid() + " with length=" + lane.getLength());
			}

			this.leading_ = leadVehicle;
			this.leading_.trailing(this);
			if (lagVehicle != null) {
				if (lagVehicle.distance_ < this.distance_) {
					logger.error("Vehicle.changeLane(): distance_=" + distance_ + 
							" is greater than lagVehicle.distance_=" + lagVehicle.distance_ +
							" on lane=" + lane.getLaneid() + " with length=" + lane.getLength());
				}
				this.trailing_ = lagVehicle;
				this.trailing_.leading(this);
			} else {
				this.trailing(null);
				this.lane.setLastVehicle(this);
			}
		} else if (lagVehicle != null) {
			if (lagVehicle.distance_ < this.distance_) {
				logger.error("Vehicle.changeLane(): distance_=" + distance_ + 
						" is greater than leadVehicle.distance_=" + lagVehicle.distance_ +
						" (when leadVehicle is null), $this on lane=" +
						lane.getLaneid() + " with length=" + lane.getLength());
			}
			this.lane.setFirstVehicle(this);
			this.leading(null); //LZ: 2021/1/20, this line was missing
			this.trailing_ = lagVehicle;
			this.trailing_.leading(this);
		} else {
			this.lane.setFirstVehicle(this);
			this.lane.setLastVehicle(this);
			this.leading(null);
			this.trailing(null);
		}
		this.lane.increaseNumVehicles();
		this.updateCoordMap(this.lane); // update coordMap
	}

	/**
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
	}

	/**
	 * BL: if the vehicle with MLC state can't change the lane after some
	 * distance. The vehicle need to nose and yield the lag Vehicle of the
	 * target lane in order to have enough gap to change the lane This function
	 * is called only when nosingFlag is true and must be recalled until
	 * nosingFlag receive false value after the vehicle nosed, tag the lag
	 * vehicle in target lane to yielding status. This function will be called
	 * in makeAccelerationDecision
	 */
	
	
	public float nosing()
	{
		float acc = 0;
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
				// LZ: Nov 3, 2020. Based on the description, should not be
				// lagVehicle.yieldingFlag = true
				this.yieldingFlag = true;
			}
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
		if (Float.isNaN(acc)) {
			logger.error("Vehicle.nosing(): acceleration=NaN for " + this);
		}
		return acc;
	}

	/**
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

	/**
	 * BL: When change lane, distance need to be adjusted with the lane width.
	 * Calculate critical lead gap of the vehicle with the lead vehicle in the
	 * target lane.
	 */
	
	
	public float critLeadGapMLC(Vehicle leadVehicle) {
		float critLead = 0;
		float minLead_ = GlobalVariables.minLead;
		float betaLead01 = GlobalVariables.betaLeadMLC01;
		float betaLead02 = GlobalVariables.betaLeadMLC02;
		float gama = GlobalVariables.gama;
		if (leadVehicle != null)
			critLead = (float) (minLead_
					+ (betaLead01 * this.currentSpeed() + betaLead02
							* (this.currentSpeed() - leadVehicle.currentSpeed()))
					* (1 - Math.exp(-gama * this.distance())));
		if (critLead < minLead_)
			critLead = minLead_;
		return critLead;
	}

	/**
	 * BL: Calculate lead gap of $this with the lead vehicle in the target lane.
	 */
	

	public double leadGap(Vehicle leadVehicle) {
		double leadGap = 0;
		if (leadVehicle != null){
			// leadGap>=-leadVehicle.length()
			leadGap = this.distance() - leadVehicle.distance() - leadVehicle.length();
		}
		else{
			// LZ: Nov 3, 2020. Change this.distance() to 9999999
			leadGap = this.distance();
		}
		return leadGap;
	}

	/** 
	 * BL: Calculate critical lag gap of $this with lag vehicle in target lane.
	 */
	
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

	/**
	 * BL: Calculate lag gap of $this with the lag vehicle in target lane.
	 */
	
	public double lagGap(Vehicle lagVehicle) {
		double lagGap = 0;
		if (lagVehicle != null)
			lagGap = lagVehicle.distance() - this.distance() - this.length();
		else
			// LZ: Nov 3, 2020. Still need to -this.length()
			lagGap = this.lane.getLength() - this.distance() - this.length();
		return lagGap;
	}

	/**
	 * BL: Find the lead vehicle in target lane (plane)
	 */
	
	public Vehicle leadVehicle(Lane plane) {
		if (plane == this.lane)
			throw new IllegalArgumentException(
					this+".leadVehicle(): target lane = current lane");
		Vehicle leadVehicle = this.macroLeading_;
		while (leadVehicle != null && leadVehicle.lane != plane) {
			leadVehicle = leadVehicle.macroLeading_;
		}
		if (leadVehicle == this)
			throw new IllegalStateException(this+".leadVehicle(): this = leadVehicle");
		return leadVehicle;
	}

	/** 
	 * BL: Find lag vehicle in target lane (plane)
	 */
	
	public Vehicle lagVehicle(Lane plane) {
		Vehicle lagVehicle = this.macroTrailing_;
		while (lagVehicle != null && lagVehicle.lane != plane) {
			lagVehicle = lagVehicle.macroTrailing_;
		}
		return lagVehicle;
	}

	/**
	 * BL: Following we will implement discretionary LC model At current stage,
	 * the DLC is implementing as follow: 1. If the vehicle is not close to
	 * downstream node 2. and it finds a correct lane with better traffic
	 * condition -> then it will change lane.
	 * If the vehicle is in correct lane then we find a better lane that is
	 * also connected to downstream line this function is called at the
	 * makeLaneChangingDecision
	 */
	
	public Lane findBetterLane() {
		Lane curLane = this.lane;
		Lane targetLane = null;
		Lane rightLane = this.rightLane();
		Lane leftLane = this.leftLane();
		// if left and right lane exist then check if they are both connect to
		// next lane or not
		if (this.equals(curLane.getFirstVehicle())) {
			return null;
		} else {
			if (leftLane != null && rightLane != null) {
				Lane tempLane = leftLane.betterLane(rightLane);
				// get the lane that has best traffic condition 
				targetLane = curLane.betterLane(tempLane); 
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
				} else if (this.leading_ != null
						&& this.leading_.currentSpeed_ < this.desiredSpeed_
						&& this.currentSpeed_ < this.desiredSpeed_) {
					if (front.currentSpeed_ > this.currentSpeed_
							&& front.accRate_ > 0)
						return targetLane;
				} else if (this.leading_ == null) {
					logger.error(
							"Vehicle.findBetterLane(): this.leading_=null for"+this);
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
		// if left and right lane exist then check if they are both connect to
		// next lane or not
		if (this.equals(curLane.getFirstVehicle())) {
			return null;
		} else {
			if (leftLane != null && rightLane != null) {
				// if both left and right lanes are connected to downstream lane
				if (leftLane.isConnectToLane(this.nextLane_)
						&& rightLane.isConnectToLane(this.nextLane_)) {
					Lane tempLane = leftLane.betterLane(rightLane);
					// get the lane that has best traffic condition 
					targetLane = curLane.betterLane(tempLane); 
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


	/**
	 * Once $this finds a better lane (plane), it changes to that lane.
	 */
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
		// change betaLead01 and 02 to DLC value.
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
		// change betaLag01 and 02 to DLC value
		if (pv != null) {
			critLag = minLag + GlobalVariables.betaLagDLC01
					* this.currentSpeed_ + GlobalVariables.betaLagDLC02
					* (this.currentSpeed_ - pv.currentSpeed_);
		}
		critLag = Math.max(minLag, critLag);
		return critLag;
	}


	/**
	 * After appending vehicle to next road, a new nextlane being assigned. The
	 * coordinate map will be updated with the last point is the starting point
	 * of the next lane
	 */
	public void updateCoorMapWithNewLane() {
		Coordinate coor1, coor2;
		if (!this.coordMap.isEmpty()) {
			int end = this.coordMap.size() - 1;
			coor1 = this.coordMap.get(end);
			if (this.vehicleID_ == GlobalVariables.GLOBAL_VEHICLE_ID
					&& !GlobalVariables.DEBUG_ON_ROAD) {
				logger.error("Vehicle.updateCoordMapWithNewLane(): " + this +
						" already had its coordinate map but need to update " +
						"for moving through junction which connects to the " +
						"point: " + coor1);
			}
		} else {
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
			if (this.vehicleID_ == GlobalVariables.GLOBAL_VEHICLE_ID
					&& !GlobalVariables.DEBUG_ON_ROAD) {
				logger.info("The adding coordinate is from the lane: "
						+ plane.getLaneid());
			}
		} else
			coor2 = this.destCoord;
		if (!coor2.equals(coor1)) {
			this.coordMap.add(coor2);
			if (this.vehicleID_ == GlobalVariables.GLOBAL_VEHICLE_ID
					&& !GlobalVariables.DEBUG_ON_ROAD) {
				logger.info("Vehicle.updateCoorMapWithNewLane(): " +
						" Added new coordinate " + this);
			}
		} else {
			if (this.vehicleID_ == GlobalVariables.GLOBAL_VEHICLE_ID
					&& !GlobalVariables.DEBUG_ON_ROAD) {
				logger.info("Vehicle.updateCoorMapWithNewLane(): " +
						" No coordinate added " + this);
			}
		}
		if (this.id == GlobalVariables.GLOBAL_VEHICLE_ID
				&& !GlobalVariables.DEBUG_ON_ROAD) {
			int end = this.coordMap.size() - 1;
			logger.info("Vehicle.updateCoorMapWithNewLane(): " +
					" Coordinate map after the next lane for " + this + ": ");
			logger.info("Distance to added point: "
					+ distance(this.coordMap.get(end - 1),
							this.coordMap.get(end)));
		}
	}

	/**
	 * BL: when vehicle approach junction, need new coordinate and distance
	 */

	public int appendToJunction(Lane nextlane) {
		if (this.atDestination()) {
			return 0;
		} else { // LZ: want to change to next lane
			coordMap.clear();
			coordMap.add(this.getCurrentCoord());
		}
		this.onlane = false;
		if (this.changeRoad() == 0){
			return 0;
		}
		return 1;
	}


	
	public float entranceGap(Lane nextlane) {
		float gap = 0;
		if (nextlane != null) {
			Vehicle newleader = nextlane.getLastVehicle();
			if (newleader != null) {
				gap = nextlane.getLength() - newleader.distance_ - newleader.length();
			} else
				gap = 9999999;
		}
		return gap;
	}

	
	private Coordinate getNearestCoordinate(Coordinate c, Coordinate c1,
			Coordinate c2) {
		float dist1 = distance(c, c1);
		float dist2 = distance(c, c2);
		if (dist1 < dist2)
			return c1;
		else
			return c2;
	}

	
	private float distance(Coordinate c1, Coordinate c2) {
		calculator.setStartingGeographicPoint(c1.x, c1.y);
		calculator.setDestinationGeographicPoint(c2.x, c2.y);
		float distance;
		try {
			distance = (float) calculator.getOrthodromicDistance();
		} catch (AssertionError ex) {
			logger.error("Error with finding distance");
			distance = 0.0f;
		}
		if (Double.isNaN(distance)) {
			logger.error("distance is NaN in Vehicle.distance() for " + this);
		}
		return distance;
	}

	// LZ: Jan 19, 2021, it turns out the Azimuth is not the bearing, so I replaced the caculation of radius
	
	private float distance2(Coordinate c1, Coordinate c2, float[] distAndAngle) {
		float distance;
		float radius;
		calculator.setStartingGeographicPoint(c1.x, c1.y);
		calculator.setDestinationGeographicPoint(c2.x, c2.y);
		try {
			distance = (float) calculator.getOrthodromicDistance();
			radius = (float) calculator.getAzimuth(); // the azimuth in degree, value from -180-180
		} catch (AssertionError e) {
			logger.error("Error with finding distance: " + e);
			distance = 0.0f;
			radius = 0.0f;
		}
		if (distAndAngle != null && distAndAngle.length == 2) {
			distAndAngle[0] = distance;
			distAndAngle[1] = radius;
		}
		if (Double.isNaN(distance)) {
			// RV: Check if this condition ever occurs
			logger.error("Geodetic distance is NaN for " + this);
			distance = 0.0f;
			radius = 0.0f;
		}
		return distance;
	}

	
	/**
	 * LZ: A light weight move function based on moveVehicleByVector, and is 
	 * much faster than the one using geom
	 * 2021/1/20 update, this is just for getting the coordinate within line 
	 * segments (has nothing to do with the distance update), therefore 
	 * linear interpolation is enough
	 */
	private void move2(Coordinate origin, Coordinate target, 
			float distanceToTarget, float distanceTravelled){
		float p = distanceTravelled/distanceToTarget;
		if (p<1) {
			this.setCurrentCoord(new Coordinate((1-p)*origin.x + p*target.x,
					(1-p)*origin.y + + p*target.y));
		}
		else {
			logger.error("Vehicle.move2(): Cannot move " + this + "from "
					+ origin + " by dist=" +  distanceTravelled);
		}
	}


	public void setVehicleClass(int vehicleClass) {
		this.vehicleClass = vehicleClass;
	}


	/**
	 * RV, JX: Assign the indifference band (eta) as found in Mahmassani & 
	 * Jayakrishnan (1991) [doi.org/10.1016/0191-2607(91)90145-G].
	 * It is assumed to be distributed as an isosceles triangle,
	 * centered at the mean eta and having base width 0.5*eta.
	 * @return the eta value
	 */
	public float assignIndiffBand(){
		float mean = GlobalVariables.ETA;
		float base = 0.5f * mean; // base width of the triangle
		float start = mean - 0.5f * base; // starting point of the distribution
		float end = mean + 0.5f * base; // end point ""
		float rand = GlobalVariables.RandomGenerator.nextFloat();
		float indiffbandvalue;
		if (rand <= 0.5) {
			indiffbandvalue = (float) (start + base * Math.sqrt(0.5f * rand));
		} else {
			indiffbandvalue = (float) (end - base * Math.sqrt(0.5f * (1.0f - rand)));
		}
		return indiffbandvalue;	
	}

	
	/**
	 * LZ,RV: Find next shelter destination given current destination.
	 */
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
			logger.info(this + " reached dest shelter " + curDest);
			String formatted_msg = (getVehicleID()
					+ ";" + 1 +
					";" + getDepTime() + 
					";" + getEndTime() +
					";" + getHouse().getZoneId() +
					";" + getDestinationID() + 
					";" + accummulatedDistance_ +
					";" + visitedShelters.size()+
					";" + this.linkHistory.toString() +
					";" + this.linkTimeHistory.toString()
					);
			try {
				ContextCreator.bw.write(formatted_msg);
				ContextCreator.bw.newLine();
			} catch (IOException e) {
				e.printStackTrace();
			}
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
			}
			// if all shelters have rejected this vehicle, kill it
			else {
				logger.info(this + ": All shelters exhausted; killing self");
				this.killVehicle();
			}
		}
	}

	@Override
	public String toString() {
		return "<Veh" + this.vehicleID_ + ">";
	}

	public Vehicle moveToNewLane(Lane toLane) {
		Vehicle oldLeading = this.leading();
		if (oldLeading != null && oldLeading.trailing() != this)
			throw new IllegalStateException(
					this+".moveToNewLane(): this.leading.trailing != this");
		this.removeFromMacroList();
		// remove vehicle from its lane (but not call $this.removeLane())
		this.lane.decreaseNumVehicles();
		this.trailing_ = null;
		this.leading_ = null;
		// add this vehicle to the new road and lane
		this.appendToLane(toLane);
		this.appendToRoad(toLane.getRoad());
		this.setNextRoad();
		this.assignNextLane();
		this.trailing_ = oldLeading;
		// reverse the distance from the downstream node of the lane
		float newDistance_ = toLane.getLength() - this.distance();
		if (Float.isNaN(newDistance_) || newDistance_ < 0)
			throw new IllegalStateException(
					"Distance from downstream junction is negative/NaN.");
		this.distance_ = newDistance_;
		// also invert its lane's remaining control points
		this.updateCoordMap(toLane);
		// sort the macroList according to updated position
		this.advanceInMacroList();
		// move on to the next vehicle (i.e., traverse the old lane's 
		// vehicle list upstream)
		return oldLeading;
	}
	
}
