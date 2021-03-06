package evacSim.citycontext;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Iterator;
import java.util.LinkedList;
import java.util.Queue;
import java.util.Set;
import java.util.TreeMap;
import org.apache.log4j.Logger;

import com.vividsolutions.jts.geom.Coordinate;
import com.vividsolutions.jts.geom.Geometry;

import repast.simphony.engine.environment.RunEnvironment;
import repast.simphony.essentials.RepastEssentials;
import repast.simphony.space.gis.Geography;
import evacSim.*;
import evacSim.citycontext.Lane;
import evacSim.data.DataCollector;
import evacSim.vehiclecontext.Vehicle;

public class Road {
	private Logger logger = ContextCreator.logger;
	private int id;
	private int linkid;
	private int left;
	private int right;
	private int nLanes;
	private int fromNode;
	private int toNode;
	private int through; // next link when going straight
	private int tlinkid; // coupled (opposite direction) link
	private int curhour; // BL: to find the current hour of the simulation
	private String identifier; // can be used to match with shape file roads
	private int nVehicles_; // SH: number of vehicles currently in the road
	private double length;
	private float speed_; // current speed
	private double travelTime;
	private float freeSpeed_;
	private Road oppositeRoad;
	private ArrayList<Road> downStreamMovements;
	private ArrayList<Lane> lanes; // Use lanes as objects inside the road
	private ArrayList<Junction> junctions;
	
	/* For adaptive network partitioning */
	// Potential vehicles might be loaded on the road
	private int nShadowVehicles;
	// Potential vehicles might performing routing on the road
	private int nFutureRoutingVehicles;
	
	/* RV: Road snapshot parameters */
	// No. of vehicles & avg. speed on road as of last recording
	private int lastRecordedNumVehicles;
	private float lastRecordedSpeed;
	
	// SH: Dynamic travel time of
	private ArrayList<Double> dynamicTravelTime;
	// LZ: Use LinkedList which implement Queue for O(1) complexity of removing vehicles.
	private TreeMap<Double, Queue<Vehicle>> newqueue;
	//LZ: Can be null when no vehicle is was on the road
	private Vehicle lastVehicle_ = null;
	private Vehicle firstVehicle_ = null;
	// Indicator whether there is an event happening on the road
	private boolean eventFlag;
	// Store default speed limit value in case of events
	private double defaultFreeSpeed_;
	
	private ArrayList<Vehicle> newVehs;
	private ArrayList<Vehicle> arrVehs;

	// Road constructor
	public Road() {
		this.id = ContextCreator.generateAgentID();
		this.junctions = new ArrayList<Junction>();
		this.lanes = new ArrayList<Lane>();
		this.nVehicles_ = 0;
		this.defaultFreeSpeed_ = this.freeSpeed_;
		this.downStreamMovements = new ArrayList<Road>();
		this.oppositeRoad = null;
		this.newqueue = new TreeMap<Double, Queue<Vehicle>>();
		this.identifier = " ";
		this.curhour = -1;
		this.travelTime = (float) this.length / this.freeSpeed_;
		
		// For adaptive network partitioning
		this.nShadowVehicles = 0;
		this.nFutureRoutingVehicles = 0;
		this.eventFlag = false;
		// RV: For road snapshot
		this.lastRecordedNumVehicles = 0;
		this.lastRecordedSpeed = 0f;
		
		// LZ: Handle vehicles' entering the link in single thread manner
		this.newVehs = new ArrayList<Vehicle>();
		this.arrVehs = new ArrayList<Vehicle>();
//		enteringVehicles = Collections.synchronizedList(new ArrayList<Vehicle>());
	}
	
	public String toString() {
		return "<Road"+this.linkid+">";
	}
	
	// Set the defaultFreeSpeed_
	public void setDefaultFreeSpeed() {
		this.defaultFreeSpeed_ = this.freeSpeed_;
	}
	
	// Get the defaultFreeSpeed_
	public double getDefaultFreeSpeed() {
		return this.defaultFreeSpeed_;
	}
	
	// Check the eventFlag
	public boolean checkEventFlag() {
		return this.eventFlag;
	}
	
	// Set the eventFlag
	public void setEventFlag() {
		this.eventFlag = true;
	}
	
	// Restore the eventFlag after the event
	public void restoreEventFlag() {
		this.eventFlag = false;
	}
	
	/* New step function using node based routing */
	// @ScheduledMethod(start=1, priority=1, duration=1)
	public void step() {	
		// if (RepastEssentials.GetTickCount() % 10 == 0)
		// this.setTravelTime();
		// double time = System.currentTimeMillis();
		Vehicle v;
		int tickcount = (int) RepastEssentials.GetTickCount();
		// double maxHours = 0.5;
		// double maxTicks = 3600 * maxHours // time that a path is kept in the
		// list
		// / GlobalVariables.SIMULATION_STEP_SIZE;
		try {
			while (this.newqueue.size() > 0) {
				v = this.newqueueHead(); // BL: change to use the TreeMap

//				if (v.atActivityLocation()) {
////					logger.info(v.getCurrentCoord());
//					v.setCoordMap(this.firstLane());
//				}
				
				if (v.closeToRoad(this) == 1 && tickcount >= v.getDepTime()) {
					if (v.enterNetwork(this) == 1) {
						newVehs.add(v);
						v.advanceInMacroList();
					}
					else {
						break;
					}
				} else {
					// BL: iterate all element in the TreeMap
					@SuppressWarnings("rawtypes")
					Set keys = (Set) this.newqueue.keySet();
					for (@SuppressWarnings("rawtypes")
					Iterator i = (Iterator) keys.iterator(); i.hasNext();) {
						Double key = (Double) i.next();
						Queue<Vehicle> temList = this.newqueue.get(key);
						for (Vehicle pv : temList) {
							if (tickcount >= pv.getDepTime()) {
								pv.primitiveMove();
							} else {
								break;
							}
						}
					}
					break;
				}

			}
			// the first vehicle in a road
			Vehicle pv = this.firstVehicle();
			if (pv != null && pv.leading() != null) {
				// if the behind vehicle surpass the front one, it's a problem
//				logger.warn("Road.step(): Inconsistency of macroleading and leading is detected"
//						+ "," + pv.getLane().getLaneid()
//						+ "," + pv.getLane().getLength()
//						+ "," + pv.leading().getLane().getLaneid()
//						+ "," + pv.distance()
//						+ "," + pv.leading().distance());
				pv.leading(null);
			}
			while (pv != null) {
				if(tickcount<=pv.getLastMoveTick()){
					pv = pv.macroTrailing();
					break; // reached the last vehicle
				}
				pv.updateLastMoveTick(tickcount);
				if (!pv.calcState()) {
					// this vehicle list is corrupted, do not proceed for this road
					logger.info("Link "+this.linkid+" vehicle list is corrupted");
					break;
				}
				if (tickcount % GlobalVariables.FREQ_RECORD_VEH_SNAPSHOT_FORVIZ == 0 && pv.getMovingFlag()) {
					/* LZ: if you don't want to record unmoved vehicles,
					 * also add the condition in the if block: && pv.getMovingFlag()){
					 * LZ: Note vehicle can be killed after calling pv.travel,
					 * so we record vehicle location here! */
					pv.recVehSnaphotForVisInterp();
				}
				// main step in traveling
				pv.travel();
				
				if (pv.atDestination()) {
					arrVehs.add(pv);
				}
				pv = pv.macroTrailing();
			}
			if(tickcount % GlobalVariables.FREQ_RECORD_VEH_SNAPSHOT_FORVIZ==0){
				for (Vehicle veh : newVehs) {
					try {
						DataCollector.getInstance().recordNewVehicleTickSnapshot(veh);
					} catch (Throwable t) {
						logger.error("Cannot store the new vehicles " +
								veh.getVehicleID() + "," +
								veh.getOriginalCoord() + "," + 
								veh.getDestCoord());
						DataCollector.printDebug("ERR" + t.getMessage());
					}
				}
				for (Vehicle veh : arrVehs) {
					try {
						DataCollector.getInstance().recordArrVehicleTickSnapshot(veh);
					} catch (Throwable t) {
						// System.out.println("Cannot store the
						// data"+t.getMessage());
						System.err.println("Cannot store the arrived vehicles"+t.getMessage());
						DataCollector.printDebug("ERR" + t.getMessage());
					}
				}
				newVehs = new ArrayList<Vehicle>();
				arrVehs = new ArrayList<Vehicle>();
			}
		} catch (Exception e) {
			logger.error("Road " + this.linkid
					+ " had an error while moving vehicles");
			e.printStackTrace();
			RunEnvironment.getInstance().pauseRun();
		}
		
		/**
		 * RV: Record snapshot of this road only if its attributes are different
		 * from the last time the snapshot was recorded. This is done to save 
		 * memory and storage space.
		 * */
		if (tickcount % GlobalVariables.FREQ_RECORD_ROAD_SNAPSHOT_FORVIZ == 0) {
			// if any snapshot attribute is different from the last snapshot
			if (speed_ != lastRecordedSpeed || getVehicleNum() != lastRecordedNumVehicles) {
				lastRecordedNumVehicles = getVehicleNum();
				lastRecordedSpeed = speed_;
				try {
					DataCollector.getInstance().recordRoadTickSnapshot(this);
				} catch (Throwable t) {
					// could not log the vehicle's new position in data buffer!
					DataCollector.printDebug("ERR" + t.getMessage());
				}
			}
		}
		
	}
	
	public void setLinkid(int linkid) {
		this.linkid = linkid;
	}

	public int getLinkid() {
		return linkid;
	}

	public void setTlinkid(int tlinkid) {
		this.tlinkid = tlinkid;

	}

	public int getTlinkid() {
		return tlinkid;
	}

	public int getID() {
		return id;
	}

	public void setOppositeRoad(Road r) {
		this.oppositeRoad = r;
	}

	public Road getOppositeRoad() {
		return this.oppositeRoad;
	}
	
	public void sortLanes() {
		Collections.sort(this.lanes, new LaneComparator());
	}

	/**
	 * Get a unique identifier for this Road. Not the same as ID (which is an
	 * auto-generated field common to every agent used in the model), this
	 * identifier is used to link road features in a GIS with Edges added to the
	 * RoadNetwork (a repast Network Projection).
	 * 
	 * @return the identifier for this Road.
	 * @throws NoIdentifierException
	 *             if the identifier has not been set correctly. This might
	 *             occur if the roads are not initialized correctly (e.g. there
	 *             is no attribute called 'identifier' present in the shape-file
	 *             used to create this Road).
	 */
	public String getIdentifier() {
		if (identifier == "" || identifier == null) {
			System.err
					.println("Road: error, the identifier field for this road has not been initialised."
							+ "\n\tIf reading in a shapefile please make sure there is a string column called 'identifier' which is"
							+ " unique to each feature");
		}
		return identifier;
	}

	public void setLeft(int left) {
		this.left = left;
	}

	public int getLeft() {
		/*
		 * if (left == "" || left == null) { logger.error(
		 * "Road: error, the name field for this road has not been initialised."
		 * +
		 * "\n\tIf reading in a shapefile please make sure there is a string column called 'left' which is"
		 * + " unique to each feature"); }
		 */
		return left;
	}

	public void setThrough(int through) {
		this.through = through;
	}

	public int getThrough() {
		/*
		 * if (through == "" || through == null) { logger.error(
		 * "Road: error, the name field for this road has not been initialised."
		 * +
		 * "\n\tIf reading in a shapefile please make sure there is a string column called 'through' which is"
		 * + " unique to each feature"); }
		 */
		return through;
	}

	public void setRight(int right) {
		this.right = right;
	}

	public int getRight() {
		/*
		 * if (right == "" || right == null) { logger.error(
		 * "Road: error, the name field for this road has not been initialised."
		 * +
		 * "\n\tIf reading in a shapefile please make sure there is a string column called 'right' which is"
		 * + " unique to each feature"); }
		 */
		return right;
	}

	public void setFn(int node) {
		this.fromNode = node;
	}

	public int getFn() {
		return fromNode;
	}

	public void setTn(int node) {
		this.toNode = node;
	}

	public int getTn() {
		return toNode;
	}

	public void setLength(double length) {
		this.length = length;
	}

	public double getLength() {
		return length;
	}

	public void setIdentifier(String identifier) {
		this.identifier = identifier;
	}

	/**
	 * Used to tell this Road who it's Junctions (end-points) are.
	 * @param j the Junction at either end of this Road.
	 */
	public void addJunction(Junction j) {
		if (this.junctions.size() == 2) {
			System.err
					.println("Road: Error: this Road object already has two Junctions.");
		}
		this.junctions.add(j);
	}

	public ArrayList<Junction> getJunctions() {
		if (this.junctions.size() != 2) {
			System.err
					.println("Road: Error: This Road does not have two Junctions");
		}
		return this.junctions;
	}

	/**
	 * BL: This function returns all the downstream movement of the current road
	 */
	public void roadMovement() {
		ArrayList<Road> allRoads_ = new ArrayList<Road>();
		ArrayList<Road> allMovements_ = new ArrayList<Road>();
		// logger.info("Searching connection for road " +
		// this.getIdentifier());
		if (this.getJunctions() != null) {
			for (Road r : this.getJunctions().get(1).getRoads()) {
				if (r.getID() != this.getID()) {
					if (r.getJunctions().get(1).getID() != this.getJunctions()
							.get(0).getID()) {
						allRoads_.add(r);
					}
				}
			}
			if (allRoads_.size() != 0) {
				for (Road r : allRoads_) {
					if (this.getJunctions().get(1).getID() == r.getJunctions()
							.get(0).getID()) {
						allMovements_.add(r);
					}
				}
			}

			double[] angles_ = new double[allMovements_.size()];

			// Form this to a separated function to calculate the counterclockwise angle between two links
			if (allMovements_.size() != 0) {
				double x1 = this.getJunctions().get(0).getCoordinate().x;
				double y1 = this.getJunctions().get(0).getCoordinate().y;
				double x0 = this.getJunctions().get(1).getCoordinate().x;
				double y0 = this.getJunctions().get(1).getCoordinate().y;
				double angle1 = atan3(x1, y1, x0, y0);
				int i = 0;
				for (Road r : allMovements_) {
					double x2 = r.getJunctions().get(1).getCoordinate().x;
					double y2 = r.getJunctions().get(1).getCoordinate().y;
					double angle2 = atan3(x2, y2, x0, y0);
					double angle = angle2 - angle1;
					if (angle > 0)
						angles_[i] = angle;
					else
						angles_[i] = angle + 2 * Math.PI;
					i++;
				}
			}
			// now we sort the angles from smallest to biggest
			if (angles_.length != 0) {
				double listAngles[] = new double[allMovements_.size()];
				double maxAngle = 0;
				double currentAngle = 0;
				int j = 0;
				while (j < angles_.length) {
					maxAngle = 0;
					if (j == 0) {
						for (int i = 0; i < angles_.length; i++) {
							currentAngle = angles_[i];
							if (maxAngle < currentAngle) {
								maxAngle = currentAngle;
							}
						}
						listAngles[j] = maxAngle;
					} else {
						for (int i = 0; i < angles_.length; i++) {
							currentAngle = angles_[i];
							double check = listAngles[j - 1];
							if (maxAngle < currentAngle && currentAngle < check) {
								maxAngle = currentAngle;
							}
						}
						listAngles[j] = maxAngle;
					}
					j++;
				}
				j = 0;
				// check the angles and add the movement to the list from least
				// to last
				while (j < listAngles.length) {
					for (int i = 0; i < angles_.length; i++) {
						if (listAngles[j] == angles_[i]) {
							Road pr = allMovements_.get(i);
							this.downStreamMovements.add(pr);
						}
					}
					j++;
				}
			}
		}
	}

	public void addDownStreamMovement(Road dsRoad) {
		this.downStreamMovements.add(dsRoad);
	}

	public ArrayList<Road> getConnectedRoads() {
		return this.downStreamMovements;
	}

	public void setNumberOfVehicles(int nVeh){
		this.nVehicles_ = nVeh;
		if(this.nVehicles_<0) {
			this.nVehicles_=0;
		}
	}
	
	public void changeNumberOfVehicles(int nVeh){
		this.nVehicles_ += nVeh;
		if(this.nVehicles_<0) {
			this.nVehicles_=0;
		}
	}
	
	public void firstVehicle(Vehicle v) {
		if (v != null) {
//			if(v.leading()!=null){
//				logger.info("Well " + v.distance() + ","+v.getLane().getLaneid() + "," + v.leading().distance()+"," +v.leading().getLane().getLaneid());
//			}
			this.firstVehicle_ = v;
		}
		else
			this.firstVehicle_ = null;
	}

	public void lastVehicle(Vehicle v) {
		if (v != null)
			this.lastVehicle_ = v;
		else
			this.lastVehicle_ = null;
	}

	public Vehicle firstVehicle() {
		return firstVehicle_;
	}

	public Vehicle lastVehicle() {
		return lastVehicle_;
	}
	
	/* Number of vehicles on the road */
	public int getVehicleNum() {
		return this.nVehicles_;
	}
	
	public boolean hasUpdatableVehicle() {
		return (this.nVehicles_ > 0 && this.newqueue.size() > 0);  
	}
	
	/* For adaptive network partitioning */
	public int getShadowVehicleNum() {
		return this.nShadowVehicles;
	}
	
	public void incrementShadowVehicleNum() {
		this.nShadowVehicles++;
	}
	
	public void resetShadowVehicleNum() {
		this.nShadowVehicles = 0;
	}
	
	public void decreaseShadowVehicleNum() {
		this.nShadowVehicles--;
		if (this.nShadowVehicles < 0)
			this.nShadowVehicles = 0;
	}
	
	public int getFutureRoutingVehNum() {
		return this.nFutureRoutingVehicles;
	}
	
	public void incrementFutureRoutingVehNum() {
		this.nFutureRoutingVehicles++;
	}
	
	public void resetFutureRountingVehNum() {
		this.nFutureRoutingVehicles = 0;
	}
	
	public void decreaseFutureRoutingVehNum() {
		this.nFutureRoutingVehicles--;
		if (this.nFutureRoutingVehicles < 0)
			this.nFutureRoutingVehicles = 0;
	}
	
	/**
	 * BL: this add queue using TreeMap structure
	 */
	public void addVehicleToNewQueue(Vehicle v) {
		double departuretime_ = 0;
		departuretime_ = v.getDepTime();
		if (!this.newqueue.containsKey(departuretime_)) {
			Queue<Vehicle> temporalList = new LinkedList<Vehicle>();
			temporalList.add(v);
			this.newqueue.put(departuretime_, temporalList);
		} else {
			this.newqueue.get(departuretime_).add(v);
		}
		v.setRoad(this);
//		v.setGeography();
	}

	/**
	 * RemoveVehicleFromNewQueue BL: will remove vehicle v from the TreeMap by
	 * looking at the departuretime_ of the vehicle if there are more than one
	 * vehicle with the same departuretime_, it will remove the vehicle match
	 * with id of v.
	 */
	public void removeVehicleFromNewQueue(Vehicle v) {
		double departuretime_ = v.getDepTime();
		Queue<Vehicle> temporalList = this.newqueue.get(departuretime_);
		if (temporalList.size() > 1) {
			this.newqueue.get(departuretime_).poll();
		} else {
			this.newqueue.remove(departuretime_);
		}
	}

	public Vehicle newqueueHead() {
		if (this.newqueue.size() > 0) {
			double firstDeparture_ = this.newqueue.firstKey();
			return this.newqueue.get(firstDeparture_).peek();
		}
		return null;
	}

	public double length() {
		return this.length;
	}

	public float getFreeSpeed() {
		return this.freeSpeed_;
	}

	public float calcSpeed() {
		if (nVehicles_ < 1)
			return speed_ = freeSpeed_;
		int nVehicles = 0;
		float sum = 0.0f;
		Vehicle pv = this.firstVehicle();
		while ((pv != null && pv.isOnLane())) { // LZ: Vehicle in junction is not taken into account
			sum += pv.currentSpeed();
			nVehicles += 1;
			pv = pv.macroTrailing();
		}
		if (nVehicles < 1 ){
			return speed_ = freeSpeed_;
		}
		return speed_ = sum / nVehicles;
	}

	public void calcLength() {
		Geography<Road> roadGeography = ContextCreator.getRoadGeography();
		Geometry roadGeom = roadGeography.getGeometry(this);
		this.length = (float) ContextCreator.convertToMeters(roadGeom
				.getLength());
		this.initDynamicTravelTime();
//		System.out
//				.println("Road " + this.linkid + " has length " + this.length);
	}
	
	/**
	 * This function set the current travel time of the road based on the
	 * average speed of the road.
	 * @author Zhan & Hemant
	 */
	public void setTravelTime() {
		float averageSpeed = 0;
		if (this.nVehicles_ == 0) {
			averageSpeed = (float) this.freeSpeed_;
		} else {
			Vehicle pv = this.firstVehicle();
			while (pv != null) {
				if (pv.currentSpeed() < 0) {
					logger.error("Vehicle " + pv.getId()
							+ " has error speed of " + pv.currentSpeed());
				} else
					averageSpeed = +pv.currentSpeed();
				pv = pv.macroTrailing();
			}
			if (averageSpeed < 0.001f) {
				averageSpeed = 0.001f;
			} else {
				if (this.nVehicles_ < 0) {
					logger.error("Road " + this.getLinkid() + " has "
							+ this.nVehicles_ + " vehicles");
					averageSpeed = (float) this.freeSpeed_;
				} else
					averageSpeed = averageSpeed / this.nVehicles_;
			}
		}
		// outAverageSpeed: For output travel times
//		DecimalFormat myFormatter = new DecimalFormat("##.##");

//		String outAverageSpeed = myFormatter.format(averageSpeed / 0.44704);

		this.travelTime = (float) this.length / averageSpeed;
	}
	

	/**
	 * This function will initialize the dynamic travel time vector of the road.
	 * @author Samiul Hasan
	 */
	public void initDynamicTravelTime() {
		int i = 1;
		int intervalLength = GlobalVariables.SIMULATION_INTERVAL_SIZE;
		int end = GlobalVariables.SIMULATION_STOP_TIME;
		while (i < end) {
			this.dynamicTravelTime.add(this.length / this.freeSpeed_);
			i = i + intervalLength;
		}
	}

	// TODO: check if it works
	public void setDynamicTravelTime(double tick, double time) {
		int intervalLength = GlobalVariables.SIMULATION_INTERVAL_SIZE;
		int interval = (int) tick / intervalLength;
		this.dynamicTravelTime.add(interval, time);
	}

	public double getDynamicTravelTime(double tick) {
		int intervalLength = GlobalVariables.SIMULATION_INTERVAL_SIZE;
		int interval = (int) tick / intervalLength;
		return this.dynamicTravelTime.get(interval);
	}

	public double getTravelTime() {
		return this.travelTime;
	}

	public Lane getLane(int i) {
		return this.lanes.get(i);
	}

	public int getLaneIndex(Lane l) {
		return this.lanes.indexOf(l);
	}

	public void addLane(Lane l) {
		this.lanes.add(l);
		this.sortLanes();
		this.nLanes++;
	}
	
	public void removeLane(Lane l) {
		if (this.lanes.remove(l)) {
			this.sortLanes();
			this.nLanes--;
		}
	}

	public ArrayList<Lane> getLanes() {
		return this.lanes;
	}

	public Lane leftLane() {
		return this.lanes.get(0);
	}

	public Lane rightLane() {
		int size = this.lanes.size();
		return this.lanes.get(size - 1);
	}

	public double atan3(double x1, double y1, double x0, double y0) {
		double alpha = 0;
		alpha = Math.atan2(y1 - y0, x1 - x0);
		if (alpha < 0) {
			alpha += 2 * Math.PI;
		}
		return alpha;
	}

	public int getnLanes() {
		return this.nLanes;
	}

	public Lane firstLane() {
		Lane firstLane = null;
		int rightmost = this.getLanes().size() - 1;
		firstLane = this.getLane(rightmost);
		return firstLane;
	}

	public void printRoadInfo() {
		logger.info("Road: " + this.getIdentifier()
				+ " has lanes from left to right as follow: ");
		for (int i = 0; i < this.lanes.size(); i++) {
			logger.info(this.lanes.get(i).getLaneid()
					+ " with Repast ID: " + this.lanes.get(i).getID());
		}
	}

	public void printRoadCoordinate() {
		Coordinate start, end;
		start = this.getJunctions().get(0).getCoordinate();
		end = this.getJunctions().get(1).getCoordinate();
		logger.info("Coordinate of road: " + this.getLinkid());
		logger.info("Starting point: " + start);
		logger.info("Ending point: " + end);
	}
	
	/**
	 * Wenbo: update bakcground traffic through speed file.
	 * if road event flag is true, just pass to default free speed,
	 * else, update link free flow speed
	 * */
	public void updateFreeFlowSpeed() {
		
		// Get current tick
		int tickcount = (int) RepastEssentials.GetTickCount();
		// double sim_time = tickcount*GlobalVariables.SIMULATION_STEP_SIZE;
		int hour = (int) Math.floor(tickcount
				* GlobalVariables.SIMULATION_STEP_SIZE / 3600);
		hour = hour % 24;
		//each hour set events
		if (this.curhour<hour) {
			float value = (float) (BackgroundTraffic.backgroundTraffic.get(this.linkid)
					.get(hour)* 0.44704); // HG: convert from miles per hour to meters per second
                  if (this.checkEventFlag()) {
						this.setDefaultFreeSpeed();
					}
                  else{
                	  this.freeSpeed_=value;
					}
			   }
		   
			
		//for (int i=0; i<backgroundtraffic.backgroundTraffic.size();i++) {
			//BackgroundTrafficObject e = backgroundtraffic.backgroundTraffic.get(i);
			//if (this.linkid == e.roadID) {
				// Found the road, and we do the change
				//if (!this.checkEventFlag()) {
					//this.freeSpeed_=e.value.get(hour);
				//} else {
					//this.setDefaultBackgroundSpeed(e.value.get(hour));			
						//}
				//}
		//}
		this.curhour=hour;
		}
		
	/**
	 * Modify the free flow speed based on the events
	 * */
	public void updateFreeFlowSpeed_event(float newFFSpd) {
		this.freeSpeed_ = (float) (newFFSpd* 0.44704); //HG: convert from Miles per hour to meter per second
	}
	
	public void printTick(){
		int tickcount = (int) RepastEssentials.GetTickCount();
		logger.info("Tick: "+tickcount);
	}
}
